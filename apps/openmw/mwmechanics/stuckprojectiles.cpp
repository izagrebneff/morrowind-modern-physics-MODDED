#include "stuckprojectiles.hpp"

#include <algorithm>
#include <exception>
#include <map>
#include <optional>
#include <vector>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadcrea.hpp>
#include <components/esm3/loadnpc.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwrender/animation.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/containerstore.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/manualref.hpp"
#include "../mwworld/ptr.hpp"

#include "actorutil.hpp"
#include "combat.hpp"
#include "creaturestats.hpp"

namespace MWMechanics
{
    namespace
    {
        // Shown at most; any more embedded or carried are still there to loot.
        constexpr size_t maxShown = 8;

        bool hasStats(const MWWorld::Ptr& actor)
        {
            return actor.getType() == ESM::NPC::sRecordId || actor.getType() == ESM::Creature::sRecordId;
        }

        // Embedded ones stay out of the inventory until notifyDied, which also makes the body lootable.
        bool inInventory(const CreatureStats& stats)
        {
            return stats.isDead() && stats.isDeathAnimationFinished();
        }

        void showAll(const MWWorld::Ptr& actor, MWRender::Animation& animation)
        {
            animation.removeStuckProjectiles();
            for (const StuckProjectile& projectile : actor.getClass().getCreatureStats(actor).getStuckProjectiles())
            {
                try
                {
                    MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), projectile.mAmmo);
                    animation.showStuckProjectile(
                        ref.getPtr().getClass().getCorrectedModel(ref.getPtr()), projectile.mBone, projectile.mOnBone);
                }
                catch (const std::exception& e)
                {
                    // Its record is gone (a mod removed); nothing to show.
                    Log(Debug::Verbose) << "Stuck projectile " << projectile.mAmmo << " not shown: " << e.what();
                }
            }
        }

        // Each ammo shown at most as many times as there is of it: embedded, or in the inventory.
        void match(const MWWorld::Ptr& actor, MWRender::Animation* animation)
        {
            CreatureStats& stats = actor.getClass().getCreatureStats(actor);
            std::vector<StuckProjectile>& shown = stats.getStuckProjectiles();
            if (shown.empty())
                return;
            const bool carried = inInventory(stats);
            std::map<ESM::RefId, int> left;
            for (const StuckProjectile& projectile : shown)
            {
                if (left.contains(projectile.mAmmo))
                    continue;
                if (carried)
                    left[projectile.mAmmo] = actor.getClass().getContainerStore(actor).count(projectile.mAmmo);
                else
                {
                    const auto embedded = stats.getEmbeddedAmmo().find(projectile.mAmmo);
                    left[projectile.mAmmo] = embedded == stats.getEmbeddedAmmo().end() ? 0 : embedded->second;
                }
            }
            // The newest of each stay.
            std::vector<StuckProjectile> keep;
            for (auto it = shown.rbegin(); it != shown.rend(); ++it)
                if (left[it->mAmmo]-- > 0)
                    keep.push_back(*it);
            if (keep.size() == shown.size())
                return;
            std::reverse(keep.begin(), keep.end());
            shown = std::move(keep);
            if (animation != nullptr)
                showAll(actor, *animation);
        }
    }

    bool recoverableStuckProjectiles()
    {
        return Settings::modernPhysics().mRecoverableStuckProjectiles;
    }

    bool projectileHitAndStick(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, const osg::Vec3f& direction,
        float attackStrength, float attackWindUp)
    {
        bool stored = false;
        bool hitBody = false;
        if (!victim.isEmpty() && hasStats(victim) && victim.getClass().getCreatureStats(victim).isDead()
            && projectile.getClass().getEnchantment(projectile).empty())
        {
            // By RefId: on a point-blank hit, projectile is the attacker's live ammo stack.
            embedProjectile(victim, projectile.getCellRef().getRefId());
            stored = hitBody = true;
        }
        else
            hitBody = projectileHit(
                attacker, victim, weapon, projectile, hitPosition, attackStrength, attackWindUp, &stored);
        if (stored && victim != getPlayer())
            showEmbeddedProjectile(victim, projectile, hitPosition, direction);
        return hitBody;
    }

    void embedProjectile(const MWWorld::Ptr& victim, const ESM::RefId& ammo)
    {
        CreatureStats& stats = victim.getClass().getCreatureStats(victim);
        if (inInventory(stats))
            victim.getClass().getContainerStore(victim).add(ammo, 1);
        else
            ++stats.getEmbeddedAmmo()[ammo];
    }

    void showEmbeddedProjectile(const MWWorld::Ptr& victim, const MWWorld::Ptr& projectile,
        const osg::Vec3f& hitPosition, const osg::Vec3f& direction)
    {
        MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(victim);
        if (animation == nullptr || !hasStats(victim))
            return;
        const std::optional<MWRender::Animation::StuckProjectilePlacement> placement
            = animation->findStuckProjectilePlacement(
                projectile.getClass().getCorrectedModel(projectile), hitPosition, direction);
        if (!placement)
            return;
        std::vector<StuckProjectile>& shown = victim.getClass().getCreatureStats(victim).getStuckProjectiles();
        shown.push_back({ projectile.getCellRef().getRefId(), placement->mBone, placement->mOnBone });
        if (shown.size() > maxShown)
            shown.erase(shown.begin(), shown.end() - maxShown);
        showAll(victim, *animation);
    }

    void releaseEmbeddedProjectiles(const MWWorld::Ptr& actor)
    {
        if (!hasStats(actor))
            return;
        std::map<ESM::RefId, int> embedded;
        std::swap(embedded, actor.getClass().getCreatureStats(actor).getEmbeddedAmmo());
        for (const auto& [ammo, count] : embedded)
        {
            try
            {
                actor.getClass().getContainerStore(actor).add(ammo, count);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Embedded projectile " << ammo << " lost: " << e.what();
            }
        }
    }

    void restoreStuckProjectiles(const MWWorld::Ptr& actor, MWRender::Animation& animation)
    {
        if (!hasStats(actor))
            return;
        CreatureStats& stats = actor.getClass().getCreatureStats(actor);
        // Died before being saved or unloaded, before its death was noticed: what is in it is its own now.
        if (inInventory(stats) && !stats.getEmbeddedAmmo().empty())
            releaseEmbeddedProjectiles(actor);
        if (stats.getStuckProjectiles().empty())
            return;
        showAll(actor, animation);
        match(actor, &animation);
    }

    void updateStuckProjectiles(const MWWorld::Ptr& actor)
    {
        if (!hasStats(actor) || actor.getClass().getCreatureStats(actor).getStuckProjectiles().empty())
            return;
        match(actor, MWBase::Environment::get().getWorld()->getAnimation(actor));
    }
}
