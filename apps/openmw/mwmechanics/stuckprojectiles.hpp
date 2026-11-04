#ifndef GAME_MWMECHANICS_STUCKPROJECTILES_H
#define GAME_MWMECHANICS_STUCKPROJECTILES_H

// Fork feature "recoverable stuck projectiles" ([Modern Physics] in settings.cfg): an arrow, bolt or thrown weapon
// shown stuck in a body is a real one. A stored hit (fProjectileThrownStoreChance) is embedded: shown stuck, and
// kept out of the living actor's inventory (so it can't be shot back) until it dies, when it is added there. From
// then on, what is shown never outnumbers what the inventory holds of that ammo. Saved with the actor's stats.

#include <components/esm/refid.hpp>

#include <osg/Matrixf>
#include <osg/Vec3f>

#include <string>

namespace MWWorld
{
    class Ptr;
}

namespace MWRender
{
    class Animation;
}

namespace MWMechanics
{
    /// A projectile shown stuck in a body: the bone it is in, and where on that bone.
    struct StuckProjectile
    {
        ESM::RefId mAmmo;
        std::string mBone;
        osg::Matrixf mOnBone;
    };

    /// Whether the feature is on. Off, hits are shown stuck for a while only, and stored ones go straight into the
    /// victim's inventory.
    bool recoverableStuckProjectiles();

    /// projectileHit (see combat.hpp), and what stays in a body is shown stuck there (it flew along direction,
    /// normalized). A plain projectile hitting a dead body isn't an attack: it always stays in.
    bool projectileHitAndStick(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, const osg::Vec3f& direction,
        float attackStrength, float attackWindUp);

    /// A stored projectile hit the victim: embedded in a living one, straight into the inventory of a dead one.
    void embedProjectile(const MWWorld::Ptr& victim, const ESM::RefId& ammo);

    /// Show the projectile just embedded stuck where it hit (it flew along direction, normalized), and remember it.
    void showEmbeddedProjectile(const MWWorld::Ptr& victim, const MWWorld::Ptr& projectile,
        const osg::Vec3f& hitPosition, const osg::Vec3f& direction);

    /// The actor died: what is embedded in it goes into its inventory.
    void releaseEmbeddedProjectiles(const MWWorld::Ptr& actor);

    /// Show the remembered stuck projectiles on a newly built body.
    void restoreStuckProjectiles(const MWWorld::Ptr& actor, MWRender::Animation& animation);

    /// Stop showing ones that are no longer there (looted, taken by a script, ...). Cheap; called every frame.
    void updateStuckProjectiles(const MWWorld::Ptr& actor);
}

#endif
