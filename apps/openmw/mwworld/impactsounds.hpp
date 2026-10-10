#ifndef GAME_MWWORLD_IMPACTSOUNDS_H
#define GAME_MWWORLD_IMPACTSOUNDS_H

// Fork ("Morrowind - Modern Physics"): sounds where things hit, made from Morrowind's own sounds. What plays, how
// loud and how often is in resources/vfs/modernphysics/impactsounds.yaml; switches are in settings.cfg under
// [Modern Physics].

#include <deque>
#include <filesystem>
#include <map>
#include <string_view>
#include <string>
#include <unordered_map>
#include <vector>

#include <osg/Vec3f>

#include <components/esm/refid.hpp>

#include "materials.hpp"
#include "ptr.hpp"

namespace MWPhysics
{
    class PhysicsSystem;
    struct Impact;
}

namespace MWRender
{
    class RenderingManager;
}

namespace MWWorld
{
    class ImpactSounds
    {
    public:
        static ImpactSounds& get();

        /// Once a frame, after physics: reload the configuration if it changed (in debug mode), and handle the
        /// impacts of simulated objects.
        void update(float duration, MWPhysics::PhysicsSystem& physics, MWRender::RenderingManager& rendering);

        /// An arrow, bolt or thrown weapon hit the world (not someone, not an item, not water) at position, and stuck
        /// in it or glanced off. speed: how fast it flew.
        void projectileHit(const osg::Vec3f& position, Material surface, bool stuck, float speed, std::string_view what);

        /// The last impacts and what was done about them, newest last (for tests and tuning).
        const std::deque<std::string>& getLog() const { return mLog; }
        void clearLog() { mLog.clear(); }

        struct Limits
        {
            float mMinSpeed = 80.f;
            float mLoudSpeed = 700.f;
            float mObjectCooldown = 0.15f;
            int mMaxSounds = 6;
            float mMaxSoundsWindow = 0.4f;
            float mMinDistance = 500.f; // full volume within
            float mMaxDistance = 4000.f; // silent beyond
            float mQuietest = 0.6f; // share of the volume for the softest impact that is heard
        };

        /// One kind of impact sound: any of these sound records or files (in the Sound folder), at a volume and
        /// pitch picked between their limits.
        struct Choice
        {
            std::vector<ESM::RefId> mSounds;
            std::vector<std::string> mFiles;
            float mVolume[2] = { 1.f, 1.f };
            float mPitch[2] = { 1.f, 1.f };
            bool empty() const { return mSounds.empty() && mFiles.empty(); }
        };
        /// Choices by the material hit; Material::Unknown is the default.
        using ByMaterial = std::map<Material, Choice>;

    private:
        ImpactSounds() = default;

        void loadConfig();
        void handleObjectImpact(const MWPhysics::Impact& impact, MWRender::RenderingManager& rendering);

        /// Whether a sound from these objects (keys; null ones ignored) may start now: not too soon after their
        /// last, and not too many at once. Starts their cooldown if so.
        bool allow(std::initializer_list<const void*> keys, float cooldown, std::string& why);

        void log(std::string&& entry);

        /// Play a choice at position, at strength (0 to 1: how hard the impact was) of its volume, if allowed for
        /// these objects. Returns what was played, or why nothing was, for the log.
        std::string play(const Choice& choice, const osg::Vec3f& position, float strength,
            std::initializer_list<const void*> keys, float cooldown = -1);

        /// What an item hitting something sounds like: by its material, deeper the heavier it is.
        Choice getItemChoice(Material material, float mass) const;

        Limits mLimits;
        float mProjectileLoudSpeed = 2500.f;
        ByMaterial mProjectileStick;
        ByMaterial mProjectileGlance;
        float mObjectLoudSpeed = 600.f;
        ByMaterial mItems;
        float mPitchMass = 2.f; // mass at normal pitch
        float mPitchByMass = 0.15f; // how much the pitch follows the mass
        float mPitchByMassLimits[2] = { 0.75f, 1.3f };
        ByMaterial mLandingOn; // volume and pitch factors by what is landed on
        float mBodyCooldown = 0.35f;
        float mBodyMediumSpeed = 250.f;
        float mBodyHardSpeed = 500.f;
        Choice mBodySoft;
        Choice mBodyMedium;
        Choice mBodyHard;
        bool mConfigLoaded = false;
        std::filesystem::file_time_type mConfigTime;
        float mConfigCheckTimer = 0;

        float mTime = 0;
        std::unordered_map<const void*, float> mLastSound; // per object
        std::deque<float> mRecentSounds; // start times
        std::deque<std::string> mLog;
    };
}

#endif
