#ifndef GAME_MWWORLD_IMPACTSOUNDS_H
#define GAME_MWWORLD_IMPACTSOUNDS_H

// Fork ("Morrowind - Modern Physics"): sounds where things hit, made from Morrowind's own sounds. What plays, how
// loud and how often is in resources/vfs/modernphysics/impactsounds.yaml; switches are in settings.cfg under
// [Modern Physics].

#include <deque>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <osg/Vec3f>

#include "materials.hpp"

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
        };

    private:
        ImpactSounds() = default;

        void loadConfig();
        void handleObjectImpact(const MWPhysics::Impact& impact, MWRender::RenderingManager& rendering);

        /// Whether a sound from these objects (keys; null ones ignored) may start now: not too soon after their
        /// last, and not too many at once. Starts their cooldown if so.
        bool allow(std::initializer_list<const void*> keys, std::string& why);

        void log(std::string&& entry);

        Limits mLimits;
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
