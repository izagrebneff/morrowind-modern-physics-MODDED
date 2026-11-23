#include <components/misc/brutalconstants.hpp>
#include "mtphysics.hpp"
#include "ragdollstep.hpp"
#include "superkick.hpp"
#include "limbsevering.hpp"
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <cassert>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <variant>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btThreads.h>

#include <osg/Stats>

#include "components/debug/debuglog.hpp"
#include "components/misc/convert.hpp"
#include <components/misc/barrier.hpp>
#include <components/settings/values.hpp>

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"

#include "../mwrender/bulletdebugdraw.hpp"

#include "../mwworld/class.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/world.hpp"

#include "actor.hpp"
#include "contacttestwrapper.h"
#include "movementsolver.hpp"
#include "playerkinetics.hpp"
#include "object.hpp"
#include "physicssystem.hpp"
#include "projectile.hpp"

namespace MWPhysics
{
    namespace
    {
        template <class Mutex>
        std::optional<std::unique_lock<Mutex>> makeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::unique_lock(mutex);
        }

        /// @brief A scoped lock that is either exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeExclusiveLock
        {
        public:
            /// @param mutex a mutex
            /// @param threadCount decide wether the excluse lock will be taken
            explicit MaybeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeExclusiveLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::unique_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::optional<std::shared_lock<Mutex>> makeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::shared_lock(mutex);
        }

        /// @brief A scoped lock that is either shared or inexistent depending on configuration
        template <class Mutex>
        class MaybeSharedLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the shared lock will be taken
            explicit MaybeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeSharedLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::shared_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> makeLock(
            Mutex& mutex, LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return std::monostate{};
                case LockingPolicy::ExclusiveLocksOnly:
                    return std::unique_lock(mutex);
                case LockingPolicy::AllowSharedLocks:
                    return std::shared_lock(mutex);
            };

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }

        /// @brief A scoped lock that is either shared, exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the lock will be shared, exclusive or inexistent
            explicit MaybeLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeLock(mutex, lockingPolicy))
            {
            }

        private:
            std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> mImpl;
        };
    }
}

namespace
{
    bool isUnderWater(const MWPhysics::ActorFrameData& actorData)
    {
        return actorData.mPosition.z() < actorData.mSwimLevel;
    }

    osg::Vec3f interpolateMovements(const MWPhysics::PtrHolder& ptr, float timeAccum, float physicsDt)
    {
        const float interpolationFactor = std::clamp(timeAccum / physicsDt, 0.0f, 1.0f);
        return ptr.getPosition() * interpolationFactor + ptr.getPreviousPosition() * (1.f - interpolationFactor);
    }

    using LockedActorSimulation
        = std::pair<std::shared_ptr<MWPhysics::Actor>, std::reference_wrapper<MWPhysics::ActorFrameData>>;
    using LockedProjectileSimulation
        = std::pair<std::shared_ptr<MWPhysics::Projectile>, std::reference_wrapper<MWPhysics::ProjectileFrameData>>;

    namespace Visitors
    {
        template <class Impl, template <class> class Lock>
        struct WithLockedPtr
        {
            const Impl& mImpl;
            std::shared_mutex& mCollisionWorldMutex;
            const MWPhysics::LockingPolicy mLockingPolicy;

            template <class Ptr, class FrameData>
            void operator()(MWPhysics::SimulationImpl<Ptr, FrameData>& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto&& [ptr, frameData] = *std::move(locked);
                // Locked shared_ptr has to be destructed after releasing mCollisionWorldMutex to avoid
                // possible deadlock. Ptr destructor also acquires mCollisionWorldMutex.
                const std::pair arg(std::move(ptr), frameData);
                const Lock<std::shared_mutex> lock(mCollisionWorldMutex, mLockingPolicy);
                mImpl(arg);
            }
        };

        struct InitPosition
        {
            btCollisionWorld* mCollisionWorld;
            const bool mAdvanceSimulation;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                frameData.mPosition = actor->applyOffsetChange();
                if (frameData.mIsPlayer)
                {
                    // prepareSimulation snapshots requests before the previous
                    // batch joins. Shape changes belong here, after Sync and
                    // while prepareWork holds the simulation mutex on main.
                    auto& stats = actor->getPtr().getClass().getCreatureStats(actor->getPtr());
                    stats.mBrutalSlideBlocked = actor->updateSlideCollision(frameData.mBrutalSlideLow, mCollisionWorld);
                    frameData.mHalfExtentsZ = actor->getHalfExtents().z();
                }
                if (frameData.mWaterCollision && frameData.mPosition.z() < frameData.mWaterlevel
                    && actor->canMoveToWaterSurface(frameData.mWaterlevel, mCollisionWorld))
                {
                    const auto offset = osg::Vec3f(0, 0, frameData.mWaterlevel - frameData.mPosition.z());
                    MWBase::Environment::get().getWorld()->moveObjectBy(actor->getPtr(), offset, false);
                    frameData.mPosition = actor->applyOffsetChange();
                }
                actor->updateCollisionObjectPosition();
                frameData.mOldHeight = frameData.mPosition.z();
                const auto rotation = actor->getPtr().getRefData().getPosition().asRotationVec3();
                frameData.mRotation = osg::Vec2f(rotation.x(), rotation.z());
                frameData.mInertia = actor->getInertialForce();
                frameData.mExternalVelocity = actor->getExternalVelocity();
                frameData.mLocomotionVelocity = actor->getLocomotionVelocity();
                // Never consume an increment during a render frame with no physics
                // step: Sync deliberately does not publish results for such frames.
                if (mAdvanceSimulation)
                {
                    bool clear = false;
                    const osg::Vec3f delta = actor->consumeExternalVelocity(clear);
                    if (clear)
                    {
                        frameData.mExternalVelocity = osg::Vec3f();
                        frameData.mLocomotionVelocity = osg::Vec3f();
                    }
                    if (actor->updateSlidingState(frameData.mBrutalSliding && !clear)
                        && frameData.mIsOnGround && !frameData.mIsOnSlope && !frameData.mInert
                        && !frameData.mFlying && !isUnderWater(frameData))
                    {
                        auto& moving = frameData.mLocomotionVelocity;
                        const float speed = std::hypot(moving.x(), moving.y());
                        const float target = std::min(speed * MWPhysics::PlayerKinetics::SlideEntrySpeedMultiplier,
                            MWPhysics::PlayerKinetics::SprintSpeed * MWPhysics::PlayerKinetics::SlideEntrySpeedMultiplier);
                        if (speed > 0.f && target > speed)
                        {
                            moving.x() *= target / speed;
                            moving.y() *= target / speed;
                        }
                    }
                    frameData.mSlideActorContactTime = actor->getSlideActorContactTime();
                    frameData.mExternalVelocity += delta;
                    const float speed = frameData.mExternalVelocity.length();
                    if (speed > 2000.f)
                        frameData.mExternalVelocity *= 2000.f / speed;
                    if (!frameData.mFlying && !isUnderWater(frameData))
                    {
                        // Let vanilla gravity, landing and fall damage own the
                        // vertical component. Horizontal motion remains independent
                        // of movement controls, jumping and animation root motion.
                        frameData.mInertia.z() += frameData.mExternalVelocity.z();
                        if (frameData.mExternalVelocity.z() != 0.f)
                            frameData.mIsOnGround = false;
                        frameData.mExternalVelocity.z() = 0.f;
                    }
                }
                frameData.mStuckFrames = actor->getStuckFrames();
                frameData.mLastStuckPosition = actor->getLastStuckPosition();
                actor->setCanReceiveSlideKnockdown(!frameData.mIsPlayer
                    && !frameData.mInert && !frameData.mFlying && !isUnderWater(frameData)
                    && !frameData.mSkipCollisionDetection && !actor->isRagdoll()
                    && frameData.mInertia.z() <= 0.f && frameData.mMovement.z() <= 0.f
                    && actor->getOnGround() && !actor->getOnSlope() && frameData.mIsOnGround);
            }
            void operator()(MWPhysics::ProjectileSimulation& /*sim*/) const {}
        };

        struct PreStep
        {
            btCollisionWorld* mCollisionWorld;
            void operator()(const LockedActorSimulation& sim) const
            {
                MWPhysics::MovementSolver::unstuck(sim.second, mCollisionWorld);
            }
            void operator()(const LockedProjectileSimulation& /*sim*/) const {}
        };

        struct UpdatePosition
        {
            btCollisionWorld* mCollisionWorld;
            void operator()(const LockedActorSimulation& sim) const
            {
                auto& [actor, frameDataRef] = sim;
                auto& frameData = frameDataRef.get();
                if (actor->setPosition(frameData.mPosition))
                {
                    frameData.mPosition = actor->getPosition(); // account for potential position change made by script
                    actor->updateCollisionObjectPosition();
                    mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
                }
            }
            void operator()(const LockedProjectileSimulation& sim) const
            {
                auto& [proj, frameDataRef] = sim;
                auto& frameData = frameDataRef.get();
                proj->setPosition(frameData.mPosition);
                proj->updateCollisionObjectPosition();
                mCollisionWorld->updateSingleAabb(proj->getCollisionObject());
            }
        };

        struct Move
        {
            const float mPhysicsDt;
            const btCollisionWorld* mCollisionWorld;
            const MWPhysics::WorldFrameData& mWorldFrameData;
            void operator()(const LockedActorSimulation& sim) const
            {
                MWPhysics::MovementSolver::move(sim.second, mPhysicsDt, mCollisionWorld, mWorldFrameData);
            }
            void operator()(const LockedProjectileSimulation& sim) const
            {
                if (sim.first->isActive())
                    MWPhysics::MovementSolver::move(sim.second, mPhysicsDt, mCollisionWorld);
            }
        };

        struct Sync
        {
            const bool mAdvanceSimulation;
            const float mTimeAccum;
            const float mPhysicsDt;
            const MWPhysics::PhysicsTaskScheduler* scheduler;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                auto ptr = actor->getPtr();

                MWMechanics::CreatureStats& stats = ptr.getClass().getCreatureStats(ptr);
                const float heightDiff = frameData.mPosition.z() - frameData.mOldHeight;
                const bool isStillOnGround = (mAdvanceSimulation && frameData.mWasOnGround && frameData.mIsOnGround);

                if (isStillOnGround || frameData.mFlying || isUnderWater(frameData) || frameData.mSlowFall < 1)
                    stats.land(ptr == MWMechanics::getPlayer() && (frameData.mFlying || isUnderWater(frameData)));
                else if (heightDiff < 0)
                    stats.addToFallHeight(-heightDiff);

                const bool positionAccepted
                    = actor->setSimulationPosition(::interpolateMovements(*actor, mTimeAccum, mPhysicsDt));
                if (!positionAccepted)
                    actor->setCanReceiveSlideKnockdown(false);
                actor->setLastStuckPosition(frameData.mLastStuckPosition);
                actor->setStuckFrames(frameData.mStuckFrames);
                if (mAdvanceSimulation)
                {
                    MWWorld::Ptr standingOn;
                    if (frameData.mStandingOn != nullptr)
                    {
                        auto* const ptrHolder
                            = static_cast<MWPhysics::PtrHolder*>(scheduler->getUserPointer(frameData.mStandingOn));
                        if (ptrHolder != nullptr)
                            standingOn = ptrHolder->getPtr();
                    }
                    actor->setStandingOnPtr(standingOn);
                    // the "on ground" state of an actor might have been updated by a traceDown, don't overwrite the
                    // change
                    if (actor->getOnGround() == frameData.mWasOnGround)
                        actor->setOnGround(frameData.mIsOnGround);
                    actor->setOnSlope(frameData.mIsOnSlope);
                    actor->setWalkingOnWater(frameData.mWalkingOnWater);
                    actor->setInertialForce(frameData.mInertia);
                    actor->setExternalVelocity(frameData.mExternalVelocity);
                    actor->setLocomotionVelocity(frameData.mLocomotionVelocity);
                    actor->setSlideActorContactTime(positionAccepted ? frameData.mSlideActorContactTime : 0.f);
                    if (frameData.mIsPlayer)
                    {
                        // Publish only after workers finish, with the same accepted
                        // position as the support/travel result. A cancelled slide
                        // or pre-teleport snapshot must not revive continuation.
                        const bool sliding = positionAccepted && stats.mBrutalSliding && frameData.mBrutalSliding;
                        const bool lowSupport = positionAccepted && (stats.mBrutalSliding || stats.mBrutalSlideLow)
                            && (frameData.mBrutalSliding || frameData.mBrutalSlideLow);
                        stats.mBrutalSlideDownhill = sliding && frameData.mBrutalSlideDownhill;
                        stats.mBrutalSlideGroundNormal = lowSupport ? frameData.mBrutalSlideGroundNormal : osg::Vec3f();
                        stats.mBrutalSlideGroundPoint = lowSupport ? frameData.mBrutalSlideGroundPoint : osg::Vec3f();
                    }
                    // A pre-teleport snapshot must not restore the old ragdoll
                    // inheritance velocity after its position was discarded.
                    if (positionAccepted)
                        actor->setSimulationVelocity(frameData.mSimulationVelocity);
                    if (frameData.mExternalImpactSpeed > 1.f && !actor->isExternalVelocityResetPending()
                        && actor->isActive() && !stats.isDead())
                    {
                        MWWorld::Ptr hitObject;
                        if (auto* holder = static_cast<MWPhysics::PtrHolder*>(
                                scheduler->getUserPointer(frameData.mExternalImpactObject)))
                            hitObject = holder->getPtr();
                        MWBase::Environment::get().getLuaManager()->actorExternalImpact(ptr, hitObject,
                            frameData.mExternalImpactPosition, frameData.mExternalImpactNormal,
                            frameData.mExternalImpactSpeed);
                    }
                }
            }
            void operator()(MWPhysics::ProjectileSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [proj, frameData] = *locked;
                proj->setSimulationPosition(::interpolateMovements(*proj, mTimeAccum, mPhysicsDt));
            }
        };
    }
}

namespace MWPhysics
{
    namespace
    {
        unsigned getMaxBulletSupportedThreads()
        {
            auto broad = std::make_unique<btDbvtBroadphase>();
            assert(BT_MAX_THREAD_COUNT > 0);
            return std::min<unsigned>(broad->m_rayTestStacks.size(), BT_MAX_THREAD_COUNT - 1);
        }

        LockingPolicy detectLockingPolicy()
        {
            if (Settings::physics().mAsyncNumThreads < 1)
                return LockingPolicy::NoLocks;
            if (getMaxBulletSupportedThreads() > 1)
                return LockingPolicy::AllowSharedLocks;
            Log(Debug::Warning) << "Bullet was not compiled with multithreading support, 1 async thread will be used";
            return LockingPolicy::ExclusiveLocksOnly;
        }

        unsigned getNumThreads(LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return 0;
                case LockingPolicy::ExclusiveLocksOnly:
                    return 1;
                case LockingPolicy::AllowSharedLocks:
                    return static_cast<unsigned>(std::clamp<int>(
                        Settings::physics().mAsyncNumThreads, 0, static_cast<int>(getMaxBulletSupportedThreads())));
            }

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }
    }

    class PhysicsTaskScheduler::WorkersSync
    {
    public:
        void waitForWorkers()
        {
            std::unique_lock lock(mWorkersDoneMutex);
            if (mFrameCounter != mWorkersFrameCounter)
                mWorkersDone.wait(lock);
        }

        void wakeUpWorkers()
        {
            const std::lock_guard lock(mHasJobMutex);
            ++mFrameCounter;
            mHasJob.notify_all();
        }

        void stopWorkers()
        {
            const std::lock_guard lock(mHasJobMutex);
            mShouldStop = true;
            mHasJob.notify_all();
        }

        void workIsDone()
        {
            const std::lock_guard lock(mWorkersDoneMutex);
            ++mWorkersFrameCounter;
            mWorkersDone.notify_all();
        }

        template <class F>
        void runWorker(F&& f) noexcept
        {
            std::size_t lastFrame = 0;
            std::unique_lock lock(mHasJobMutex);
            while (!mShouldStop)
            {
                mHasJob.wait(lock, [&] { return mShouldStop || mFrameCounter != lastFrame; });
                lastFrame = mFrameCounter;
                lock.unlock();
                f();
                lock.lock();
            }
        }

    private:
        std::size_t mWorkersFrameCounter = 0;
        std::condition_variable mWorkersDone;
        std::mutex mWorkersDoneMutex;
        std::condition_variable mHasJob;
        bool mShouldStop = false;
        std::size_t mFrameCounter = 0;
        std::mutex mHasJobMutex;
    };

    PhysicsTaskScheduler::PhysicsTaskScheduler(
        float physicsDt, btCollisionWorld* collisionWorld, MWRender::DebugDrawer* debugDrawer)
        : mDefaultPhysicsDt(physicsDt)
        , mPhysicsDt(physicsDt)
        , mTimeAccum(0.f)
        , mCollisionWorld(collisionWorld)
        , mDebugDrawer(debugDrawer)
        , mLockingPolicy(detectLockingPolicy())
        , mNumThreads(getNumThreads(mLockingPolicy))
        , mNumJobs(0)
        , mRemainingSteps(0)
        , mLOSCacheExpiry(Settings::physics().mLineofsightKeepInactiveCache)
        , mAdvanceSimulation(false)
        , mNextJob(0)
        , mNextLOS(0)
        , mFrameNumber(0)
        , mTimer(osg::Timer::instance())
        , mPrevStepCount(1)
        , mBudget(physicsDt)
        , mAsyncBudget(0.0f)
        , mBudgetCursor(0)
        , mAsyncStartTime(0)
        , mTimeBegin(0)
        , mTimeEnd(0)
        , mFrameStart(0)
        , mWorkersSync(mNumThreads >= 1 ? std::make_unique<WorkersSync>() : nullptr)
    {
        mRagdollBaselineSolverMode = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld)->getSolverInfo().m_solverMode;
        if (mNumThreads >= 1)
        {
            Log(Debug::Info) << "Using " << mNumThreads << " async physics threads";
            for (unsigned i = 0; i < mNumThreads; ++i)
                mThreads.emplace_back([&] { worker(); });
        }
        else
        {
            mLOSCacheExpiry = 0;
        }

        mPreStepBarrier = std::make_unique<Misc::Barrier>(mNumThreads);

        mPostStepBarrier = std::make_unique<Misc::Barrier>(mNumThreads);

        mPostSimBarrier = std::make_unique<Misc::Barrier>(mNumThreads);
    }

    PhysicsTaskScheduler::~PhysicsTaskScheduler()
    {
        waitForWorkers();
        {
            MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);
            mNumJobs = 0;
            mRemainingSteps = 0;
        }
        if (mWorkersSync != nullptr)
            mWorkersSync->stopWorkers();
        for (auto& thread : mThreads)
            thread.join();
    }

    std::tuple<unsigned, float> PhysicsTaskScheduler::calculateStepConfig(float timeAccum) const
    {
        unsigned maxAllowedSteps = 2;
        unsigned numSteps = static_cast<unsigned>(timeAccum / mDefaultPhysicsDt);

        // adjust maximum step count based on whether we're likely physics bottlenecked or not
        // if maxAllowedSteps ends up higher than numSteps, we will not invoke delta time
        // if it ends up lower than numSteps, but greater than 1, we will run a number of true delta time physics steps
        // that we expect to be within budget if it ends up lower than numSteps and also 1, we will run a single delta
        // time physics step if we did not do this, and had a fixed step count limit, we would have an unnecessarily low
        // render framerate if we were only physics bottlenecked, and we would be unnecessarily invoking true delta time
        // if we were only render bottlenecked

        // get physics timing stats
        float budgetMeasurement = std::max(mBudget.get(), mAsyncBudget.get());
        // time spent per step in terms of the intended physics framerate
        budgetMeasurement /= mDefaultPhysicsDt;
        // ensure sane minimum value
        budgetMeasurement = std::max(0.00001f, budgetMeasurement);
        // we're spending almost or more than realtime per physics frame; limit to a single step
        if (budgetMeasurement > 0.95f)
            maxAllowedSteps = 1;
        // physics is fairly cheap; limit based on expense
        if (budgetMeasurement < 0.5f)
            maxAllowedSteps = static_cast<unsigned>(std::ceil(1.f / budgetMeasurement));
        // limit to a reasonable amount
        maxAllowedSteps = std::min(10u, maxAllowedSteps);

        // fall back to delta time for this frame if fixed timestep physics would fall behind
        float actualDelta = mDefaultPhysicsDt;
        if (numSteps > maxAllowedSteps)
        {
            numSteps = maxAllowedSteps;
            // ensure that we do not simulate a frame ahead when doing delta time; this reduces stutter and latency
            // this causes interpolation to 100% use the most recent physics result when true delta time is happening
            // and we deliberately simulate up to exactly the timestamp that we want to render
            actualDelta = timeAccum / float(numSteps + 1);
            // actually: if this results in a per-step delta less than the target physics steptime, clamp it
            // this might reintroduce some stutter, but only comes into play in obscure cases
            // (because numSteps is originally based on mDefaultPhysicsDt, this won't cause us to overrun)
            actualDelta = std::max(actualDelta, mDefaultPhysicsDt);
        }

        return std::make_tuple(numSteps, actualDelta);
    }

    void PhysicsTaskScheduler::applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        assert(mSimulations != &simulations);

        waitForWorkers();
        prepareWork(timeAccum, simulations, frameStart, frameNumber, stats);
        if (mWorkersSync != nullptr)
            mWorkersSync->wakeUpWorkers();
    }

    void PhysicsTaskScheduler::prepareWork(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        // This function run in the main thread.
        // While the mSimulationMutex is held, background physics threads can't run.

        MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);

        auto timeStart = mTimer->tick();

        // start by finishing previous background computation
        if (mNumThreads != 0)
        {
            syncWithMainThread();

            if (mAdvanceSimulation)
                mAsyncBudget.update(mTimer->delta_s(mAsyncStartTime, mTimeEnd), mPrevStepCount, mBudgetCursor);
            updateStats(frameStart, frameNumber, stats);
        }

        auto [numSteps, newDelta] = calculateStepConfig(timeAccum);
        timeAccum -= numSteps * newDelta;

        // init
        const Visitors::InitPosition vis{ mCollisionWorld, numSteps != 0 };
        for (auto& sim : simulations)
        {
            std::visit(vis, sim);
        }
        mPrevStepCount = numSteps;
        mRemainingSteps = numSteps;
        mTimeAccum = timeAccum;
        mPhysicsDt = newDelta;
        mSimulations = &simulations;
        mAdvanceSimulation = (mRemainingSteps != 0);
        {
            MaybeExclusiveLock worldLock(mCollisionWorldMutex, mLockingPolicy);
            // Until pending steps finish, render only the latest completed pose.
            // Publish the clock with the history, never read the worker's mutable
            // accumulator separately from its rigid-body transforms.
            mRagdollInterpolation = std::clamp(mTimeAccum / mPhysicsDt + float(mRemainingSteps), 0.f, 1.f);
        }
        mNumJobs = static_cast<int>(mSimulations->size());
        mNextLOS.store(0, std::memory_order_relaxed);
        mNextJob.store(0, std::memory_order_release);

        if (mAdvanceSimulation)
            mWorldFrameData = std::make_unique<WorldFrameData>();

        if (mAdvanceSimulation)
            mBudgetCursor += 1;

        if (mNumThreads == 0)
        {
            doSimulation();
            syncWithMainThread();
            if (mAdvanceSimulation)
                mBudget.update(mTimer->delta_s(timeStart, mTimer->tick()), numSteps, mBudgetCursor);
            return;
        }

        mAsyncStartTime = mTimer->tick();
        if (mAdvanceSimulation)
            mBudget.update(mTimer->delta_s(timeStart, mTimer->tick()), 1, mBudgetCursor);
    }

    void PhysicsTaskScheduler::resetSimulation(const ActorMap& actors)
    {
        waitForWorkers();
        MaybeExclusiveLock lock(mSimulationMutex, mLockingPolicy);
        mBudget.reset(mDefaultPhysicsDt);
        mAsyncBudget.reset(0.0f);
        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        for (const auto& [_, actor] : actors)
        {
            actor->updatePosition();
            auto& stats = actor->getPtr().getClass().getCreatureStats(actor->getPtr());
            stats.mBrutalSlideRequested = false;
            stats.mBrutalSliding = false;
            stats.mBrutalSlideLow = false;
            stats.mBrutalSlideDownhill = false;
            stats.mBrutalSlideGroundNormal = osg::Vec3f();
            stats.mBrutalSlideGroundPoint = osg::Vec3f();
            stats.mBrutalSlideBlocked = actor->updateSlideCollision(false, mCollisionWorld);
            actor->updateCollisionObjectPosition();
        }
    }

    void PhysicsTaskScheduler::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld,
        btCollisionWorld::RayResultCallback& resultCallback) const
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->rayTest(rayFromWorld, rayToWorld, resultCallback);
    }

    void PhysicsTaskScheduler::convexSweepTest(const btConvexShape* castShape, const btTransform& from,
        const btTransform& to, btCollisionWorld::ConvexResultCallback& resultCallback) const
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->convexSweepTest(castShape, from, to, resultCallback);
    }

    void PhysicsTaskScheduler::contactTest(
        btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        ContactTestWrapper::contactTest(mCollisionWorld, colObj, resultCallback);
    }

    std::optional<btVector3> PhysicsTaskScheduler::getHitPoint(const btTransform& from, btCollisionObject* target)
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        // target the collision object's world origin, this should be the center of the collision object
        btTransform rayTo;
        rayTo.setIdentity();
        rayTo.setOrigin(target->getWorldTransform().getOrigin());

        btCollisionWorld::ClosestRayResultCallback cb(from.getOrigin(), rayTo.getOrigin());

        mCollisionWorld->rayTestSingle(
            from, rayTo, target, target->getCollisionShape(), target->getWorldTransform(), cb);
        if (!cb.hasHit())
            // didn't hit the target. this could happen if point is already inside the collision box
            return std::nullopt;
        return { cb.m_hitPointWorld };
    }

    void PhysicsTaskScheduler::aabbTest(
        const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->getBroadphase()->aabbTest(aabbMin, aabbMax, callback);
    }

    void PhysicsTaskScheduler::getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max)
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        obj->getCollisionShape()->getAabb(obj->getWorldTransform(), min, max);
    }

    void PhysicsTaskScheduler::setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        collisionObject->getBroadphaseHandle()->m_collisionFilterMask = collisionFilterMask;
    }

    void PhysicsTaskScheduler::addCollisionObject(
        btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mCollisionObjects.insert(collisionObject);
        mCollisionWorld->addCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
    }

    // Dynamic object bodies: derived from sunspotdark/morrowind-modern-physics
    // (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40), GPLv3.
    void PhysicsTaskScheduler::applyObjectImpulse(
        const std::shared_ptr<Object>& object, const osg::Vec3f& impulse, float mass)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        if (object->commitPositionChange())
            resetDynamicObjectHistory(*object);
        auto* old = object->getCollisionObject();
        if (!btRigidBody::upcast(old))
        {
            mCollisionObjects.erase(old);
            world->removeCollisionObject(old);
            object->makeDynamic(mass);
            auto* body = btRigidBody::upcast(object->getCollisionObject());
            world->addRigidBody(body, CollisionType_World | CollisionType_Dynamic,
                CollisionType_World | CollisionType_Dynamic | CollisionType_Door | CollisionType_HeightMap | CollisionType_Actor | CollisionType_Projectile);
            mCollisionObjects.insert(body);
            mDynamicObjects.emplace_back(object);
            mDynamicTransforms.emplace(body, DynamicTransformHistory(body->getWorldTransform()));
        }
        object->applyDynamicImpulse(impulse, mass);
    }

    namespace
    {
        bool releasedGripOverlaps(btCollisionWorld& world, btCollisionObject* item, btCollisionObject* grip)
        {
            struct Result final : btCollisionWorld::ContactResultCallback
            {
                bool overlaps = false;
                bool needsCollision(btBroadphaseProxy*) const override { return true; }
                btScalar addSingleResult(btManifoldPoint& point, const btCollisionObjectWrapper*, int, int,
                    const btCollisionObjectWrapper*, int, int) override
                {
                    overlaps = overlaps || point.getDistance() <= 0.f;
                    return 0.f;
                }
            } result;
            // Explicit pair query intentionally bypasses its temporary broadphase exclusion.
            world.contactPairTest(item, grip, result);
            return result.overlaps;
        }

        void invalidateReleasedPairs(btCollisionWorld& world, btCollisionObject* item)
        {
            if (auto* proxy = item->getBroadphaseHandle())
                world.getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(proxy, world.getDispatcher());
            item->activate(true);
        }
    }

    void PhysicsTaskScheduler::addReleasedObject(
        const std::shared_ptr<Object>& object, const std::vector<osg::Vec3f>& vertices, float mass)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        object->commitPositionChange();
        auto* old = object->getCollisionObject();
        if (btRigidBody::upcast(old))
            throw std::runtime_error("Released equipment registered twice");
        mCollisionObjects.erase(old);
        world->removeCollisionObject(old);
        object->makeReleasedDynamic(mass, vertices, osg::Vec3f(), osg::Vec3f());
        auto* body = btRigidBody::upcast(object->getCollisionObject());
        // Loose equipment follows the ragdoll's actor-filter policy. Kinematic
        // actor sweeps must not treat its moving hull as an immovable wall.
        // World/dynamic contacts and ordinary physical ray queries stay enabled.
        world->addRigidBody(body, CollisionType_World | CollisionType_Dynamic,
            CollisionType_World | CollisionType_Dynamic | CollisionType_Door | CollisionType_HeightMap
                | CollisionType_Projectile);
        mCollisionObjects.insert(body);
        mDynamicObjects.emplace_back(object);
        mDynamicTransforms.emplace(body, DynamicTransformHistory(body->getWorldTransform()));
    }

    void PhysicsTaskScheduler::setReleasedObjectMotion(const std::shared_ptr<Object>& object,
        const osg::Vec3f& linear, const osg::Vec3f& angular, const std::vector<btRigidBody*>& gripBodies)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* body = btRigidBody::upcast(object->getCollisionObject());
        if (!body)
            throw std::runtime_error("Released equipment has no rigid body");
        if (object->commitPositionChange())
            resetDynamicObjectHistory(*object);
        const auto omega = Misc::Convert::toBullet(angular);
        const auto offset = body->getWorldTransform().getOrigin() - object->getTransform().getOrigin();
        const auto velocity = Misc::Convert::toBullet(linear) + omega.cross(offset);
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(velocity[axis]) || !std::isfinite(omega[axis]))
                throw std::runtime_error("Non-finite released equipment motion");
        if (velocity.length2() > 2000.f * 2000.f || omega.length2() > 200.f * 200.f)
            throw std::runtime_error("Released equipment motion exceeds numeric bound");
        body->setLinearVelocity(velocity);
        body->setAngularVelocity(omega);
        body->setInterpolationLinearVelocity(velocity);
        body->setInterpolationAngularVelocity(omega);
        body->activate(true);
        mReleasedGripPairs.reserve(mReleasedGripPairs.size() + gripBodies.size());
        for (auto* grip : gripBodies)
            if (grip && releasedGripOverlaps(*mCollisionWorld, body, grip))
            {
                body->setIgnoreCollisionCheck(grip, true);
                grip->setIgnoreCollisionCheck(body, true);
                mReleasedGripPairs.push_back({ body, grip });
                invalidateReleasedPairs(*mCollisionWorld, grip);
            }
        invalidateReleasedPairs(*mCollisionWorld, body);
    }

    void PhysicsTaskScheduler::clearReleasedGripPairs(btCollisionObject* removed)
    {
        for (auto it = mReleasedGripPairs.begin(); it != mReleasedGripPairs.end();)
            if (it->item == removed || it->grip == removed)
            {
                it->item->setIgnoreCollisionCheck(it->grip, false);
                it->grip->setIgnoreCollisionCheck(it->item, false);
                invalidateReleasedPairs(*mCollisionWorld, it->item);
                invalidateReleasedPairs(*mCollisionWorld, it->grip);
                it = mReleasedGripPairs.erase(it);
            }
            else
                ++it;
    }

    void PhysicsTaskScheduler::updateReleasedGripPairs()
    {
        for (auto it = mReleasedGripPairs.begin(); it != mReleasedGripPairs.end();)
            if (!releasedGripOverlaps(*mCollisionWorld, it->item, it->grip))
            {
                it->item->setIgnoreCollisionCheck(it->grip, false);
                it->grip->setIgnoreCollisionCheck(it->item, false);
                invalidateReleasedPairs(*mCollisionWorld, it->item);
                invalidateReleasedPairs(*mCollisionWorld, it->grip);
                it = mReleasedGripPairs.erase(it);
            }
            else
                ++it;
    }

    std::vector<osg::Vec3f> PhysicsTaskScheduler::readReleasedObject(const std::shared_ptr<Object>& object) const
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        const auto* body = btRigidBody::upcast(object->getCollisionObject());
        if (!body)
            return {};
        int contacts = 0, ignored = 0, worldContacts = 0, ragdollContacts = 0;
        auto* dispatcher = mCollisionWorld->getDispatcher();
        for (int index = 0; index < dispatcher->getNumManifolds(); ++index)
        {
            const auto* manifold = dispatcher->getManifoldByIndexInternal(index);
            if (manifold->getBody0() == body || manifold->getBody1() == body)
            {
                const auto* other = manifold->getBody0() == body ? manifold->getBody1() : manifold->getBody0();
                for (int point = 0; point < manifold->getNumContacts(); ++point)
                    if (manifold->getContactPoint(point).getDistance() <= 0.f)
                    {
                        ++contacts;
                        if (mRagdollTransforms.contains(btRigidBody::upcast(other)))
                            ++ragdollContacts;
                        else
                            ++worldContacts;
                    }
            }
        }
        for (const auto& pair : mReleasedGripPairs)
            ignored += pair.item == body;
        return { Misc::Convert::toOsg(body->getWorldTransform().getOrigin()),
            Misc::Convert::toOsg(body->getLinearVelocity()), Misc::Convert::toOsg(body->getAngularVelocity()),
            osg::Vec3f(body->isActive() ? 1.f : 0.f, contacts, ignored),
            osg::Vec3f(worldContacts, ragdollContacts, 0.f) };
    }

    void PhysicsTaskScheduler::changeRagdollVelocities(const std::vector<std::pair<btRigidBody*, btVector3>>& changes)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        for (const auto& [body, change] : changes)
        {
            body->activate(true);
            body->setLinearVelocity(body->getLinearVelocity() + change);
        }
    }

    void PhysicsTaskScheduler::configureRagdoll(btActionInterface* action, const RagdollControlConfig& config)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        static_cast<RagdollStepAction*>(action)->configure(config, mCollisionWorld);
    }

    bool PhysicsTaskScheduler::applyRagdollImpact(btActionInterface* action, const RagdollImpact& event)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        return static_cast<RagdollStepAction*>(action)->impact(event);
    }

    bool PhysicsTaskScheduler::applyRagdollKick(btActionInterface* action, std::size_t part,
        const btVector3& point, const btVector3& velocity, std::uint64_t id)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        return static_cast<RagdollStepAction*>(action)->kick(part, point, velocity, id);
    }

    std::map<std::string, double> PhysicsTaskScheduler::readRagdollControl(btActionInterface* action) const
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        return static_cast<RagdollStepAction*>(action)->snapshot();
    }

    namespace
    {
        std::pair<btVector3, btVector3> superKickCenter(const std::vector<btRigidBody*>& bodies)
        {
            btVector3 position(0, 0, 0), velocity(0, 0, 0);
            btScalar mass = 0;
            for (const auto* body : bodies)
            {
                const btScalar weight = 1 / body->getInvMass();
                position += body->getCenterOfMassPosition() * weight;
                velocity += body->getLinearVelocity() * weight;
                mass += weight;
            }
            return { position / mass, velocity / mass };
        }

        bool superKickSupported(btCollisionWorld& world, const std::vector<btRigidBody*>& bodies)
        {
            const auto owns = [&](const btCollisionObject* object) {
                return std::find(bodies.begin(), bodies.end(), object) != bodies.end();
            };
            auto* dispatcher = world.getDispatcher();
            for (int index = 0; index < dispatcher->getNumManifolds(); ++index)
            {
                const auto* manifold = dispatcher->getManifoldByIndexInternal(index);
                const bool first = owns(manifold->getBody0());
                if (!first && !owns(manifold->getBody1())) continue;
                const auto* other = first ? manifold->getBody1() : manifold->getBody0();
                const auto* proxy = other->getBroadphaseHandle();
                if (!proxy || !(proxy->m_collisionFilterGroup & SuperKick::WorldMask)) continue;
                for (int point = 0; point < manifold->getNumContacts(); ++point)
                {
                    const auto& contact = manifold->getContactPoint(point);
                    const auto normal = first ? contact.m_normalWorldOnB : -contact.m_normalWorldOnB;
                    if (contact.getDistance() <= 0 && normal.z() > btScalar(.5)) return true;
                }
            }
            return false;
        }
    }

    bool PhysicsTaskScheduler::hasActiveSuperKicks() const
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        return std::any_of(mSuperKicks.begin(), mSuperKicks.end(),
            [](const auto& item) { return !item.second.stopped; });
    }

    void PhysicsTaskScheduler::setSuperKickActors(const std::vector<SuperKickActor>& actors)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        mSuperKickActors.clear();
        mSuperKickActorVelocities.clear();
        mSuperKickActorRefs.clear();
        for (const auto& [actor, velocity, refNum] : actors)
            if (mCollisionObjects.contains(actor))
            {
                mSuperKickActors.insert(actor);
                mSuperKickActorVelocities.emplace(actor, Misc::Convert::toBullet(velocity));
                mSuperKickActorRefs.emplace(actor, refNum);
            }
    }

    void PhysicsTaskScheduler::startSuperKick(std::uint64_t token, std::uint64_t kickId,
        const std::vector<btRigidBody*>& bodies, const osg::Vec3f& velocity, const btCollisionObject* attacker)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        forgetSuperKickObject(bodies.front());
        auto& projectile = mSuperKicks[token];
        projectile.kickId = kickId;
        projectile.bodies = bodies;
        projectile.attacker = attacker;
        projectile.initialCenter = superKickCenter(bodies).first;
        projectile.direction = btVector3(velocity.x(), velocity.y(), 0).normalized();
        projectile.velocity = Misc::Convert::toBullet(velocity);
        if (attacker) projectile.hitActors.insert(attacker);
        // Launch the whole connected body coherently. A 900 u/s chest-only
        // impulse would leave neighbouring limbs several times slower, forcing
        // the constraints to absorb the launch in their first solver iteration.
        const btVector3 spin = btVector3(0, 0, 1).cross(projectile.direction) * btScalar(2);
        for (auto* body : bodies)
        {
            const auto linear = projectile.velocity
                + spin.cross(body->getCenterOfMassPosition() - projectile.initialCenter);
            body->setLinearVelocity(linear);
            body->setInterpolationLinearVelocity(linear);
            body->setAngularVelocity(spin);
            body->setInterpolationAngularVelocity(spin);
            body->activate(true);
        }
    }

    std::map<std::string, double> PhysicsTaskScheduler::readSuperKick(std::uint64_t token) const
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        const auto found = mSuperKicks.find(token);
        if (found == mSuperKicks.end()) return {};
        const auto& p = found->second;
        std::map<std::string, double> result{
            { "active", p.stopped == 0 }, { "kickId", double(p.kickId) }, { "age", p.age },
            { "airborne", p.airborne }, { "stopReason", p.stopped },
            { "speed", p.velocity.length() }, { "forwardSpeed", p.velocity.dot(p.direction) },
            { "steps", p.steps }, { "sweeps", p.sweeps }, { "collisions", p.contacts },
            { "occluded", p.blocked }, { "rejectedSpeed", p.rejectedSpeed },
            { "lastPart", p.lastPart }, { "lastFraction", p.lastFraction },
            { "lastContactSpeed", p.lastContactSpeed }, { "lastOverlap", p.lastOverlap },
            { "lifetime", SuperKick::Lifetime }, { "momentumShare", SuperKick::MomentumShare }
        };
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::string suffix(1, "XYZ"[axis]);
            result["velocity" + suffix] = p.velocity[axis];
            result["lastPoint" + suffix] = p.lastPoint[axis];
            result["lastNormal" + suffix] = p.lastNormal[axis];
            result["lastTransfer" + suffix] = p.lastTransfer[axis];
        }
        return result;
    }

    std::vector<PhysicsTaskScheduler::SuperKickContact> PhysicsTaskScheduler::takeSuperKickContacts()
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        std::vector<SuperKickContact> result;
        result.swap(mSuperKickContacts);
        return result;
    }

    void PhysicsTaskScheduler::cancelSuperKick(std::uint64_t token)
    {
        if (!token) return;
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        if (const auto found = mSuperKicks.find(token); found != mSuperKicks.end()) found->second.stopped = 4;
        std::erase_if(mSuperKickContacts, [token](const SuperKickContact& contact) { return contact.token == token; });
    }

    void PhysicsTaskScheduler::guardSuperKickSpawn(const std::vector<btRigidBody*>& bodies)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        for (auto* body : bodies)
            for (const auto& [existing, history] : mRagdollTransforms)
            {
                if (std::find(bodies.begin(), bodies.end(), existing) != bodies.end()) continue;
                auto* other = const_cast<btRigidBody*>(existing);
                if (!releasedGripOverlaps(*mCollisionWorld, body, other)) continue;
                body->setIgnoreCollisionCheck(other, true);
                other->setIgnoreCollisionCheck(body, true);
                mReleasedGripPairs.push_back({ body, other });
                invalidateReleasedPairs(*mCollisionWorld, body);
                invalidateReleasedPairs(*mCollisionWorld, other);
            }
    }

    void PhysicsTaskScheduler::forgetSuperKickObject(const btCollisionObject* object)
    {
        mSuperKickActors.erase(object);
        mSuperKickActorVelocities.erase(object);
        mSuperKickActorRefs.erase(object);
        for (auto it = mSuperKicks.begin(); it != mSuperKicks.end();)
            if (std::find(it->second.bodies.begin(), it->second.bodies.end(), object) != it->second.bodies.end())
            {
                const auto token = it->first;
                std::erase_if(mSuperKickContacts, [token](const SuperKickContact& c) { return c.token == token; });
                it = mSuperKicks.erase(it);
            }
            else
            {
                it->second.hitActors.erase(object);
                if (it->second.attacker == object) it->second.attacker = nullptr;
                ++it;
            }
        std::erase_if(mSuperKickContacts, [object](const SuperKickContact& c) { return c.actor == object; });
    }

    void PhysicsTaskScheduler::beforeSuperKickStep()
    {
        for (auto& [token, projectile] : mSuperKicks)
        {
            if (projectile.stopped) continue;
            projectile.before.clear(); projectile.linear.clear(); projectile.angular.clear();
            for (const auto* body : projectile.bodies)
            {
                projectile.before.push_back(body->getWorldTransform());
                projectile.linear.push_back(body->getLinearVelocity());
                projectile.angular.push_back(body->getAngularVelocity());
            }
            projectile.velocity = superKickCenter(projectile.bodies).second;
        }
    }

    void PhysicsTaskScheduler::afterSuperKickStep(btScalar duration)
    {
        for (auto& [token, p] : mSuperKicks)
        {
            if (p.stopped) continue;
            p.age += duration;
            ++p.steps;
            const auto [center, currentVelocity] = superKickCenter(p.bodies);
            if (!SuperKick::finite(center) || !SuperKick::finite(currentVelocity)) { p.stopped = 4; continue; }
            if (p.age > SuperKick::Lifetime) { p.stopped = 1; continue; }
            // A sleeping body resting on another ragdoll is no longer in flight,
            // even when it has no direct floor manifold of its own.
            if (std::all_of(p.bodies.begin(), p.bodies.end(), [](const auto* body) { return !body->isActive(); }))
            { p.stopped = 2; continue; }
            const bool supported = superKickSupported(*mCollisionWorld, p.bodies);
            p.unsupported = supported ? 0 : p.unsupported + duration;
            p.airborne = p.airborne || p.unsupported > btScalar(.025)
                || center.z() - p.initialCenter.z() > 8;
            std::vector<SuperKick::Witness> witnesses;
            for (unsigned part = 0; part < p.bodies.size(); ++part)
            {
                auto result = SuperKick::trace(*mCollisionWorld, *p.bodies[part], p.before[part],
                    p.bodies[part]->getWorldTransform(), mSuperKickActors);
                ++p.sweeps;
                p.blocked += result.blocked;
                for (auto hit : result.hits)
                {
                    const auto refNum = mSuperKickActorRefs.at(hit.actor);
                    if (p.hitActors.contains(hit.actor) || (refNum.isSet() && p.hitRefs.contains(refNum))) continue;
                    hit.part = part;
                    witnesses.push_back(hit);
                }
            }
            std::sort(witnesses.begin(), witnesses.end(), [](const auto& a, const auto& b) {
                if (a.fraction != b.fraction) return a.fraction < b.fraction;
                return a.part < b.part;
            });
            for (const auto& hit : witnesses)
            {
                const auto refNum = mSuperKickActorRefs.at(hit.actor);
                if (p.hitActors.contains(hit.actor) || (refNum.isSet() && p.hitRefs.contains(refNum))) continue;
                const btVector3 targetVelocity = mSuperKickActorVelocities.at(hit.actor);
                const auto contactVelocity = p.linear[hit.part]
                    + p.angular[hit.part].cross(hit.point - p.before[hit.part].getOrigin()) - targetVelocity;
                const btScalar approach = -contactVelocity.dot(hit.normal);
                if (approach < SuperKick::MinimumContactSpeed) { ++p.rejectedSpeed; continue; }
                // Every standard NPC ragdoll uses the same anatomical masses.
                // Transfer only a fraction of relative COM momentum and remove
                // it from the source. Ten victims cannot each inherit a fresh
                // 900 u/s launch. A victim never becomes another projectile.
                const auto sourceVelocity = superKickCenter(p.bodies).second;
                const auto relative = sourceVelocity - targetVelocity;
                const auto delta = SuperKick::transfer(relative);
                for (auto* body : p.bodies)
                    body->setLinearVelocity(body->getLinearVelocity() - delta);
                p.hitActors.insert(hit.actor);
                // A survivor keeps its identity even if a cell unload replaces
                // the capsule. Raw addresses are retired on deletion so an
                // unrelated NPC reusing an allocation remains eligible.
                if (refNum.isSet()) p.hitRefs.insert(refNum);
                ++p.contacts;
                p.lastPart = hit.part; p.lastPoint = hit.point; p.lastNormal = hit.normal;
                p.lastFraction = hit.fraction; p.lastContactSpeed = approach; p.lastOverlap = hit.overlap;
                p.lastTransfer = delta;
                mSuperKickContacts.push_back({ token, hit.actor, Misc::Convert::toOsg(delta) });
            }
            // Initial feet contact is expected during launch. Disarm only once
            // actual flight has occurred, or a failed launch stayed on support.
            if (supported && currentVelocity.z() > -30 && currentVelocity.z() < 30
                && ((p.airborne && p.age > btScalar(.08)) || p.age > btScalar(.25))) p.stopped = 3;
            p.velocity = superKickCenter(p.bodies).second;
        }
    }

    void PhysicsTaskScheduler::addRagdollBodies(const std::vector<btRigidBody*>& bodies,
        const std::vector<btTypedConstraint*>& constraints, btActionInterface* action)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        mCollisionObjects.reserve(mCollisionObjects.size() + bodies.size());
        for (auto* body : bodies)
        {
            world->addRigidBody(body, CollisionType_Dynamic,
                CollisionType_World | CollisionType_Door | CollisionType_HeightMap | CollisionType_Dynamic);
            mCollisionObjects.insert(body);
            const auto transform = body->getWorldTransform();
            mRagdollTransforms.emplace(body, RagdollTransformHistory{ transform, transform });
        }
        for (auto* constraint : constraints)
            world->addConstraint(constraint, false); // Per-pair exclusions are explicitly owned by the ragdoll.
        if (action)
        {
            world->addAction(action);
            mRagdollActions.push_back(action);
        }
        // Eleven linked bodies benefit from the donor's firmer solver setting.
        world->getSolverInfo().m_numIterations = std::max(world->getSolverInfo().m_numIterations, 20);
    }

    void PhysicsTaskScheduler::removeRagdollBodies(const std::vector<btRigidBody*>& bodies,
        const std::vector<btTypedConstraint*>& constraints, btActionInterface* action)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        if (action)
        {
            world->removeAction(action);
            mRagdollActions.erase(std::remove(mRagdollActions.begin(), mRagdollActions.end(), action), mRagdollActions.end());
            if (mRagdollActions.empty())
            {
                world->getSolverInfo().m_numIterations = 20;
                world->getSolverInfo().m_solverMode = mRagdollBaselineSolverMode;
            }
        }
        for (auto* constraint : constraints)
            world->removeConstraint(constraint);
        for (auto* body : bodies)
        {
            forgetSuperKickObject(body);
            clearReleasedGripPairs(body);
            mRagdollTransforms.erase(body);
            mCollisionObjects.erase(body);
            world->removeRigidBody(body);
        }
    }

    PhysicsTaskScheduler::RagdollDetachSnapshot PhysicsTaskScheduler::detachRagdollConstraint(
        btTypedConstraint* constraint, btActionInterface* oldAction, btActionInterface* newAction,
        btRigidBody* detachedBody, const std::vector<btRigidBody*>& bodies,
        const btVector3& cutVelocity, const btVector3& headAngularImpulse)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        RagdollDetachSnapshot result;
        if (oldAction && newAction)
            static_cast<RagdollStepAction*>(newAction)->inherit(
                *static_cast<RagdollStepAction*>(oldAction), constraint, mCollisionWorld);
        result.before = detachedBody->getWorldTransform();
        result.linearBefore = detachedBody->getLinearVelocity();
        result.angularBefore = detachedBody->getAngularVelocity();
        result.requestedVelocity = cutVelocity;
        result.requestedHeadImpulse = headAngularImpulse;
        result.launched = cutVelocity.length2() > 0;
        struct PartSnapshot
        {
            btRigidBody* body;
            btTransform transform;
            btVector3 linear, angular;
        };
        std::vector<PartSnapshot> retained;
        const auto momentum = [&](bool before) {
            RagdollDetachSnapshot::Momentum state;
            for (btRigidBody* body : bodies)
            {
                if (body == detachedBody || body->getInvMass() <= 0)
                    continue;
                const btScalar mass = 1 / body->getInvMass();
                state.mass += mass;
                state.center += body->getCenterOfMassPosition() * mass;
                state.linear += body->getLinearVelocity() * mass;
                if (before)
                    retained.push_back({ body, body->getWorldTransform(), body->getLinearVelocity(),
                        body->getAngularVelocity() });
            }
            if (state.mass > 0)
                state.center /= state.mass;
            for (btRigidBody* body : bodies)
            {
                if (body == detachedBody || body->getInvMass() <= 0)
                    continue;
                // Spin plus orbital angular momentum about the retained body's COM.
                state.angular += body->getInvInertiaTensorWorld().inverse() * body->getAngularVelocity()
                    + (body->getCenterOfMassPosition() - state.center)
                        .cross(body->getLinearVelocity() / body->getInvMass());
            }
            return state;
        };
        result.retainedBefore = momentum(true);
        // The old friction action also links the two bodies. Replace it in the
        // same critical section as the constraint; neither can survive the cut.
        if (oldAction)
        {
            world->removeAction(oldAction);
            mRagdollActions.erase(std::remove(mRagdollActions.begin(), mRagdollActions.end(), oldAction), mRagdollActions.end());
        }
        world->removeConstraint(constraint);
        if (newAction)
        {
            world->addAction(newAction);
            mRagdollActions.push_back(newAction);
        }
        // Dark Messiah's Cut assigns the same directed linear velocity to the
        // retained body and fragment. Its separate fragment torque is additive.
        // Keep this in the constraint transaction: no intermediate pose/step.
        result.headAngularExpected = result.angularBefore;
        if (result.launched)
        {
            for (btRigidBody* body : bodies)
            {
                body->setLinearVelocity(cutVelocity);
                body->activate(true);
            }
            // User-requested separation: central impulse changes head velocity only,
            // without another torque, transform write, or intermediate physics step.
            detachedBody->applyCentralImpulse(btVector3(0, 0, Misc::BrutalHeadSeparationSpeed)
                / detachedBody->getInvMass());
            result.headAngularExpected += detachedBody->getInvInertiaTensorWorld() * headAngularImpulse;
            result.unclampedHeadAngularSpeed = result.headAngularExpected.length();
            // The provided Source vphysics environment defaults to 3600 deg/s.
            // Its large cut torque reaches that cap; never feed an unbounded
            // converted torque velocity into Bullet's next solver step.
            constexpr btScalar angularLimit = 62.831853071795864;
            if (result.headAngularExpected.length2() > angularLimit * angularLimit)
                result.headAngularExpected *= angularLimit / result.headAngularExpected.length();
            detachedBody->applyTorqueImpulse(headAngularImpulse);
            detachedBody->setAngularVelocity(result.headAngularExpected);
        }
        detachedBody->activate(true);
        result.after = detachedBody->getWorldTransform();
        result.linearAfter = detachedBody->getLinearVelocity();
        result.angularAfter = detachedBody->getAngularVelocity();
        result.appliedHeadImpulse = detachedBody->getInvInertiaTensorWorld().inverse()
            * (result.angularAfter - result.angularBefore);
        for (btRigidBody* body : bodies)
        {
            result.partLinearAfter.push_back(body->getLinearVelocity());
            if (result.launched)
                result.targetLinearVelocityError = btMax(result.targetLinearVelocityError,
                    (body->getLinearVelocity() - cutVelocity
                        - (body == detachedBody ? btVector3(0, 0, Misc::BrutalHeadSeparationSpeed) : btVector3(0, 0, 0))).length());
        }
        result.retainedAfter = momentum(false);
        for (const PartSnapshot& part : retained)
        {
            const btTransform& after = part.body->getWorldTransform();
            for (int row = 0; row < 3; ++row)
            {
                result.retainedTransformError = btMax(result.retainedTransformError,
                    btFabs(part.transform.getOrigin()[row] - after.getOrigin()[row]));
                for (int column = 0; column < 3; ++column)
                    result.retainedTransformError = btMax(result.retainedTransformError,
                        btFabs(part.transform.getBasis()[row][column] - after.getBasis()[row][column]));
            }
            result.retainedLinearVelocityError = btMax(result.retainedLinearVelocityError,
                ((result.launched ? cutVelocity : part.linear) - part.body->getLinearVelocity()).length());
            result.retainedAngularVelocityError = btMax(result.retainedAngularVelocityError,
                (part.angular - part.body->getAngularVelocity()).length());
        }
        return result;
    }

    PhysicsTaskScheduler::RagdollTorsoDetachSnapshot PhysicsTaskScheduler::detachRagdollTorso(btTypedConstraint* constraint,
        btActionInterface* oldAction, btActionInterface* newAction,
        const std::vector<btRigidBody*>& bodies, const btVector3& cutVelocity,
        const btVector3& upperSeparation, const std::optional<std::array<btTransform, 2>>& desiredAnchors)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        RagdollTorsoDetachSnapshot result;
        const auto finiteRigid = [](const btTransform& transform) {
            const auto& p = transform.getOrigin();
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) return false;
            const auto& basis = transform.getBasis();
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                    if (!std::isfinite(basis[row][column])) return false;
                if (btFabs(basis[row].length2() - 1) > btScalar(.001)) return false;
                for (int other = 0; other < row; ++other)
                    if (btFabs(basis[row].dot(basis[other])) > btScalar(.001)) return false;
            }
            return btFabs(basis.determinant() - 1) <= btScalar(.001);
        };
        const auto transformError = [](const btTransform& actual, const btTransform& expected) {
            btScalar error = 0;
            for (int row = 0; row < 3; ++row)
            {
                error = btMax(error, btFabs(actual.getOrigin()[row] - expected.getOrigin()[row]));
                for (int column = 0; column < 3; ++column)
                    error = btMax(error, btFabs(actual.getBasis()[row][column] - expected.getBasis()[row][column]));
            }
            return error;
        };
        // Finish validation and allocation before changing the controller,
        // constraints, contacts, or any body. A rejected pose still restores an
        // ordinary boolean cut using the caller's zero restoration velocity.
        std::vector<btTransform> restoredTransforms;
        if (desiredAnchors && bodies.size() == 11 && finiteRigid((*desiredAnchors)[0])
            && finiteRigid((*desiredAnchors)[1]) && finiteRigid(bodies[0]->getWorldTransform())
            && finiteRigid(bodies[1]->getWorldTransform()))
        {
            const std::array<btTransform, 2> delta{
                (*desiredAnchors)[0] * bodies[0]->getWorldTransform().inverse(),
                (*desiredAnchors)[1] * bodies[1]->getWorldTransform().inverse() };
            restoredTransforms.reserve(bodies.size());
            for (std::size_t i = 0; i < bodies.size(); ++i)
            {
                const auto target = delta[i >= 1 && i <= 6 ? 1 : 0] * bodies[i]->getWorldTransform();
                if (!finiteRigid(target) || !mRagdollTransforms.contains(bodies[i]))
                { restoredTransforms.clear(); break; }
                restoredTransforms.push_back(target);
            }
        }
        result.poseRestored = restoredTransforms.size() == bodies.size() && !restoredTransforms.empty();
        static_cast<RagdollStepAction*>(newAction)->inherit(
            *static_cast<RagdollStepAction*>(oldAction), constraint, world);
        world->removeAction(oldAction);
        mRagdollActions.erase(std::remove(mRagdollActions.begin(), mRagdollActions.end(), oldAction), mRagdollActions.end());
        world->removeConstraint(constraint);
        world->addAction(newAction);
        mRagdollActions.push_back(newAction);
        const bool launch = !result.poseRestored && cutVelocity.length2() > 0;
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            auto* body = bodies[i];
            const auto before = body->getWorldTransform();
            const auto desiredTransform = result.poseRestored ? restoredTransforms[i] : before;
            const auto desiredAngular = result.poseRestored ? btVector3(0, 0, 0) : body->getAngularVelocity();
            const auto desiredLinear = result.poseRestored ? btVector3(0, 0, 0)
                : launch ? cutVelocity + (i >= 1 && i <= 6 ? upperSeparation : btVector3(0, 0, 0))
                         : body->getLinearVelocity();
            if (result.poseRestored)
            {
                // Each fragment undergoes one rigid change of frame. Its joints
                // and individual limb angles remain internally unchanged.
                body->setCenterOfMassTransform(desiredTransform);
                body->setLinearVelocity(desiredLinear);
                body->setAngularVelocity(desiredAngular);
                body->setInterpolationWorldTransform(desiredTransform);
                body->setInterpolationLinearVelocity(btVector3(0, 0, 0));
                body->setInterpolationAngularVelocity(btVector3(0, 0, 0));
                body->clearForces();
                mRagdollTransforms.at(body) = RagdollTransformHistory{ desiredTransform, desiredTransform };
                result.poseRestoreDisplacement = btMax(result.poseRestoreDisplacement,
                    (desiredTransform.getOrigin() - before.getOrigin()).length());
                const auto a = before.getRotation(), b = desiredTransform.getRotation();
                const btScalar cosine = btClamped(btFabs(a.dot(b)) / btSqrt(a.length2() * b.length2()), btScalar(0), btScalar(1));
                result.poseRestoreRotation = btMax(result.poseRestoreRotation, 2 * btAcos(cosine));
            }
            else if (launch) body->setLinearVelocity(desiredLinear);
            body->activate(true);
            const auto& after = body->getWorldTransform();
            result.errors[0] = btMax(result.errors[0], transformError(after, desiredTransform));
            result.errors[1] = btMax(result.errors[1], (body->getLinearVelocity() - desiredLinear).length());
            result.errors[2] = btMax(result.errors[2], (body->getAngularVelocity() - desiredAngular).length());
        }
        if (result.poseRestored)
        {
            // Drop every stale contact from the coherent pre-cut pose before
            // the next solver step. Cross-fragment filters remain owned by the
            // new controller and rearm only after its normal clearance check.
            auto* cache = world->getBroadphase()->getOverlappingPairCache();
            for (auto* body : bodies) world->updateSingleAabb(body);
            for (auto* body : bodies)
                if (auto* proxy = body->getBroadphaseHandle())
                    cache->cleanProxyFromPairs(proxy, world->getDispatcher());
            for (unsigned side = 0; side < 2; ++side)
                result.poseRestoreError = btMax(result.poseRestoreError,
                    transformError(bodies[side]->getWorldTransform(), (*desiredAnchors)[side]));
            result.poseAnchorDistance = (bodies[1]->getWorldTransform().getOrigin()
                - bodies[0]->getWorldTransform().getOrigin()).length();
        }
        return result;
    }

    PhysicsTaskScheduler::RagdollLimbDetachSnapshot PhysicsTaskScheduler::detachRagdollLimbs(
        const std::vector<btTypedConstraint*>& constraints, btActionInterface* oldAction,
        btActionInterface* newAction, const std::vector<btRigidBody*>& bodies, unsigned mask,
        const btVector3& cutVelocity, const std::optional<std::array<btTransform, 11>>& desiredPose)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        RagdollLimbDetachSnapshot result;
        result.poseRestored = desiredPose.has_value() && bodies.size() == LimbSevering::PartCount;
        if (result.poseRestored)
            for (std::size_t i = 0; i < bodies.size(); ++i)
                if (!LimbSevering::finiteRigid((*desiredPose)[i]) || !mRagdollTransforms.contains(bodies[i]))
                { result.poseRestored = false; break; }
        // Controller links and collision fragment identities are prepared together.
        // Removed constraints must never remain in the soft/tone/friction drives.
        static_cast<RagdollStepAction*>(newAction)->inherit(
            *static_cast<RagdollStepAction*>(oldAction), nullptr, world);
        world->removeAction(oldAction);
        mRagdollActions.erase(std::remove(mRagdollActions.begin(), mRagdollActions.end(), oldAction), mRagdollActions.end());
        for (auto* constraint : constraints) world->removeConstraint(constraint);
        world->addAction(newAction);
        mRagdollActions.push_back(newAction);
        const btVector3 separation = result.poseRestored ? btVector3(0, 0, 0)
            : LimbSevering::separationVelocity(cutVelocity);
        const auto transformError = [](const btTransform& actual, const btTransform& expected) {
            btScalar error = 0;
            for (int row = 0; row < 3; ++row)
            {
                error = btMax(error, btFabs(actual.getOrigin()[row] - expected.getOrigin()[row]));
                for (int column = 0; column < 3; ++column)
                    error = btMax(error, btFabs(actual.getBasis()[row][column] - expected.getBasis()[row][column]));
            }
            return error;
        };
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            auto* body = bodies[i];
            const auto before = body->getWorldTransform();
            const auto desiredTransform = result.poseRestored ? (*desiredPose)[i] : before;
            const auto desiredAngular = result.poseRestored ? btVector3(0, 0, 0) : body->getAngularVelocity();
            const auto desiredLinear = result.poseRestored ? btVector3(0, 0, 0)
                : body->getLinearVelocity() + (LimbSevering::fragment(i, mask) ? separation : btVector3(0, 0, 0));
            if (result.poseRestored)
            {
                body->setCenterOfMassTransform(desiredTransform);
                body->setLinearVelocity(desiredLinear);
                body->setAngularVelocity(desiredAngular);
                body->setInterpolationWorldTransform(desiredTransform);
                body->setInterpolationLinearVelocity(btVector3(0, 0, 0));
                body->setInterpolationAngularVelocity(btVector3(0, 0, 0));
                body->clearForces();
                mRagdollTransforms.at(body) = RagdollTransformHistory{ desiredTransform, desiredTransform };
            }
            else if (separation.length2() > 0 && LimbSevering::fragment(i, mask))
                body->setLinearVelocity(desiredLinear);
            body->activate(true);
            result.errors[0] = btMax(result.errors[0], transformError(body->getWorldTransform(), desiredTransform));
            result.errors[1] = btMax(result.errors[1], (body->getLinearVelocity() - desiredLinear).length());
            result.errors[2] = btMax(result.errors[2], (body->getAngularVelocity() - desiredAngular).length());
            if (result.poseRestored)
                result.poseRestoreError = btMax(result.poseRestoreError,
                    transformError(body->getWorldTransform(), (*desiredPose)[i]));
        }
        if (result.poseRestored)
        {
            auto* cache = world->getBroadphase()->getOverlappingPairCache();
            for (auto* body : bodies) world->updateSingleAabb(body);
            for (auto* body : bodies)
                if (auto* proxy = body->getBroadphaseHandle())
                    cache->cleanProxyFromPairs(proxy, world->getDispatcher());
        }
        return result;
    }

    std::vector<btTransform> PhysicsTaskScheduler::readRigidBodyTransforms(
        const std::vector<btRigidBody*>& bodies, std::vector<btVector3>* linear, std::vector<btVector3>* angular,
        std::vector<btTransform>* rendered, std::optional<float> interpolation) const
    {
        MaybeLock lock(mCollisionWorldMutex, mLockingPolicy);
        if (linear) linear->clear();
        if (angular) angular->clear();
        if (rendered) { rendered->clear(); rendered->reserve(bodies.size()); }
        const btScalar alpha = std::clamp(interpolation.value_or(mRagdollInterpolation), 0.f, 1.f);
        std::vector<btTransform> transforms;
        transforms.reserve(bodies.size());
        for (const auto* body : bodies)
        {
            transforms.push_back(body->getWorldTransform());
            if (rendered)
            {
                const auto history = mRagdollTransforms.find(body);
                if (history == mRagdollTransforms.end())
                    rendered->push_back(transforms.back());
                else
                {
                    const auto& [previous, current] = history->second;
                    auto rotation = current.getRotation();
                    const auto from = previous.getRotation();
                    if (from.dot(rotation) < 0)
                        rotation = btQuaternion(-rotation.x(), -rotation.y(), -rotation.z(), -rotation.w());
                    rendered->emplace_back(from.slerp(rotation, alpha), previous.getOrigin().lerp(current.getOrigin(), alpha));
                }
            }
            if (linear) linear->push_back(body->getLinearVelocity());
            if (angular) angular->push_back(body->getAngularVelocity());
        }
        return transforms;
    }

    void PhysicsTaskScheduler::disableDynamicObjects(bool props, bool equipment)
    {
        if (mDynamicObjects.empty() || (!props && !equipment)) return;
        waitForWorkers();
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        auto* world = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld);
        for (auto it = mDynamicObjects.begin(); it != mDynamicObjects.end(); )
        {
            const auto object = it->lock();
            if (!object) { it = mDynamicObjects.erase(it); continue; }
            const bool released = bool(object->getPtr().getRefData().getReleasedEquipment());
            if (!(released ? equipment : props)) { ++it; continue; }
            auto* body = btRigidBody::upcast(object->getCollisionObject());
            if (body)
            {
                clearReleasedGripPairs(body);
                forgetSuperKickObject(body);
                mCollisionObjects.erase(body);
                mDynamicTransforms.erase(body);
                world->removeRigidBody(body);
                object->makeStatic();
                auto* collision = object->getCollisionObject();
                world->addCollisionObject(collision, CollisionType_World,
                    CollisionType_Actor | CollisionType_HeightMap | CollisionType_Projectile | CollisionType_Dynamic);
                mCollisionObjects.insert(collision);
            }
            it = mDynamicObjects.erase(it);
        }
    }

    void PhysicsTaskScheduler::resetDynamicObjectHistory(const Object& object)
    {
        if (const auto* body = btRigidBody::upcast(object.getCollisionObject()))
            if (const auto history = mDynamicTransforms.find(body); history != mDynamicTransforms.end())
                history->second.reset(body->getWorldTransform());
    }

    std::vector<PhysicsTaskScheduler::DynamicObjectTransform> PhysicsTaskScheduler::readDynamicObjectTransforms()
    {
        if (mDynamicObjects.empty())
            return {};
        // Match the ragdoll read boundary: finish this frame's pending outer
        // steps before publishing history with its residual clock. Waiting
        // under the collision-world mutex would block those same workers.
        waitForWorkers();
        // Snapshot every item and the shared render clock atomically. World
        // cell moves may remove objects after this lock is released.
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        std::vector<DynamicObjectTransform> result;
        result.reserve(mDynamicObjects.size());
        for (auto it = mDynamicObjects.begin(); it != mDynamicObjects.end(); )
        {
            if (const auto object = it->lock())
            {
                const auto* body = btRigidBody::upcast(object->getCollisionObject());
                const auto history = mDynamicTransforms.find(body);
                if (history != mDynamicTransforms.end())
                {
                    DynamicObjectTransform snapshot{ object->getPtr(), {}, {}, false };
                    if (object->takeDynamicTransforms(history->second.rendered(mRagdollInterpolation),
                            snapshot.authoritative, snapshot.rendered, snapshot.authoritativeChanged))
                        result.push_back(std::move(snapshot));
                }
                ++it;
            }
            else
                it = mDynamicObjects.erase(it);
        }
        return result;
    }

    void PhysicsTaskScheduler::removeCollisionObject(btCollisionObject* collisionObject)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        forgetSuperKickObject(collisionObject);
        clearReleasedGripPairs(collisionObject);
        mCollisionObjects.erase(collisionObject);
        if (auto* body = btRigidBody::upcast(collisionObject))
        {
            mDynamicTransforms.erase(body);
            static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld)->removeRigidBody(body);
        }
        else
            mCollisionWorld->removeCollisionObject(collisionObject);
    }

    void PhysicsTaskScheduler::updateSingleAabb(const std::shared_ptr<PtrHolder>& ptr, bool immediate)
    {
        if (immediate || mNumThreads == 0)
        {
            updatePtrAabb(ptr);
        }
        else
        {
            MaybeExclusiveLock lock(mUpdateAabbMutex, mLockingPolicy);
            mUpdateAabb.insert(ptr);
        }
    }

    bool PhysicsTaskScheduler::getLineOfSight(
        const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2)
    {
        MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);

        auto req = LOSRequest(actor1, actor2);
        auto result = std::find(mLOSCache.begin(), mLOSCache.end(), req);
        if (result == mLOSCache.end())
        {
            req.mResult = hasLineOfSight(actor1.get(), actor2.get());
            mLOSCache.push_back(req);
            return req.mResult;
        }
        result->mAge = 0;
        return result->mResult;
    }

    void PhysicsTaskScheduler::refreshLOSCache()
    {
        MaybeSharedLock lock(mLOSCacheMutex, mLockingPolicy);
        int job = 0;
        int numLOS = static_cast<int>(mLOSCache.size());
        while ((job = mNextLOS.fetch_add(1, std::memory_order_relaxed)) < numLOS)
        {
            auto& req = mLOSCache[job];
            auto actorPtr1 = req.mActors[0].lock();
            auto actorPtr2 = req.mActors[1].lock();

            if (req.mAge++ > mLOSCacheExpiry || !actorPtr1 || !actorPtr2)
                req.mStale = true;
            else
                req.mResult = hasLineOfSight(actorPtr1.get(), actorPtr2.get());
        }
    }

    void PhysicsTaskScheduler::updateAabbs()
    {
        MaybeExclusiveLock lock(mUpdateAabbMutex, mLockingPolicy);
        std::for_each(mUpdateAabb.begin(), mUpdateAabb.end(), [this](const std::weak_ptr<PtrHolder>& ptr) {
            auto p = ptr.lock();
            if (p != nullptr)
                updatePtrAabb(p);
        });
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr)
    {
        MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
        if (const auto actor = std::dynamic_pointer_cast<Actor>(ptr))
        {
            actor->updateCollisionObjectPosition();
            mCollisionWorld->updateSingleAabb(actor->getCollisionObject());
        }
        else if (const auto object = std::dynamic_pointer_cast<Object>(ptr))
        {
            if (object->commitPositionChange())
                resetDynamicObjectHistory(*object);
            mCollisionWorld->updateSingleAabb(object->getCollisionObject());
        }
        else if (const auto projectile = std::dynamic_pointer_cast<Projectile>(ptr))
        {
            projectile->updateCollisionObjectPosition();
            mCollisionWorld->updateSingleAabb(projectile->getCollisionObject());
        }
    }

    void PhysicsTaskScheduler::worker()
    {
        mWorkersSync->runWorker([this] {
            std::shared_lock lock(mSimulationMutex);
            doSimulation();
        });
    }

    void PhysicsTaskScheduler::updateActorsPositions()
    {
        const Visitors::UpdatePosition impl{ mCollisionWorld };
        const Visitors::WithLockedPtr<Visitors::UpdatePosition, MaybeExclusiveLock> vis{ impl, mCollisionWorldMutex,
            mLockingPolicy };
        for (Simulation& sim : *mSimulations)
            std::visit(vis, sim);
    }

    bool PhysicsTaskScheduler::hasLineOfSight(const Actor* actor1, const Actor* actor2)
    {
        btVector3 pos1 = Misc::Convert::toBullet(
            actor1->getCollisionObjectPosition() + osg::Vec3f(0, 0, actor1->getHalfExtents().z() * 0.9f)); // eye level
        btVector3 pos2 = Misc::Convert::toBullet(
            actor2->getCollisionObjectPosition() + osg::Vec3f(0, 0, actor2->getHalfExtents().z() * 0.9f));

        btCollisionWorld::ClosestRayResultCallback resultCallback(pos1, pos2);
        resultCallback.m_collisionFilterGroup = CollisionType_AnyPhysical;
        resultCallback.m_collisionFilterMask = CollisionType_World | CollisionType_HeightMap | CollisionType_Door;

        MaybeLock lockColWorld(mCollisionWorldMutex, mLockingPolicy);
        mCollisionWorld->rayTest(pos1, pos2, resultCallback);

        return !resultCallback.hasHit();
    }

    void PhysicsTaskScheduler::doSimulation()
    {
        while (mRemainingSteps)
        {
            mPreStepBarrier->wait([this] { afterPreStep(); });
            int job = 0;
            const Visitors::Move impl{ mPhysicsDt, mCollisionWorld, *mWorldFrameData };
            const Visitors::WithLockedPtr<Visitors::Move, MaybeLock> vis{ impl, mCollisionWorldMutex, mLockingPolicy };
            while ((job = mNextJob.fetch_add(1, std::memory_order_relaxed)) < mNumJobs)
                std::visit(vis, (*mSimulations)[job]);

            mPostStepBarrier->wait([this] { afterPostStep(); });
        }

        refreshLOSCache();
        mPostSimBarrier->wait([this] { afterPostSim(); });
    }

    void PhysicsTaskScheduler::updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        if (!stats.collectStats("engine"))
            return;
        if (mFrameNumber == frameNumber - 1)
        {
            stats.setAttribute(mFrameNumber, "physicsworker_time_begin", mTimer->delta_s(mFrameStart, mTimeBegin));
            stats.setAttribute(mFrameNumber, "physicsworker_time_taken", mTimer->delta_s(mTimeBegin, mTimeEnd));
            stats.setAttribute(mFrameNumber, "physicsworker_time_end", mTimer->delta_s(mFrameStart, mTimeEnd));
        }
        mFrameStart = frameStart;
        mTimeBegin = mTimer->tick();
        mFrameNumber = frameNumber;
    }

    void PhysicsTaskScheduler::debugDraw()
    {
        MaybeSharedLock lock(mCollisionWorldMutex, mLockingPolicy);
        mDebugDrawer->step();
    }

    void* PhysicsTaskScheduler::getUserPointer(const btCollisionObject* object) const
    {
        auto it = mCollisionObjects.find(object);
        if (it == mCollisionObjects.end())
            return nullptr;
        return (*it)->getUserPointer();
    }

    void PhysicsTaskScheduler::releaseSharedStates()
    {
        waitForWorkers();
        std::scoped_lock lock(mSimulationMutex, mUpdateAabbMutex);
        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::afterPreStep()
    {
        updateAabbs();
        if (!mRemainingSteps)
            return;
        const Visitors::PreStep impl{ mCollisionWorld };
        const Visitors::WithLockedPtr<Visitors::PreStep, MaybeExclusiveLock> vis{ impl, mCollisionWorldMutex,
            mLockingPolicy };
        for (auto& sim : *mSimulations)
            std::visit(vis, sim);
    }

    void PhysicsTaskScheduler::afterPostStep()
    {
        if (mRemainingSteps)
        {
            --mRemainingSteps;
            updateActorsPositions();
            // All worker sweeps have finished; no Bullet solver runs concurrently
            // with character queries. Query wrappers honor this exclusive lock.
            MaybeExclusiveLock lock(mCollisionWorldMutex, mLockingPolicy);
            if (!mRagdollActions.empty())
            {
                // Bullet stops solving contacts/friction after the global iteration
                // count, even when a joint override extends the joint-only loop.
                // Keep support contacts in the solve until the enhanced joints finish.
                // With the enhancement off the solver keeps 20 contact iterations.
                int iterations = 20;
                for (btActionInterface* action : mRagdollActions)
                    iterations = std::max(iterations, static_cast<RagdollStepAction*>(action)->solverIterations());
                auto& info = static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld)->getSolverInfo();
                info.m_numIterations = iterations;
                // Resolve sliding in the whole tangent plane, including velocity
                // created by joint impulses after the initial tangent was chosen.
                info.m_solverMode = mRagdollBaselineSolverMode;
                if (iterations > 20) info.m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;
            }
            for (auto& [body, history] : mRagdollTransforms)
                history.previous = history.current;
            // Ragdoll limbs turn fast enough on a hard landing (up to ~85 rad/s) that one
            // 1/60 s step solves their joints from too far apart: the elbows and neck opened
            // by up to 9 units. Step the dynamics world in quarters while any ragdoll exists.
            const int substeps = mRagdollTransforms.empty() ? 1 : 4;
            for (int i = 0; i < substeps; ++i)
            {
                updateReleasedGripPairs();
                for (btActionInterface* action : mRagdollActions)
                    static_cast<RagdollStepAction*>(action)->beforeStep(mPhysicsDt / substeps);
                beforeSuperKickStep();
                static_cast<btDiscreteDynamicsWorld*>(mCollisionWorld)->stepSimulation(mPhysicsDt / substeps, 0);
                afterSuperKickStep(mPhysicsDt / substeps);
            }
            for (auto& [body, history] : mRagdollTransforms)
                history.current = body->getWorldTransform();
            for (auto& [body, history] : mDynamicTransforms)
                history.advance(body->getWorldTransform());
            mRagdollInterpolation = std::clamp(mTimeAccum / mPhysicsDt + float(mRemainingSteps), 0.f, 1.f);
        }
        mNextJob.store(0, std::memory_order_release);
    }

    void PhysicsTaskScheduler::afterPostSim()
    {
        {
            MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);
            mLOSCache.erase(
                std::remove_if(mLOSCache.begin(), mLOSCache.end(), [](const LOSRequest& req) { return req.mStale; }),
                mLOSCache.end());
        }
        mTimeEnd = mTimer->tick();
        if (mWorkersSync != nullptr)
            mWorkersSync->workIsDone();
    }

    void PhysicsTaskScheduler::syncWithMainThread()
    {
        // Workers are joined/asleep here. Capture before exposing render updates.
        for (auto it = mDynamicObjects.begin(); it != mDynamicObjects.end(); )
        {
            if (auto object = it->lock())
            {
                object->captureDynamicTransform();
                ++it;
            }
            else
                it = mDynamicObjects.erase(it);
        }
        if (mSimulations == nullptr)
            return;
        const Visitors::Sync vis{ mAdvanceSimulation, mTimeAccum, mPhysicsDt, this };
        for (auto& sim : *mSimulations)
            std::visit(vis, sim);
        mSimulations->clear();
        mSimulations = nullptr;
    }

    // Attempt to acquire unique lock on mSimulationMutex while not all worker
    // threads are holding shared lock but will have to may lead to a deadlock because
    // C++ standard does not guarantee priority for exclusive and shared locks
    // for std::shared_mutex. For example microsoft STL implementation points out
    // for the absence of such priority:
    // https://docs.microsoft.com/en-us/windows/win32/sync/slim-reader-writer--srw--locks
    void PhysicsTaskScheduler::waitForRagdollStep()
    {
        waitForWorkers();
    }

    void PhysicsTaskScheduler::waitForWorkers()
    {
        if (mWorkersSync != nullptr)
            mWorkersSync->waitForWorkers();
    }
}
