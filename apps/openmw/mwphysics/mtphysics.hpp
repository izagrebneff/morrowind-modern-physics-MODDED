#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <thread>
#include <unordered_set>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include "components/misc/budgetmeasurement.hpp"
#include "components/misc/framestats.hpp"
#include "physicssystem.hpp"
#include "ptrholder.hpp"

namespace Misc
{
    class Barrier;
}

namespace MWPhysics
{
    enum class LockingPolicy
    {
        NoLocks,
        ExclusiveLocksOnly,
        AllowSharedLocks,
    };

    class PhysicsTaskScheduler
    {
    public:
        PhysicsTaskScheduler(float physicsDt, btCollisionWorld* collisionWorld);
        ~PhysicsTaskScheduler();

        /// @brief move actors taking into account desired movements and collisions
        /// @param numSteps how much simulation step to run
        /// @param timeAccum accumulated time from previous run to interpolate movements
        /// @param actorsData per actor data needed to compute new positions
        /// @return new position of each actor
        void applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations,
            Misc::FrameClock::time_point statsEpoch, unsigned int frameNumber, Misc::FrameStats& stats);

        void resetSimulation(const ActorMap& actors);

        // Thread safe wrappers
        void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld,
            btCollisionWorld::RayResultCallback& resultCallback) const;
        void convexSweepTest(const btConvexShape* castShape, const btTransform& from, const btTransform& to,
            btCollisionWorld::ConvexResultCallback& resultCallback) const;
        void contactTest(btCollisionObject* colObj, btCollisionWorld::ContactResultCallback& resultCallback);
        std::optional<btVector3> getHitPoint(const btTransform& from, btCollisionObject* target);
        void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);
        void getAabb(const btCollisionObject* obj, btVector3& min, btVector3& max);
        void setCollisionFilterMask(btCollisionObject* collisionObject, int collisionFilterMask);
        void addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask);
        void addRigidBody(btRigidBody* body, int collisionFilterGroup, int collisionFilterMask);
        /// A joint between two rigid bodies already added (its bodies don't collide with each other).
        void addConstraint(btTypedConstraint* constraint);
        void removeConstraint(btTypedConstraint* constraint);
        /// Something run every simulation step (inside it, with the world locked).
        void addAction(btActionInterface* action);
        void removeAction(btActionInterface* action);
        /// Stop a body and put it to sleep.
        void setAtRest(btRigidBody* body);
        /// Override gravity for an individual rigid body while holding the physics world lock.
        void setBodyGravity(btRigidBody* body, const btVector3& gravity);
        /// Lock physics and permanently stop a corpse part from taking further impulses.
        void freezeBody(btRigidBody* body);
        /// Moves a simulated body to transform, still.
        void placeBody(btRigidBody* body, const btTransform& transform);
        /// Wake the body and push it: impulse applied at point (world space).
        void applyImpulse(btRigidBody* body, const btVector3& impulse, const btVector3& point);

        // Carrying a dynamic object: each physics step it is steered towards the hold target.
        /// steerRotation: also turn it to the target orientation. holdMass: if set, its mass while held (a heavier
        /// grip drags what it is jointed to, e.g. the rest of a ragdoll).
        /// grabPoint: if set, it isn't carried but hangs from this world point (pinned there), which then follows
        /// the hold target.
        /// lightened: other objects (the rest of a ragdoll) whose mass is scaled by lightenScale while it's held.
        void holdObject(const std::shared_ptr<PtrHolder>& object, bool steerRotation, btScalar holdMass,
            const std::optional<btVector3>& grabPoint = std::nullopt,
            const std::vector<std::shared_ptr<PtrHolder>>& lightened = {}, btScalar lightenScale = 1);
        void setHoldTarget(const btVector3& position, const btQuaternion& rotation);
        /// Let go of the held object, optionally setting its velocity (for throwing).
        void releaseHeldObject(const std::optional<btVector3>& velocity);
        std::shared_ptr<PtrHolder> getHeldObject() const;

        /// Counts the simulation runs that stepped (as of the last sync with the main thread).
        const std::atomic<unsigned>& getStepGeneration() const { return mStepGeneration; }
        /// How far from the last step to the next the main thread is (as of the last sync), for showing
        /// simulated objects between steps.
        float getSyncInterpolation() const { return mSyncInterpolation; }

        /// Change the velocity of dynamic objects as if struck at a world point (velocityChange is independent
        /// of the object's mass). Ignores the held object.
        struct Strike
        {
            std::shared_ptr<Object> mObject;
            btVector3 mVelocityChange;
            btVector3 mPoint;
            btVector3 mSource; // where the blow came from
        };
        void strikeObjects(const std::vector<Strike>& strikes);

        /// Water level for buoyancy, or none.
        void setWaterHeight(std::optional<float> height);

        /// Watch a thrown object for hitting an actor (other than the thrower) while it flies.
        void addFlyingObject(const std::shared_ptr<Object>& object, const btCollisionObject* thrower);
        struct FlyingObjectHit
        {
            std::weak_ptr<Object> mObject;
            const btCollisionObject* mActor;
            btScalar mSpeed;
        };
        /// Hits since the last call.
        std::vector<FlyingObjectHit> takeFlyingObjectHits();

        /// A new contact of a simulated body (item or ragdoll part) with something, hard enough to be heard.
        struct Impact
        {
            std::weak_ptr<PtrHolder> mBody; // the simulated one
            std::weak_ptr<PtrHolder> mOther; // empty for terrain
            bool mOtherSimulated;
            bool mOtherTerrain;
            btVector3 mPoint;
            btVector3 mNormal; // towards mBody
            btScalar mSpeed; // the change of speed at the contact
            btScalar mBodyMass;
        };
        /// Collect impacts with at least this speed change (0: don't).
        void setImpactMinSpeed(float speed) { mImpactMinSpeed = speed; }
        /// Impacts since the last call.
        std::vector<Impact> takeImpacts();
        void removeCollisionObject(btCollisionObject* collisionObject);
        void updateSingleAabb(const std::shared_ptr<PtrHolder>& ptr, bool immediate = false);
        bool getLineOfSight(const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2);
        void debugDraw();
        void* getUserPointer(const btCollisionObject* object) const;
        void releaseSharedStates(); // destroy all objects whose destructor can't be safely called from
                                    // ~PhysicsTaskScheduler()

    private:
        class WorkersSync;

        void doSimulation();
        void worker();
        void updateActorsPositions();
        bool hasLineOfSight(const Actor* actor1, const Actor* actor2);
        void refreshLOSCache();
        void updateAabbs();
        void updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr);
        void updateStats(Misc::FrameClock::time_point statsEpoch, unsigned int frameNumber, Misc::FrameStats& stats);
        std::tuple<unsigned, float> calculateStepConfig(float timeAccum) const;
        void afterPreStep();
        void afterPostStep();
        void afterPostSim();
        void pushDynamicObjects();
        void stepDynamics();
        void releaseHeldObjectUnsafe(btRigidBody& body, const std::optional<btVector3>& velocity);
        void applyWaterForces(float waterHeight);
        void freeWedgedObjectUnsafe(const std::shared_ptr<Object>& object);
        void setStuckUnsafe(Object& object, bool stuck);
        void updateWedgedObjects(const std::vector<std::shared_ptr<Object>>& objects);
        struct FlyingObject
        {
            std::weak_ptr<Object> mObject;
            const btCollisionObject* mThrower;
        };
        void updateFlyingObjects(const std::vector<std::pair<std::shared_ptr<Object>, FlyingObject>>& objects);
        void syncWithMainThread();
        void waitForWorkers();
        void prepareWork(float& timeAccum, std::vector<Simulation>& simulations,
            Misc::FrameClock::time_point statsEpoch, unsigned int frameNumber, Misc::FrameStats& stats);

        std::unique_ptr<WorldFrameData> mWorldFrameData;
        std::vector<Simulation>* mSimulations = nullptr;
        std::unordered_set<const btCollisionObject*> mCollisionObjects;
        float mDefaultPhysicsDt;
        float mPhysicsDt;
        float mTimeAccum;
        btCollisionWorld* mCollisionWorld;
        std::vector<LOSRequest> mLOSCache;
        std::set<std::weak_ptr<PtrHolder>, std::owner_less<std::weak_ptr<PtrHolder>>> mUpdateAabb;

        // TODO: use std::experimental::flex_barrier or std::barrier once it becomes a thing
        std::unique_ptr<Misc::Barrier> mPreStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostStepBarrier;
        std::unique_ptr<Misc::Barrier> mPostSimBarrier;

        LockingPolicy mLockingPolicy;
        unsigned mNumThreads;
        int mNumJobs;
        unsigned mRemainingSteps;
        int mLOSCacheExpiry;
        bool mAdvanceSimulation;
        std::atomic<int> mNextJob;
        std::atomic<int> mNextLOS;
        std::vector<std::thread> mThreads;

        mutable std::shared_mutex mSimulationMutex;
        mutable std::shared_mutex mCollisionWorldMutex;
        mutable std::shared_mutex mLOSCacheMutex;
        mutable std::mutex mUpdateAabbMutex;

        unsigned int mFrameNumber;

        unsigned mPrevStepCount;
        Misc::BudgetMeasurement mBudget;
        Misc::BudgetMeasurement mAsyncBudget;
        unsigned int mBudgetCursor;
        Misc::FrameClock::time_point mAsyncStartTime;
        Misc::FrameClock::time_point mTimeBegin;
        Misc::FrameClock::time_point mTimeEnd;
        Misc::FrameClock::time_point mStatsEpoch;

        std::unique_ptr<WorkersSync> mWorkersSync;
    };

}
#endif
