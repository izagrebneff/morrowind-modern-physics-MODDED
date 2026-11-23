#ifndef OPENMW_MWPHYSICS_PHYSICSSYSTEM_H
#define OPENMW_MWPHYSICS_PHYSICSSYSTEM_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include <osg/BoundingBox>
#include <osg/Matrixf>
#include <osg/Quat>
#include <osg/Timer>
#include <osg/ref_ptr>

#include <components/vfs/pathutil.hpp>

#include "../mwworld/ptr.hpp"

#include "collisiontype.hpp"
#include "raycasting.hpp"

namespace osg
{
    class Group;
    class Object;
    class Stats;
}

namespace MWRender
{
    class DebugDrawer;
}

namespace Resource
{
    class BulletShapeManager;
    class ResourceSystem;
}

class btCollisionWorld;
class btSequentialImpulseConstraintSolver;
class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btCollisionObject;
class btCollisionShape;
class btVector3;

namespace MWPhysics
{
    class HeightField;
    class Object;
    class Actor;
    class Ragdoll;
    class PhysicsTaskScheduler;
    class Projectile;
    enum ScriptedCollisionType : char;

    using ActorMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Actor>>;

    struct ContactPoint
    {
        MWWorld::Ptr mObject;
        osg::Vec3f mPoint;
        osg::Vec3f mNormal;
    };

    struct LOSRequest
    {
        LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2);
        std::array<std::weak_ptr<Actor>, 2> mActors;
        std::array<const Actor*, 2> mRawActors;
        bool mResult;
        bool mStale;
        int mAge;
    };
    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept;

    struct ActorFrameData
    {
        ActorFrameData(Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer);
        osg::Vec3f mPosition;
        osg::Vec3f mInertia;
        osg::Vec3f mExternalVelocity;
        osg::Vec3f mLocomotionVelocity;
        osg::Vec3f mSimulationVelocity;
        const bool mKineticMovement;
        const bool mBrutalSliding;
        const bool mBrutalSlideLow;
        const float mBrutalSlideFrictionBlend; // Active glide, then the phase-3 exit only.
        bool mBrutalSlideDownhill = false;
        osg::Vec3f mBrutalSlideGroundNormal;
        osg::Vec3f mBrutalSlideGroundPoint;
        float mSlideActorContactTime = 0.f;
        float mExternalImpactSpeed = 0.f;
        osg::Vec3f mExternalImpactPosition;
        osg::Vec3f mExternalImpactNormal;
        const btCollisionObject* mExternalImpactObject = nullptr;
        const btCollisionObject* mStandingOn;
        bool mIsOnGround;
        bool mIsOnSlope;
        bool mWalkingOnWater;
        const bool mInert;
        btCollisionObject* mCollisionObject;
        const float mSwimLevel;
        const float mSlowFall;
        osg::Vec2f mRotation;
        osg::Vec3f mMovement;
        osg::Vec3f mLastStuckPosition;
        const float mWaterlevel;
        float mHalfExtentsZ;
        float mOldHeight;
        unsigned int mStuckFrames;
        const bool mFlying;
        const bool mWasOnGround;
        const bool mIsAquatic;
        const bool mWaterCollision;
        const bool mSkipCollisionDetection;
        const bool mIsPlayer;
    };

    struct ProjectileFrameData
    {
        explicit ProjectileFrameData(Projectile& projectile);
        osg::Vec3f mPosition;
        osg::Vec3f mMovement;
        const btCollisionObject* mCaster;
        const btCollisionObject* mCollisionObject;
        Projectile* mProjectile;
    };

    struct WorldFrameData
    {
        WorldFrameData();
        bool mIsInStorm;
        osg::Vec3f mStormDirection;
    };

    template <class Ptr, class FrameData>
    class SimulationImpl
    {
    public:
        explicit SimulationImpl(const std::weak_ptr<Ptr>& ptr, FrameData&& data)
            : mPtr(ptr)
            , mData(data)
        {
        }

        std::optional<std::pair<std::shared_ptr<Ptr>, std::reference_wrapper<FrameData>>> lock()
        {
            if (auto locked = mPtr.lock())
                return { { std::move(locked), std::ref(mData) } };
            return std::nullopt;
        }

    private:
        std::weak_ptr<Ptr> mPtr;
        FrameData mData;
    };

    using ActorSimulation = SimulationImpl<Actor, ActorFrameData>;
    using ProjectileSimulation = SimulationImpl<Projectile, ProjectileFrameData>;
    using Simulation = std::variant<ActorSimulation, ProjectileSimulation>;

    class PhysicsSystem : public RayCastingInterface
    {
    public:
        PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode);
        virtual ~PhysicsSystem();

        Resource::BulletShapeManager* getShapeManager();

        void enableWater(float height);
        void setWaterHeight(float height);
        void disableWater();

        void addObject(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation,
            int collisionType = CollisionType_World);
        void addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh);

        int addProjectile(
            const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius);
        void setCaster(int projectileId, const MWWorld::Ptr& caster);
        void removeProjectile(const int projectileId);

        void updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated);

        Actor* getActor(const MWWorld::Ptr& ptr);
        const Actor* getActor(const MWWorld::ConstPtr& ptr) const;

        const Object* getObject(const MWWorld::ConstPtr& ptr) const;

        Projectile* getProjectile(int projectileId) const;

        // Object or Actor
        void remove(const MWWorld::Ptr& ptr);

        void updateScale(const MWWorld::Ptr& ptr);
        void updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate);
        void updatePosition(const MWWorld::Ptr& ptr);

        void addHeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
            const osg::Object* holdObject);

        void removeHeightField(int x, int y);

        const HeightField* getHeightField(int x, int y) const;

        bool toggleCollisionMode();

        /// Determine new position based on all queued movements, then clear the list.
        void stepSimulation(
            float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);

        /// Apply new positions to actors
        void moveActors();
        void applyObjectImpulse(const MWWorld::Ptr& ptr, const osg::Vec3f& impulse, float mass);
        void addReleasedEquipment(const MWWorld::Ptr& ptr);
        void initializeReleasedEquipment(const MWWorld::Ptr& item, const MWWorld::Ptr& actor,
            const osg::Vec3f& linear, const osg::Vec3f& angular);
        bool getRagdollEquipmentFrame(const MWWorld::Ptr& actor, bool left,
            osg::Matrixf& frame, osg::Vec3f& linear, osg::Vec3f& angular) const;
        std::vector<osg::Vec3f> getReleasedEquipmentState(const MWWorld::Ptr& item) const;
        void moveDynamicObjects();
        void createNpcRagdoll(const MWWorld::Ptr& ptr,
            const std::map<std::string, osg::Matrixf, std::less<>>& bones, const osg::Vec3f& kick);
        void stopNpcRagdoll(const MWWorld::Ptr& ptr);
        bool detachNpcRagdollHead(const MWWorld::Ptr& ptr, const osg::Vec3f& cutVelocity,
            const osg::Vec3f& headAngularImpulse);
        bool detachNpcRagdollTorso(const MWWorld::Ptr& ptr, const osg::Vec3f& cutVelocity,
            const osg::Vec3f& upperSeparation, const std::string& savedPose = {});
        std::string getNpcBisectionPoseData(const MWWorld::Ptr& ptr) const;
        bool detachNpcRagdollLimbs(const MWWorld::Ptr& ptr, unsigned mask,
            const osg::Vec3f& cutVelocity, const std::string& savedPose = {});
        std::string getNpcLimbSeveringPoseData(const MWWorld::Ptr& ptr) const;
        unsigned getNpcSeveredLimbsMask(const MWWorld::Ptr& ptr) const;
        void moveNpcRagdolls(float duration);
        // Called after OSG has evaluated this frame's animation controllers.
        void sampleNpcRagdollAnimation(float duration);
        bool applyNpcKick(const MWWorld::Ptr& ptr, const osg::Vec3f& kick, const MWWorld::Ptr& attacker, bool continuing = false,
            bool knockDown = false, bool slideTrip = false);
        bool applyNpcSuperKick(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity,
            const MWWorld::Ptr& attacker, std::uint64_t kickId);
        std::map<std::string, double> getNpcSuperKickState(const MWWorld::Ptr& ptr) const;
        std::vector<osg::Vec3f> getNpcRagdollDynamics(const MWWorld::Ptr& ptr) const;
        std::vector<osg::Vec3f> getNpcRagdollState(const MWWorld::Ptr& ptr) const;
        std::map<std::string, double> getNpcRagdollEnhancementState(const MWWorld::Ptr& ptr) const;
        bool applyNpcRagdollImpact(const MWWorld::Ptr& ptr, std::size_t part, const osg::Vec3f& point,
            const osg::Vec3f& impulse, std::uint64_t id);
        // Kicks the nearest reachable ragdoll part (body or severed head). Returns the corpse,
        // or an empty Ptr; headHit reports a severed head.
        MWWorld::Ptr kickNpcRagdoll(const osg::Vec3f& origin, const osg::Vec3f& forward, float reach,
            float minimumDot, const osg::Vec3f& velocity, bool& headHit, osg::Vec3f& hitPoint, std::size_t& hitPart);
        void debugDraw();

        std::vector<MWWorld::Ptr> getCollisions(const MWWorld::ConstPtr& ptr, int collisionGroup,
            int collisionMask) const; ///< get handles this object collides with
        std::vector<ContactPoint> getCollisionsPoints(
            const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const;
        osg::Vec3f traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight);

        /// @param ignore Optional, a list of Ptr to ignore in the list of results. targets are actors to filter for,
        /// ignoring all other actors.
        RayCastingResult castRay(const osg::Vec3f& from, const osg::Vec3f& to,
            const std::vector<MWWorld::ConstPtr>& ignore = {}, const std::vector<MWWorld::Ptr>& targets = {},
            int mask = CollisionType_Default, int group = 0xff) const override;
        using RayCastingInterface::castRay;

        RayCastingResult castSphere(const osg::Vec3f& from, const osg::Vec3f& to, float radius,
            int mask = CollisionType_Default, int group = 0xff) const override;

        /// Return true if actor1 can see actor2.
        bool getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const override;

        bool isOnGround(const MWWorld::Ptr& actor);

        bool canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel);

        /// Get physical half extents (scaled) of the given actor.
        osg::Vec3f getHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get physical half extents (not scaled) of the given actor.
        osg::Vec3f getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// @see MWPhysics::Actor::getRenderingHalfExtents
        osg::Vec3f getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get the position of the collision shape for the actor. Use together with getHalfExtents() to get the
        /// collision bounds in world space.
        /// @note The collision shape's origin is in its center, so the position returned can be described as center of
        /// the actor collision box in world space.
        osg::Vec3f getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const;

        /// Get bounding box in world space of the given object.
        osg::BoundingBox getBoundingBox(const MWWorld::ConstPtr& object) const;

        /// Queues velocity movement for a Ptr. If a Ptr is already queued, its velocity will
        /// be overwritten. Valid until the next call to stepSimulation
        void queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity);

        /// Clear the queued movements list without applying.
        void clearQueuedMovement();

        /// Return true if \a actor has been standing on \a object in this frame
        /// This will trigger whenever the object is directly below the actor.
        /// It doesn't matter if the actor is stationary or moving.
        bool isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const;

        /// Get the handle of all actors standing on \a object in this frame.
        void getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        /// Return true if an object of the given type has collided with this object
        bool isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const;

        /// Get the handle of all actors colliding with \a object in this frame.
        void getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        bool toggleDebugRendering();

        /// Mark the given object as a 'non-solid' object. A non-solid object means that
        /// \a isOnSolidGround will return false for actors standing on that object.
        void markAsNonSolid(const MWWorld::ConstPtr& ptr);

        bool isOnSolidGround(const MWWorld::Ptr& actor) const;

        void updateAnimatedCollisionShape(const MWWorld::Ptr& object);

        template <class Function>
        void forEachAnimatedObject(Function&& function) const
        {
            std::for_each(mAnimatedObjects.begin(), mAnimatedObjects.end(), function);
        }

        bool isAreaOccupiedByOtherActor(
            const MWWorld::LiveCellRefBase* actor, const osg::Vec3f& position, float radius) const;

        void reportStats(unsigned int frameNumber, osg::Stats& stats) const;
        void reportCollision(const btVector3& position, const btVector3& normal);

        float mPhysicsDt;

    private:
        void updateWater();
        bool applyNpcKickImpl(const MWWorld::Ptr& ptr, const osg::Vec3f& kick, const MWWorld::Ptr& attacker,
            bool continuing, bool knockDown, bool superKick, bool slideTrip = false);
        void refreshSuperKickActors();
        void processSuperKickContacts();

        void prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations);

        std::unique_ptr<btBroadphaseInterface> mBroadphase;
        std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfiguration;
        std::unique_ptr<btCollisionDispatcher> mDispatcher;
        std::unique_ptr<btSequentialImpulseConstraintSolver> mDynamicsSolver;
        std::unique_ptr<btCollisionWorld> mCollisionWorld;
        std::unique_ptr<PhysicsTaskScheduler> mTaskScheduler;

        std::unique_ptr<Resource::BulletShapeManager> mShapeManager;
        Resource::ResourceSystem* mResourceSystem;

        using ObjectMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Object>>;
        struct PushRagdoll { float age=0.f, settled=0.f, peakZ=0.f, previousZ=0.f, downSpeed=0.f, unsupported=0.f;
            bool landed=false, supported=false, knockDown=false; MWWorld::Ptr attacker; };
        std::unordered_map<const MWWorld::LiveCellRefBase*, PushRagdoll> mPushRagdolls;
        struct SuperKickSource
        {
            MWWorld::Ptr attacker;
            std::uint64_t kickId = 0;
            unsigned knockdowns = 0, rejectedContacts = 0;
        };
        std::unordered_map<const MWWorld::LiveCellRefBase*, SuperKickSource> mSuperKickSources;
        std::unordered_map<const MWWorld::LiveCellRefBase*, std::uint64_t> mSuperKickVictims;
        // Accepted nonces remain consumed when a source recovers or is removed;
        // replaying the queued action must never create another powered launch.
        std::unordered_set<std::uint64_t> mSuperKickIds;
        struct KickWatch { MWWorld::Ptr actor, attacker; osg::Vec3f direction; float age=0.f; float lift=0.f; };
        std::unordered_map<const MWWorld::LiveCellRefBase*, KickWatch> mKickWatches;
        ObjectMap mObjects;
        std::unordered_map<const MWWorld::LiveCellRefBase*, std::unique_ptr<Ragdoll>> mRagdolls;
        struct RagdollAnimationSample
        {
            std::map<std::string, osg::Matrixf, std::less<>> bones;
            std::map<std::string, std::pair<osg::Vec3f, osg::Vec3f>, std::less<>> motion;
            osg::Vec3f renderedRootPosition;
            float deadAge = 0.f;
        };
        std::unordered_map<const MWWorld::LiveCellRefBase*, RagdollAnimationSample> mRagdollAnimation;

        std::map<Object*, bool> mAnimatedObjects; // stores pointers to elements in mObjects

        ActorMap mActors;

        using ProjectileMap = std::map<int, std::shared_ptr<Projectile>>;
        ProjectileMap mProjectiles;

        using HeightFieldMap = std::map<std::pair<int, int>, std::unique_ptr<HeightField>>;
        HeightFieldMap mHeightFields;

        bool mDebugDrawEnabled;

        float mTimeAccum;

        unsigned int mProjectileId;

        float mWaterHeight;
        bool mWaterEnabled;

        std::unique_ptr<btCollisionObject> mWaterCollisionObject;
        std::unique_ptr<btCollisionShape> mWaterCollisionShape;

        std::unique_ptr<MWRender::DebugDrawer> mDebugDrawer;

        osg::ref_ptr<osg::Group> mParentNode;

        std::size_t mSimulationsCounter = 0;
        std::array<std::vector<Simulation>, 2> mSimulations;
        std::vector<std::pair<MWWorld::Ptr, osg::Vec3f>> mActorsPositions;

        PhysicsSystem(const PhysicsSystem&);
        PhysicsSystem& operator=(const PhysicsSystem&);
    };
}

#endif
