#include "actor.hpp"

#include <BulletCollision/CollisionShapes/btCylinderShape.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/class.hpp"

#include "collisiontype.hpp"
#include "contacttestwrapper.h"
#include "mtphysics.hpp"
#include "playerkinetics.hpp"
#include "trace.h"

#include <cmath>

namespace MWPhysics
{
    namespace
    {
        class StandingClearanceCallback final : public btCollisionWorld::ContactResultCallback
        {
        public:
            explicit StandingClearanceCallback(const btCollisionObject* actor)
                : mActor(actor)
            {
                m_collisionFilterGroup = CollisionType_Actor;
                m_collisionFilterMask = actor->getBroadphaseHandle()->m_collisionFilterMask
                    & ~CollisionType_Projectile;
            }

            bool needsCollision(btBroadphaseProxy* proxy) const override
            {
                return proxy->m_clientObject != mActor && ContactResultCallback::needsCollision(proxy);
            }

            btScalar addSingleResult(btManifoldPoint& contact, const btCollisionObjectWrapper*, int, int,
                const btCollisionObjectWrapper*, int, int) override
            {
                // Resting contact and round-off are not an obstruction. Check the
                // complete standing volume so overhangs and thin ceilings count.
                mBlocked |= contact.getDistance() < -0.01f;
                return 0;
            }

            bool mBlocked = false;

        private:
            const btCollisionObject* mActor;
        };
    }

    Actor::Actor(const MWWorld::Ptr& ptr, const Resource::BulletShape* shape, PhysicsTaskScheduler* scheduler,
        bool canWaterWalk, DetourNavigator::CollisionShapeType collisionShapeType)
        : PtrHolder(ptr, ptr.getRefData().getPosition().asVec3())
        , mStandingOnPtr(nullptr)
        , mCanWaterWalk(canWaterWalk)
        , mWalkingOnWater(false)
        , mMeshTranslation(shape->mCollisionBox.mCenter)
        , mOriginalHalfExtents(shape->mCollisionBox.mExtents)
        , mStuckFrames(0)
        , mLastStuckPosition{ 0, 0, 0 }
        , mForce(0.f, 0.f, 0.f)
        , mOnGround(ptr.getClass().getCreatureStats(ptr).getFallHeight() == 0)
        , mOnSlope(false)
        , mInternalCollisionMode(true)
        , mExternalCollisionMode(true)
        , mActive(false)
        , mTaskScheduler(scheduler)
    {
        // We can not create actor without collisions - he will fall through the ground.
        // In this case we should autogenerate collision box based on mesh shape
        // (NPCs have bodyparts and use a different approach)
        if (!ptr.getClass().isNpc() && mOriginalHalfExtents.length2() == 0.f)
        {
            if (shape->mCollisionShape)
            {
                btTransform transform;
                transform.setIdentity();
                btVector3 min;
                btVector3 max;

                shape->mCollisionShape->getAabb(transform, min, max);
                mOriginalHalfExtents.x() = static_cast<float>((max[0] - min[0]) / 2.f);
                mOriginalHalfExtents.y() = static_cast<float>((max[1] - min[1]) / 2.f);
                mOriginalHalfExtents.z() = static_cast<float>((max[2] - min[2]) / 2.f);

                mMeshTranslation = osg::Vec3f(0.f, 0.f, mOriginalHalfExtents.z());
            }

            if (mOriginalHalfExtents.length2() == 0.f)
                Log(Debug::Error) << "Error: Failed to calculate bounding box for actor \""
                                  << ptr.getCellRef().getRefId() << "\".";
        }

        const btVector3 halfExtents = Misc::Convert::toBullet(mOriginalHalfExtents);
        float extRatio = 0.f;
        if (mOriginalHalfExtents.y() != 0.f)
            extRatio = mOriginalHalfExtents.x() / mOriginalHalfExtents.y();

        if (mMeshTranslation.x() == 0.0 && mMeshTranslation.y() == 0.0 && extRatio >= 1.f / 1.1f && extRatio <= 1.1f)
        {
            switch (collisionShapeType)
            {
                case DetourNavigator::CollisionShapeType::Aabb:
                    mShape = std::make_unique<btBoxShape>(halfExtents);
                    mRotationallyInvariant = true;
                    break;
                case DetourNavigator::CollisionShapeType::RotatingBox:
                    mShape = std::make_unique<btBoxShape>(halfExtents);
                    mRotationallyInvariant = false;
                    break;
                case DetourNavigator::CollisionShapeType::Cylinder:
                    mShape = std::make_unique<btCylinderShapeZ>(halfExtents);
                    mRotationallyInvariant = true;
                    break;
            }
            mCollisionShapeType = collisionShapeType;
        }
        else
        {
            mShape = std::make_unique<btBoxShape>(halfExtents);
            mRotationallyInvariant = false;
            mCollisionShapeType = DetourNavigator::CollisionShapeType::RotatingBox;
        }

        mConvexShape = static_cast<btConvexShape*>(mShape.get());
        mConvexShape->setMargin(0.001); // make sure bullet isn't using the huge default convex shape margin of 0.04

        mCollisionObject = std::make_unique<btCollisionObject>();
        mCollisionObject->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        mCollisionObject->setActivationState(DISABLE_DEACTIVATION);
        mCollisionObject->setCollisionShape(mShape.get());
        mCollisionObject->setUserPointer(this);

        updateScaleUnsafe();

        if (!mRotationallyInvariant)
        {
            const SceneUtil::PositionAttitudeTransform* baseNode = mPtr.getRefData().getBaseNode();
            if (baseNode)
                mRotation = baseNode->getAttitude();
        }

        addCollisionMask(getCollisionMask());
        updateCollisionObjectPositionUnsafe();
    }

    Actor::~Actor()
    {
        mTaskScheduler->removeCollisionObject(mCollisionObject.get());
    }

    void Actor::enableCollisionMode(bool collision)
    {
        mInternalCollisionMode = collision;
    }

    void Actor::enableCollisionBody(bool collision)
    {
        if (mExternalCollisionMode != collision)
        {
            mExternalCollisionMode = collision;
            updateCollisionMask();
        }
    }

    void Actor::addCollisionMask(int collisionMask)
    {
        mTaskScheduler->addCollisionObject(mCollisionObject.get(), CollisionType_Actor, collisionMask);
    }

    void Actor::updateCollisionMask()
    {
        mTaskScheduler->setCollisionFilterMask(mCollisionObject.get(), getCollisionMask());
    }

    void Actor::setRagdoll(bool value)
    {
        mRagdoll = value;
        updateCollisionMask();
    }

    int Actor::getCollisionMask() const
    {
        if (mRagdoll)
            return 0;
        int collisionMask = CollisionType_World | CollisionType_HeightMap;
        if (mExternalCollisionMode)
            collisionMask |= CollisionType_Actor | CollisionType_Projectile | CollisionType_Door;
        if (mCanWaterWalk)
            collisionMask |= CollisionType_Water;
        return collisionMask;
    }

    void Actor::updatePosition()
    {
        std::scoped_lock lock(mPositionMutex);
        const auto worldPosition = mPtr.getRefData().getPosition().asVec3();
        mPreviousPosition = worldPosition;
        mPosition = worldPosition;
        mSimulationPosition = worldPosition;
        // A scripted relocation is not a measured movement for ragdoll inheritance.
        mSimulationVelocity = osg::Vec3f();
        mPositionOffset = osg::Vec3f();
        mStandingOnPtr = nullptr;
        mSkipSimulation = true;
        clearExternalVelocity();
    }

    bool Actor::setSimulationPosition(const osg::Vec3f& position)
    {
        if (std::exchange(mSkipSimulation, false))
            return false;
        mSimulationPosition = position;
        return true;
    }

    osg::Vec3f Actor::getScaledMeshTranslation() const
    {
        osg::Vec3f translation = osg::componentMultiply(mMeshTranslation, mScale);
        if (mSlideCollisionLow)
            translation.z() -= mOriginalHalfExtents.z() * mScale.z() * (1.f - PlayerKinetics::SlideHeightFraction);
        return mRotation * translation;
    }

    bool Actor::updateSlideCollision(bool low, btCollisionWorld* world)
    {
        std::scoped_lock lock(mPositionMutex);
        if (!mSlideCollisionLow && !low)
            return false;

        bool blocked = false;
        if (mSlideCollisionLow && mInternalCollisionMode)
        {
            // Probe a separate standing shape. Never temporarily enlarge the
            // live body: other queries must only see a shape we can occupy.
            std::unique_ptr<btConvexShape> standingShape;
            const btVector3 extents = Misc::Convert::toBullet(mOriginalHalfExtents);
            if (mCollisionShapeType == DetourNavigator::CollisionShapeType::Cylinder)
                standingShape = std::make_unique<btCylinderShapeZ>(extents);
            else
                standingShape = std::make_unique<btBoxShape>(extents);
            standingShape->setMargin(mConvexShape->getMargin());
            standingShape->setLocalScaling(Misc::Convert::toBullet(mScale));

            btCollisionObject probe;
            probe.setCollisionShape(standingShape.get());
            btTransform transform;
            transform.setIdentity();
            transform.setRotation(Misc::Convert::toBullet(mRotation));
            const osg::Vec3f center
                = mPosition + mRotation * osg::componentMultiply(mMeshTranslation, mScale);
            transform.setOrigin(Misc::Convert::toBullet(center));
            probe.setWorldTransform(transform);
            StandingClearanceCallback callback(mCollisionObject.get());
            ContactTestWrapper::contactTest(world, &probe, callback);
            blocked = callback.mBlocked;
            if (blocked && !low)
                return true;
        }

        // Report overhead clearance even while the low hull is still requested.
        // Camera/exit animation must know before they begin rising, rather than
        // finding out after a standing shape has already been requested.
        if (mSlideCollisionLow == low)
            return blocked;

        mSlideCollisionLow = low;
        mHalfExtents.z() = mOriginalHalfExtents.z() * mScale.z()
            * (low ? PlayerKinetics::SlideHeightFraction : 1.f);
        updateCollisionObjectPositionUnsafe();
        world->updateSingleAabb(mCollisionObject.get());
        return blocked;
    }

    void Actor::updateCollisionObjectPosition()
    {
        std::scoped_lock lock(mPositionMutex);
        updateCollisionObjectPositionUnsafe();
    }

    void Actor::updateCollisionObjectPositionUnsafe()
    {
        osg::Vec3f collisionScale = mScale;
        if (mSlideCollisionLow)
            collisionScale.z() *= PlayerKinetics::SlideHeightFraction;
        mShape->setLocalScaling(Misc::Convert::toBullet(collisionScale));
        osg::Vec3f newPosition = getScaledMeshTranslation() + mPosition;

        auto& trans = mCollisionObject->getWorldTransform();
        trans.setOrigin(Misc::Convert::toBullet(newPosition));
        trans.setRotation(Misc::Convert::toBullet(mRotation));
        mCollisionObject->setWorldTransform(trans);
    }

    osg::Vec3f Actor::getCollisionObjectPosition() const
    {
        std::scoped_lock lock(mPositionMutex);
        return getScaledMeshTranslation() + mPosition;
    }

    bool Actor::setPosition(const osg::Vec3f& position)
    {
        std::scoped_lock lock(mPositionMutex);
        const bool worldPositionChanged = mPositionOffset.length2() != 0;
        applyOffsetChange();
        if (worldPositionChanged || mSkipSimulation)
            return true;
        mPreviousPosition = mPosition;
        mPosition = position;
        return mPreviousPosition != mPosition;
    }

    void Actor::adjustPosition(const osg::Vec3f& offset)
    {
        std::scoped_lock lock(mPositionMutex);
        mPositionOffset += offset;
    }

    osg::Vec3f Actor::applyOffsetChange()
    {
        if (mPositionOffset.length2() != 0)
        {
            mPosition += mPositionOffset;
            mPreviousPosition += mPositionOffset;
            mSimulationPosition += mPositionOffset;
            mPositionOffset = osg::Vec3f();
        }
        return mPosition;
    }

    void Actor::setRotation(osg::Quat quat)
    {
        std::scoped_lock lock(mPositionMutex);
        mRotation = quat;
    }

    bool Actor::isRotationallyInvariant() const
    {
        return mRotationallyInvariant;
    }

    void Actor::updateScale()
    {
        std::scoped_lock lock(mPositionMutex);
        updateScaleUnsafe();
    }

    void Actor::updateScaleUnsafe()
    {
        float scale = mPtr.getCellRef().getScale();
        osg::Vec3f scaleVec(scale, scale, scale);

        mPtr.getClass().adjustScale(mPtr, scaleVec, false);
        mScale = scaleVec;
        mHalfExtents = osg::componentMultiply(mOriginalHalfExtents, scaleVec);
        if (mSlideCollisionLow)
            mHalfExtents.z() *= PlayerKinetics::SlideHeightFraction;

        scaleVec = osg::Vec3f(scale, scale, scale);
        mPtr.getClass().adjustScale(mPtr, scaleVec, true);
        mRenderingHalfExtents = osg::componentMultiply(mOriginalHalfExtents, scaleVec);
    }

    osg::Vec3f Actor::getHalfExtents() const
    {
        return mHalfExtents;
    }

    osg::Vec3f Actor::getOriginalHalfExtents() const
    {
        return mOriginalHalfExtents;
    }

    osg::Vec3f Actor::getRenderingHalfExtents() const
    {
        return mRenderingHalfExtents;
    }

    void Actor::addExternalVelocity(const osg::Vec3f& delta)
    {
        mPendingExternalVelocity += delta;
        const float speed = mPendingExternalVelocity.length();
        if (speed > 2000.f)
            mPendingExternalVelocity *= 2000.f / speed;
    }

    void Actor::clearExternalVelocity()
    {
        mSimulationVelocity=osg::Vec3f();
        mPendingExternalVelocity = osg::Vec3f();
        mSlideActorContactTime = 0.f;
        mClearExternalVelocity = true;
    }

    osg::Vec3f Actor::consumeExternalVelocity(bool& clear)
    {
        const osg::Vec3f delta = mPendingExternalVelocity;
        mPendingExternalVelocity = osg::Vec3f();
        clear = mClearExternalVelocity;
        mClearExternalVelocity = false;
        return delta;
    }

    void Actor::setInertialForce(const osg::Vec3f& force)
    {
        mForce = force;
    }

    void Actor::setOnGround(bool grounded)
    {
        mOnGround = grounded;
    }

    void Actor::setOnSlope(bool slope)
    {
        mOnSlope = slope;
    }

    bool Actor::isWalkingOnWater() const
    {
        return mWalkingOnWater;
    }

    void Actor::setWalkingOnWater(bool walkingOnWater)
    {
        mWalkingOnWater = walkingOnWater;
    }

    void Actor::setCanWaterWalk(bool waterWalk)
    {
        if (waterWalk != mCanWaterWalk)
        {
            mCanWaterWalk = waterWalk;
            updateCollisionMask();
        }
    }

    MWWorld::Ptr Actor::getStandingOnPtr() const
    {
        std::scoped_lock lock(mPositionMutex);
        return mStandingOnPtr;
    }

    void Actor::setStandingOnPtr(const MWWorld::Ptr& ptr)
    {
        std::scoped_lock lock(mPositionMutex);
        mStandingOnPtr = ptr;
    }

    bool Actor::canMoveToWaterSurface(float waterlevel, const btCollisionWorld* world) const
    {
        const float halfZ = getHalfExtents().z();
        const osg::Vec3f actorPosition = getPosition();
        const osg::Vec3f startingPosition(actorPosition.x(), actorPosition.y(), actorPosition.z() + halfZ);
        const osg::Vec3f destinationPosition(actorPosition.x(), actorPosition.y(), waterlevel + halfZ);
        MWPhysics::ActorTracer tracer;
        tracer.doTrace(getCollisionObject(), startingPosition, destinationPosition, world);
        return (tracer.mFraction >= 1.0f);
    }

}
