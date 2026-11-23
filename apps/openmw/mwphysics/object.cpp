#include "object.hpp"
#include "mtphysics.hpp"

#include <components/bullethelpers/collisionobject.hpp>
#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/nifosg/particle.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <LinearMath/btTransform.h>

#include "dynamictransform.hpp"

namespace MWPhysics
{
    Object::Object(const MWWorld::Ptr& ptr, osg::ref_ptr<Resource::BulletShapeInstance> shapeInstance,
        osg::Quat rotation, int collisionType, PhysicsTaskScheduler* scheduler)
        : PtrHolder(ptr, osg::Vec3f())
        , mShapeInstance(std::move(shapeInstance))
        , mSolid(true)
        , mScale(ptr.getCellRef().getScale(), ptr.getCellRef().getScale(), ptr.getCellRef().getScale())
        , mPosition(ptr.getRefData().getPosition().asVec3())
        , mRotation(rotation)
        , mTaskScheduler(scheduler)
        , mCollidedWith(ScriptedCollisionType_None)
    {
        mCollisionObject = BulletHelpers::makeCollisionObject(mShapeInstance->mCollisionShape.get(),
            Misc::Convert::toBullet(mPosition), Misc::Convert::toBullet(rotation));
        mCollisionObject->setUserPointer(this);
        mShapeInstance->setLocalScaling(mScale);
        mTaskScheduler->addCollisionObject(mCollisionObject.get(), collisionType,
            CollisionType_Actor | CollisionType_HeightMap | CollisionType_Projectile | CollisionType_Dynamic);
    }

    Object::~Object()
    {
        mTaskScheduler->removeCollisionObject(mCollisionObject.get());
        // Destroy the rigid body before its collision shape (base owns the body).
        mCollisionObject.reset();
    }

    // Dynamic bodies (makeStatic, makeDynamic, applyDynamicImpulse) are
    // derived from sunspotdark/morrowind-modern-physics
    // (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40), GPLv3.
    void Object::makeStatic()
    {
        if (!btRigidBody::upcast(mCollisionObject.get())) return;
        const auto ptr = getPtr();
        const auto& position = ptr.getRefData().getPosition();
        const auto rotation = Misc::Convert::makeOsgQuat(position);
        mCollisionObject = BulletHelpers::makeCollisionObject(mShapeInstance->mCollisionShape.get(),
            Misc::Convert::toBullet(position.asVec3()), Misc::Convert::toBullet(rotation));
        mCollisionObject->setUserPointer(this);
        mDynamicShape.reset();
        mDynamicMass = 0.f;
        mDynamicTransformReady = false;
    }

    void Object::makeDynamic(float mass)
    {
        if (btRigidBody::upcast(mCollisionObject.get()))
            return;
        btVector3 minimum, maximum;
        mCollisionObject->getCollisionShape()->getAabb(btTransform::getIdentity(), minimum, maximum);
        btVector3 extent = (maximum - minimum) * 0.5f;
        for (int axis = 0; axis < 3; ++axis)
            extent[axis] = std::clamp(extent[axis], btScalar(0.5), btScalar(256));
        mDynamicCenter = (maximum + minimum) * 0.5f;
        mDynamicInitialScale = mScale.x();
        mDynamicMass = mass;
        mDynamicShape = std::make_unique<btBoxShape>(extent);
        mDynamicShape->setMargin(0.05f);
        btVector3 inertia;
        mDynamicShape->calculateLocalInertia(mass, inertia);
        auto body = std::make_unique<btRigidBody>(mass, nullptr, mDynamicShape.get(), inertia);
        btTransform transform = mCollisionObject->getWorldTransform();
        transform.setOrigin(transform * mDynamicCenter);
        body->setWorldTransform(transform);
        body->setInterpolationWorldTransform(transform);
        body->setUserPointer(this);
        body->setFriction(0.7f);
        body->setRestitution(0.2f);
        body->setDamping(0.05f, 0.15f);
        body->setSleepingThresholds(1.f, 0.1f);
        body->setCcdMotionThreshold(1.f);
        body->setCcdSweptSphereRadius(std::max(btScalar(0.25), extent[extent.minAxis()] * 0.5f));
        mCollisionObject = std::move(body);
    }

    void Object::makeReleasedDynamic(float mass, const std::vector<osg::Vec3f>& vertices,
        const osg::Vec3f& frameLinearVelocity, const osg::Vec3f& angularVelocity)
    {
        if (btRigidBody::upcast(mCollisionObject.get()))
            throw std::runtime_error("Released equipment already has a rigid body");
        if (!std::isfinite(mass) || mass < .05f || mass > 500.f || vertices.size() < 4
            || vertices.size() > 262144)
            throw std::runtime_error("Invalid released equipment mass or geometry");
        btVector3 minimum(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT), maximum = -minimum;
        for (const auto& vertex : vertices)
        {
            for (int axis = 0; axis < 3; ++axis)
                if (!std::isfinite(vertex[axis]) || std::abs(vertex[axis]) > 4096.f)
                    throw std::runtime_error("Invalid released equipment vertex");
            const auto point = Misc::Convert::toBullet(vertex) * mScale.x();
            minimum.setMin(point);
            maximum.setMax(point);
        }
        const btVector3 center = (minimum + maximum) * .5f;
        auto shape = std::make_unique<btConvexHullShape>();
        shape->setMargin(.05f);
        for (const auto& vertex : vertices)
            shape->addPoint(Misc::Convert::toBullet(vertex) * mScale.x() - center, false);
        shape->recalcLocalAabb();
        shape->optimizeConvexHull();
        btVector3 inertia;
        shape->calculateLocalInertia(mass, inertia);
        btTransform transform = mCollisionObject->getWorldTransform();
        const auto offset = transform.getBasis() * center;
        const auto angular = Misc::Convert::toBullet(angularVelocity);
        const auto linear = Misc::Convert::toBullet(frameLinearVelocity) + angular.cross(offset);
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(linear[axis]) || !std::isfinite(angular[axis])
                || !std::isfinite(inertia[axis]) || inertia[axis] <= 0.f)
                throw std::runtime_error("Invalid released equipment motion or inertia");
        if (linear.length2() > 2000.f * 2000.f || angular.length2() > 200.f * 200.f)
            throw std::runtime_error("Released equipment motion exceeds numeric bound");
        transform.setOrigin(transform.getOrigin() + offset);
        auto body = std::make_unique<btRigidBody>(mass, nullptr, shape.get(), inertia);
        body->setWorldTransform(transform);
        body->setInterpolationWorldTransform(transform);
        body->setLinearVelocity(linear);
        body->setInterpolationLinearVelocity(linear);
        body->setAngularVelocity(angular);
        body->setInterpolationAngularVelocity(angular);
        body->setUserPointer(this);
        body->setFriction(.7f);
        body->setRestitution(0.f);
        body->setDamping(.05f, .15f);
        body->setSleepingThresholds(1.f, .1f);
        const auto halfExtent = (maximum - minimum) * .5f;
        const float inscribedRadius = std::max(.01f, float(halfExtent[halfExtent.minAxis()]));
        body->setCcdMotionThreshold(std::max(.05f, inscribedRadius));
        body->setCcdSweptSphereRadius(std::min(.5f, inscribedRadius * .5f));
        mDynamicCenter = center;
        mDynamicInitialScale = mScale.x();
        mDynamicMass = mass;
        mDynamicShape = std::move(shape);
        mCollisionObject = std::move(body);
    }

    void Object::applyDynamicImpulse(const osg::Vec3f& impulse, float mass)
    {
        auto* body = btRigidBody::upcast(mCollisionObject.get());
        if (!body)
            return;
        if (mass != mDynamicMass)
        {
            btVector3 inertia;
            mDynamicShape->calculateLocalInertia(mass, inertia);
            body->setMassProps(mass, inertia);
            body->updateInertiaTensor();
            mDynamicMass = mass;
        }
        body->activate(true);
        body->applyCentralImpulse(Misc::Convert::toBullet(impulse));
        btVector3 velocity = body->getLinearVelocity();
        if (velocity.length2() > 2000.f * 2000.f)
            body->setLinearVelocity(velocity.normalized() * 2000.f);
    }

    void Object::captureDynamicTransform()
    {
        const auto* body = btRigidBody::upcast(mCollisionObject.get());
        if (!body)
            return;
        std::lock_guard<std::mutex> lock(mPositionMutex);
        // A script teleport takes precedence over a completed older simulation.
        if (mTransformUpdatePending || mScaleUpdatePending)
            return;
        btTransform transform = body->getWorldTransform();
        transform = dynamicItemRoot(transform, mDynamicCenter, mScale.x() / mDynamicInitialScale);
        mDynamicTransformReady = mDynamicTransformReady || !(transform == mDynamicTransform);
        mDynamicTransform = transform;
        mPosition = Misc::Convert::toOsg(transform.getOrigin());
        mRotation = Misc::Convert::toOsg(transform.getRotation());
    }

    bool Object::takeDynamicTransforms(const btTransform& renderedCenterOfMass,
        btTransform& authoritative, btTransform& rendered, bool& authoritativeChanged)
    {
        std::lock_guard<std::mutex> lock(mPositionMutex);
        const auto* body = btRigidBody::upcast(mCollisionObject.get());
        if (!body || mTransformUpdatePending || mScaleUpdatePending)
            return false;
        const auto ratio = mScale.x() / mDynamicInitialScale;
        authoritative = dynamicItemRoot(body->getWorldTransform(), mDynamicCenter, ratio);
        rendered = dynamicItemRoot(renderedCenterOfMass, mDynamicCenter, ratio);
        authoritativeChanged = mDynamicTransformReady || !(authoritative == mDynamicTransform);
        mDynamicTransform = authoritative;
        mDynamicTransformReady = false;
        mPosition = Misc::Convert::toOsg(authoritative.getOrigin());
        mRotation = Misc::Convert::toOsg(authoritative.getRotation());
        return true;
    }

    const Resource::BulletShapeInstance* Object::getShapeInstance() const
    {
        return mShapeInstance.get();
    }

    void Object::setScale(float scale)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mScale = { scale, scale, scale };
        mScaleUpdatePending = true;
    }

    void Object::setRotation(osg::Quat quat)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mRotation = quat;
        mTransformUpdatePending = true;
    }

    void Object::updatePosition()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPosition = getPtr().getRefData().getPosition().asVec3();
        mTransformUpdatePending = true;
    }

    bool Object::commitPositionChange()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        bool changed = false;
        if (mScaleUpdatePending)
        {
            mShapeInstance->setLocalScaling(mScale);
            if (auto* body = btRigidBody::upcast(mCollisionObject.get()))
            {
                mDynamicShape->setLocalScaling(mScale / mDynamicInitialScale);
                btVector3 inertia;
                mDynamicShape->calculateLocalInertia(mDynamicMass, inertia);
                body->setMassProps(mDynamicMass, inertia);
                body->updateInertiaTensor();
                body->activate(true);
                mTransformUpdatePending = true;
            }
            mScaleUpdatePending = false;
        }
        if (mTransformUpdatePending)
        {
            changed = true;
            btTransform trans;
            trans.setOrigin(Misc::Convert::toBullet(mPosition));
            trans.setRotation(Misc::Convert::toBullet(mRotation));
            if (auto* body = btRigidBody::upcast(mCollisionObject.get()))
            {
                trans.setOrigin(trans * (mDynamicCenter * (mScale.x() / mDynamicInitialScale)));
                body->setLinearVelocity(btVector3(0, 0, 0));
                body->setAngularVelocity(btVector3(0, 0, 0));
                body->clearForces();
                body->setInterpolationWorldTransform(trans);
                body->activate(true);
            }
            mCollisionObject->setWorldTransform(trans);
            mTransformUpdatePending = false;
        }
        return changed;
    }

    btTransform Object::getTransform() const
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        btTransform trans;
        trans.setOrigin(Misc::Convert::toBullet(mPosition));
        trans.setRotation(Misc::Convert::toBullet(mRotation));
        return trans;
    }

    bool Object::isSolid() const
    {
        return mSolid;
    }

    void Object::setSolid(bool solid)
    {
        mSolid = solid;
    }

    bool Object::isAnimated() const
    {
        return mShapeInstance->isAnimated();
    }

    bool Object::animateCollisionShapes()
    {
        if (mShapeInstance->mAnimatedShapes.empty())
            return false;

        if (!getPtr().getRefData().getBaseNode())
            return false;

        assert(mShapeInstance->mCollisionShape->isCompound());

        btCompoundShape* compound = static_cast<btCompoundShape*>(mShapeInstance->mCollisionShape.get());
        bool result = false;
        for (const auto& [recordIndex, shapeIndex] : mShapeInstance->mAnimatedShapes)
        {
            auto nodePathFound = mRecordIndexToNodePath.find(recordIndex);
            if (nodePathFound == mRecordIndexToNodePath.end())
            {
                NifOsg::FindGroupByRecordIndex visitor(recordIndex);
                getPtr().getRefData().getBaseNode()->accept(visitor);
                if (!visitor.mFound)
                {
                    Log(Debug::Warning) << "Warning: animateCollisionShapes can't find node " << recordIndex << " for "
                                        << mPtr.getCellRef().getRefId();

                    // Remove nonexistent nodes from animated shapes map and early out
                    mShapeInstance->mAnimatedShapes.erase(recordIndex);
                    return false;
                }
                osg::NodePath nodePath = visitor.mFoundPath;
                nodePath.erase(nodePath.begin());
                nodePathFound = mRecordIndexToNodePath.emplace(recordIndex, nodePath).first;
            }

            osg::NodePath& nodePath = nodePathFound->second;
            osg::Matrixf matrix = osg::computeLocalToWorld(nodePath);
            btVector3 scale = Misc::Convert::toBullet(matrix.getScale());
            matrix.orthoNormalize(matrix);

            btTransform transform;
            transform.setOrigin(Misc::Convert::toBullet(matrix.getTrans()) * compound->getLocalScaling());
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    transform.getBasis()[i][j] = matrix(j, i); // NB column/row major difference

            btCollisionShape* childShape = compound->getChildShape(shapeIndex);
            btVector3 newScale = compound->getLocalScaling() * scale;

            if (childShape->getLocalScaling() != newScale)
            {
                childShape->setLocalScaling(newScale);
                result = true;
            }

            if (!(transform == compound->getChildTransform(shapeIndex)))
            {
                compound->updateChildTransform(shapeIndex, transform);
                result = true;
            }
        }
        return result;
    }

    bool Object::collidedWith(ScriptedCollisionType type) const
    {
        return mCollidedWith & type;
    }

    void Object::addCollision(ScriptedCollisionType type)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mCollidedWith |= type;
    }

    void Object::resetCollisions()
    {
        mCollidedWith = ScriptedCollisionType_None;
    }
}
