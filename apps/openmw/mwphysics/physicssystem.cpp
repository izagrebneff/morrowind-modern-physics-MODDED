#include "physicssystem.hpp"

#include <algorithm>
#include <optional>
#include <memory>
#include <vector>

#include <osg/Group>
#include <osg/Stats>
#include <osg/Timer>

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <LinearMath/btQuickprof.h>
#include <LinearMath/btVector3.h>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadgmst.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/rng.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/conversion.hpp>
#include <components/resource/bulletshapemanager.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/esm3/releasedequipmentstate.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "../mwbase/luamanager.hpp"
#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/character.hpp"
#include "../mwmechanics/damagesourcetype.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwmechanics/movement.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/player.hpp"

#include "../mwrender/bulletdebugdraw.hpp"

#include "../mwworld/class.hpp"

#include "actor.hpp"
#include "creaturerig.hpp"
#include "ragdoll.hpp"
#include <components/esm3/loadcrea.hpp>
#include "../mwrender/animation.hpp"
#include "collisiontype.hpp"

#include "closestnotmerayresultcallback.hpp"
#include "contacttestresultcallback.hpp"
#include "hasspherecollisioncallback.hpp"
#include "heightfield.hpp"
#include "movementsolver.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"

namespace
{
    void handleJump(const MWWorld::Ptr& ptr)
    {
        if (!ptr.getClass().isActor())
            return;
        if (ptr.getClass().getMovementSettings(ptr).mPosition[2] == 0)
            return;
        const bool isPlayer = (ptr == MWMechanics::getPlayer());
        // Advance acrobatics and set flag for GetPCJumping
        if (isPlayer)
        {
            ptr.getClass().skillUsageSucceeded(ptr, ESM::Skill::Acrobatics, ESM::Skill::Acrobatics_Jump);
            MWBase::Environment::get().getWorld()->getPlayer().setJumping(true);
        }

        // Decrease fatigue
        if (!isPlayer || (!Settings::game().mBrutalStamina
                && !MWBase::Environment::get().getWorld()->getGodModeState()))
        {
            const MWWorld::Store<ESM::GameSetting>& gmst
                = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
            const float fFatigueJumpBase = gmst.find("fFatigueJumpBase")->mValue.getFloat();
            const float fFatigueJumpMult = gmst.find("fFatigueJumpMult")->mValue.getFloat();
            const float normalizedEncumbrance = std::min(1.f, ptr.getClass().getNormalizedEncumbrance(ptr));
            const float fatigueDecrease = fFatigueJumpBase + normalizedEncumbrance * fFatigueJumpMult;
            MWMechanics::DynamicStat<float> fatigue = ptr.getClass().getCreatureStats(ptr).getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - fatigueDecrease);
            ptr.getClass().getCreatureStats(ptr).setFatigue(fatigue);
        }
        ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;
    }

}

namespace MWPhysics
{
    PhysicsSystem::PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode)
        : mPhysicsDt(1.f / 60.f)
        , mShapeManager(std::make_unique<Resource::BulletShapeManager>(resourceSystem->getVFS(),
              resourceSystem->getSceneManager(), resourceSystem->getNifFileManager(),
              Settings::cells().mCacheExpiryDelay))
        , mResourceSystem(resourceSystem)
        , mDebugDrawEnabled(false)
        , mTimeAccum(0.0f)
        , mProjectileId(0)
        , mWaterHeight(0)
        , mWaterEnabled(false)
        , mParentNode(std::move(parentNode))
    {
        mResourceSystem->addResourceManager(mShapeManager.get());

        mCollisionConfiguration = std::make_unique<btDefaultCollisionConfiguration>();
        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfiguration.get());
        mBroadphase = std::make_unique<btDbvtBroadphase>();

        mDynamicsSolver = std::make_unique<btSequentialImpulseConstraintSolver>();
        auto dynamics = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mBroadphase.get(),
            mDynamicsSolver.get(), mCollisionConfiguration.get());
        dynamics->setGravity(btVector3(0, 0, -627.2f));
        dynamics->getSolverInfo().m_numIterations = 12;
        mCollisionWorld = std::move(dynamics);

        // Don't update AABBs of all objects every frame. Most objects in MW are static, so we don't need this.
        // Should a "static" object ever be moved, we have to update its AABB manually using
        // DynamicsWorld::updateSingleAabb.
        mCollisionWorld->setForceUpdateAllAabbs(false);

        // Check if a user decided to override a physics system FPS
        if (const char* env = getenv("OPENMW_PHYSICS_FPS"))
        {
            if (const auto physFramerate = Misc::StringUtils::toNumeric<float>(env);
                physFramerate.has_value() && *physFramerate > 0)
            {
                mPhysicsDt = 1.f / *physFramerate;
                Log(Debug::Warning) << "Warning: using custom physics framerate (" << *physFramerate << " FPS).";
            }
        }

        mDebugDrawer = std::make_unique<MWRender::DebugDrawer>(mParentNode, mCollisionWorld.get(), mDebugDrawEnabled);
        mTaskScheduler = std::make_unique<PhysicsTaskScheduler>(mPhysicsDt, mCollisionWorld.get(), mDebugDrawer.get());
    }

    PhysicsSystem::~PhysicsSystem()
    {
        mResourceSystem->removeResourceManager(mShapeManager.get());

        if (mWaterCollisionObject)
            mTaskScheduler->removeCollisionObject(mWaterCollisionObject.get());

        mTaskScheduler->releaseSharedStates();
        mHeightFields.clear();
        mObjects.clear();
        mPushRagdolls.clear();
        mSuperKickSources.clear();
        mSuperKickVictims.clear();
        mKickWatches.clear();
        mRagdolls.clear();
        mActors.clear();
        mProjectiles.clear();
    }

    Resource::BulletShapeManager* PhysicsSystem::getShapeManager()
    {
        return mShapeManager.get();
    }

    bool PhysicsSystem::toggleDebugRendering()
    {
        mDebugDrawEnabled = !mDebugDrawEnabled;

        mCollisionWorld->setDebugDrawer(mDebugDrawEnabled ? mDebugDrawer.get() : nullptr);
        mDebugDrawer->setDebugMode(mDebugDrawEnabled);
        return mDebugDrawEnabled;
    }

    void PhysicsSystem::markAsNonSolid(const MWWorld::ConstPtr& ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr.mRef);
        if (found == mObjects.end())
            return;

        found->second->setSolid(false);
    }

    bool PhysicsSystem::isOnSolidGround(const MWWorld::Ptr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (!physactor || !physactor->getOnGround() || !physactor->getCollisionMode())
            return false;

        const auto obj = physactor->getStandingOnPtr();
        if (obj.isEmpty())
            return true; // assume standing on terrain (which is a non-object, so not collision tracked)

        ObjectMap::const_iterator foundObj = mObjects.find(obj.mRef);
        if (foundObj == mObjects.end())
            return false;

        if (!foundObj->second->isSolid())
            return false;

        return true;
    }

    RayCastingResult PhysicsSystem::castRay(const osg::Vec3f& from, const osg::Vec3f& to,
        const std::vector<MWWorld::ConstPtr>& ignore, const std::vector<MWWorld::Ptr>& targets, int mask,
        int group) const
    {
        if (from == to)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }
        btVector3 btFrom = Misc::Convert::toBullet(from);
        btVector3 btTo = Misc::Convert::toBullet(to);

        std::vector<const btCollisionObject*> ignoreList;
        std::vector<const btCollisionObject*> targetCollisionObjects;

        for (const auto& ptr : ignore)
        {
            if (!ptr.isEmpty())
            {
                const Actor* actor = getActor(ptr);
                if (actor)
                {
                    ignoreList.push_back(actor->getCollisionObject());
                    // A ragdoll replaces the standing hull with several bodies.
                    // Explicitly ignoring the actor must ignore all those parts.
                    if (const auto ragdoll = mRagdolls.find(ptr.mRef); ragdoll != mRagdolls.end())
                    {
                        const auto& bodies = ragdoll->second->getCollisionBodies();
                        ignoreList.insert(ignoreList.end(), bodies.begin(), bodies.end());
                    }
                }
                else
                {
                    const Object* object = getObject(ptr);
                    if (object)
                        ignoreList.push_back(object->getCollisionObject());
                }
            }
        }

        if (!targets.empty())
        {
            for (const MWWorld::Ptr& target : targets)
            {
                const Actor* actor = getActor(target);
                if (actor)
                    targetCollisionObjects.push_back(actor->getCollisionObject());
            }
        }

        ClosestNotMeRayResultCallback resultCallback(ignoreList, targetCollisionObjects, btFrom, btTo);
        resultCallback.m_collisionFilterGroup = group;
        resultCallback.m_collisionFilterMask = mask;

        mTaskScheduler->rayTest(btFrom, btTo, resultCallback);

        RayCastingResult result;
        result.mHit = resultCallback.hasHit();
        if (resultCallback.hasHit())
        {
            result.mHitPos = Misc::Convert::toOsg(resultCallback.m_hitPointWorld);
            result.mHitNormal = Misc::Convert::toOsg(resultCallback.m_hitNormalWorld);
            if (PtrHolder* ptrHolder = static_cast<PtrHolder*>(resultCallback.m_collisionObject->getUserPointer()))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    RayCastingResult PhysicsSystem::castSphere(
        const osg::Vec3f& from, const osg::Vec3f& to, float radius, int mask, int group) const
    {
        btCollisionWorld::ClosestConvexResultCallback callback(
            Misc::Convert::toBullet(from), Misc::Convert::toBullet(to));
        callback.m_collisionFilterGroup = group;
        callback.m_collisionFilterMask = mask;

        btSphereShape shape(radius);
        const btQuaternion btrot = btQuaternion::getIdentity();

        mTaskScheduler->convexSweepTest(&shape, btTransform(btrot, Misc::Convert::toBullet(from)),
            btTransform(btrot, Misc::Convert::toBullet(to)), callback);

        RayCastingResult result;
        result.mHit = callback.hasHit();
        if (result.mHit)
        {
            result.mHitPos = Misc::Convert::toOsg(callback.m_hitPointWorld);
            result.mHitNormal = Misc::Convert::toOsg(callback.m_hitNormalWorld);
            if (auto* ptrHolder = static_cast<PtrHolder*>(callback.m_hitCollisionObject->getUserPointer()))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    bool PhysicsSystem::getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const
    {
        if (actor1 == actor2)
            return true;

        const auto it1 = mActors.find(actor1.mRef);
        const auto it2 = mActors.find(actor2.mRef);
        if (it1 == mActors.end() || it2 == mActors.end())
            return false;

        return mTaskScheduler->getLineOfSight(it1->second, it2->second);
    }

    bool PhysicsSystem::isOnGround(const MWWorld::Ptr& actor)
    {
        Actor* physactor = getActor(actor);
        return physactor && physactor->getOnGround() && physactor->getCollisionMode();
    }

    bool PhysicsSystem::canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel)
    {
        const auto* physactor = getActor(actor);
        return physactor && physactor->canMoveToWaterSurface(waterlevel, mCollisionWorld.get());
    }

    osg::Vec3f PhysicsSystem::getHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        if (const Actor* physactor = getActor(actor))
            return physactor->getOriginalHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getRenderingHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::BoundingBox PhysicsSystem::getBoundingBox(const MWWorld::ConstPtr& object) const
    {
        const Object* physobject = getObject(object);
        if (!physobject)
            return osg::BoundingBox();
        btVector3 min, max;
        mTaskScheduler->getAabb(physobject->getCollisionObject(), min, max);
        return osg::BoundingBox(Misc::Convert::toOsg(min), Misc::Convert::toOsg(max));
    }

    osg::Vec3f PhysicsSystem::getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getCollisionObjectPosition();
        else
            return osg::Vec3f();
    }

    std::vector<ContactPoint> PhysicsSystem::getCollisionsPoints(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        btCollisionObject* me = nullptr;

        auto found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            me = found->second->getCollisionObject();
        else
            return {};

        ContactTestResultCallback resultCallback(me);
        resultCallback.m_collisionFilterGroup = collisionGroup;
        resultCallback.m_collisionFilterMask = collisionMask;
        mTaskScheduler->contactTest(me, resultCallback);
        return resultCallback.mResult;
    }

    std::vector<MWWorld::Ptr> PhysicsSystem::getCollisions(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        std::vector<MWWorld::Ptr> actors;
        for (auto& [actor, point, normal] : getCollisionsPoints(ptr, collisionGroup, collisionMask))
            actors.emplace_back(actor);
        return actors;
    }

    osg::Vec3f PhysicsSystem::traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found == mActors.end())
            return ptr.getRefData().getPosition().asVec3();
        return MovementSolver::traceDown(ptr, position, found->second.get(), mCollisionWorld.get(), maxHeight);
    }

    void PhysicsSystem::addHeightField(
        const float* heights, int x, int y, int size, int verts, float minH, float maxH, const osg::Object* holdObject)
    {
        mHeightFields[std::make_pair(x, y)]
            = std::make_unique<HeightField>(heights, x, y, size, verts, minH, maxH, holdObject, mTaskScheduler.get());
    }

    void PhysicsSystem::removeHeightField(int x, int y)
    {
        HeightFieldMap::iterator heightfield = mHeightFields.find(std::make_pair(x, y));
        if (heightfield != mHeightFields.end())
            mHeightFields.erase(heightfield);
    }

    const HeightField* PhysicsSystem::getHeightField(int x, int y) const
    {
        const auto heightField = mHeightFields.find(std::make_pair(x, y));
        if (heightField == mHeightFields.end())
            return nullptr;
        return heightField->second.get();
    }

    void PhysicsSystem::addObject(
        const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation, int collisionType)
    {
        if (ptr.mRef->mData.mPhysicsPostponed)
            return;

        const VFS::Path::Normalized animationMesh = ptr.getClass().useAnim()
            ? Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS())
            : VFS::Path::Normalized(mesh);
        osg::ref_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(animationMesh);
        if (!shapeInstance || !shapeInstance->mCollisionShape)
            return;

        assert(!getObject(ptr));

        // Override collision type based on shape content.
        switch (shapeInstance->mVisualCollisionType)
        {
            case Resource::VisualCollisionType::None:
                break;
            case Resource::VisualCollisionType::Default:
                collisionType = CollisionType_VisualOnly;
                break;
            case Resource::VisualCollisionType::Camera:
                collisionType = CollisionType_CameraOnly;
                break;
        }

        auto obj = std::make_shared<Object>(ptr, shapeInstance, rotation, collisionType, mTaskScheduler.get());
        mObjects.emplace(ptr.mRef, obj);

        if (obj->isAnimated())
            mAnimatedObjects.emplace(obj.get(), false);
    }

    void PhysicsSystem::remove(const MWWorld::Ptr& ptr)
    {
        stopNpcRagdoll(ptr);
        mKickWatches.erase(ptr.mRef);
        mRagdollAnimation.erase(ptr.mRef);
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            mAnimatedObjects.erase(foundObject->second.get());

            mObjects.erase(foundObject);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            mActors.erase(foundActor);
        }
    }

    void PhysicsSystem::removeProjectile(const int projectileId)
    {
        ProjectileMap::iterator foundProjectile = mProjectiles.find(projectileId);
        if (foundProjectile != mProjectiles.end())
            mProjectiles.erase(foundProjectile);
    }

    void PhysicsSystem::updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated)
    {
        if (old.mRef != updated.mRef)
        {
            if (auto source = mSuperKickSources.extract(old.mRef); !source.empty())
            { source.key() = updated.mRef; mSuperKickSources.insert(std::move(source)); }
            if (auto victim = mSuperKickVictims.extract(old.mRef); !victim.empty())
            { victim.key() = updated.mRef; mSuperKickVictims.insert(std::move(victim)); }
        }
        for (auto& [ref, source] : mSuperKickSources)
            if (source.attacker == old) source.attacker = updated;
        if (old.mRef != updated.mRef)
            if (auto sample = mRagdollAnimation.extract(old.mRef); !sample.empty())
            { sample.key() = updated.mRef; mRagdollAnimation.insert(std::move(sample)); }
        if (auto watch=mKickWatches.extract(old.mRef); !watch.empty())
        { watch.key()=updated.mRef; watch.mapped().actor=updated; mKickWatches.insert(std::move(watch)); }
        if (auto found = mRagdolls.find(old.mRef); found != mRagdolls.end())
        {
            found->second->updatePtr(updated);
            if (auto push=mPushRagdolls.extract(old.mRef); !push.empty())
            { push.key()=updated.mRef; mPushRagdolls.insert(std::move(push)); }
            if (old.mRef != updated.mRef)
            {
                auto ragdoll = std::move(found->second);
                mRagdolls.erase(found);
                mRagdolls.emplace(updated.mRef, std::move(ragdoll));
            }
        }
        if (auto foundObject = mObjects.find(old.mRef); foundObject != mObjects.end())
            foundObject->second->updatePtr(updated);
        else if (auto foundActor = mActors.find(old.mRef); foundActor != mActors.end())
            foundActor->second->updatePtr(updated);

        for (auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == old)
                actor->setStandingOnPtr(updated);
        }

        for (auto& [_, projectile] : mProjectiles)
        {
            if (projectile->getCaster() == old)
                projectile->setCaster(updated);
        }
    }

    Actor* PhysicsSystem::getActor(const MWWorld::Ptr& ptr)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Actor* PhysicsSystem::getActor(const MWWorld::ConstPtr& ptr) const
    {
        ActorMap::const_iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Object* PhysicsSystem::getObject(const MWWorld::ConstPtr& ptr) const
    {
        ObjectMap::const_iterator found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            return found->second.get();
        return nullptr;
    }

    Projectile* PhysicsSystem::getProjectile(int projectileId) const
    {
        ProjectileMap::const_iterator found = mProjectiles.find(projectileId);
        if (found != mProjectiles.end())
            return found->second.get();
        return nullptr;
    }

    void PhysicsSystem::updateScale(const MWWorld::Ptr& ptr)
    {
        mRagdollAnimation.erase(ptr.mRef);
        stopNpcRagdoll(ptr);
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            float scale = ptr.getCellRef().getScale();
            foundObject->second->setScale(scale);
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            foundActor->second->updateScale();
            mTaskScheduler->updateSingleAabb(foundActor->second);
        }
    }

    void PhysicsSystem::updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate)
    {
        stopNpcRagdoll(ptr);
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            foundObject->second->setRotation(rotate);
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            if (!foundActor->second->isRotationallyInvariant())
            {
                foundActor->second->setRotation(rotate);
                mTaskScheduler->updateSingleAabb(foundActor->second);
            }
        }
    }

    void PhysicsSystem::updatePosition(const MWWorld::Ptr& ptr)
    {
        mRagdollAnimation.erase(ptr.mRef);
        stopNpcRagdoll(ptr);
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            foundObject->second->updatePosition();
            mTaskScheduler->updateSingleAabb(foundObject->second);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            foundActor->second->updatePosition();
            mTaskScheduler->updateSingleAabb(foundActor->second, true);
        }
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh)
    {
        const VFS::Path::Normalized animationMesh
            = Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS());
        osg::ref_ptr<const Resource::BulletShape> shape = mShapeManager->getShape(animationMesh);

        // Try to get shape from basic model as fallback for creatures
        if (!ptr.getClass().isNpc() && shape && shape->mCollisionBox.mExtents.length2() == 0)
        {
            if (animationMesh != mesh)
            {
                shape = mShapeManager->getShape(mesh);
            }
        }

        if (!shape)
            return;

        // check if Actor should spawn above water
        const MWMechanics::MagicEffects& effects = ptr.getClass().getCreatureStats(ptr).getMagicEffects();
        const bool canWaterWalk = effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude() > 0;

        auto actor = std::make_shared<Actor>(
            ptr, shape, mTaskScheduler.get(), canWaterWalk, Settings::game().mActorCollisionShapeType);

        mActors.emplace(ptr.mRef, std::move(actor));
    }

    int PhysicsSystem::addProjectile(
        const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius)
    {
        osg::ref_ptr<Resource::BulletShapeInstance> shapeInstance = mShapeManager->getInstance(mesh);
        assert(shapeInstance);
        float radius = computeRadius ? shapeInstance->mCollisionBox.mExtents.length() / 2.f : 1.f;

        mProjectileId++;

        auto projectile = std::make_shared<Projectile>(caster, position, radius, mTaskScheduler.get(), this);
        mProjectiles.emplace(mProjectileId, std::move(projectile));

        return mProjectileId;
    }

    void PhysicsSystem::setCaster(int projectileId, const MWWorld::Ptr& caster)
    {
        const auto foundProjectile = mProjectiles.find(projectileId);
        assert(foundProjectile != mProjectiles.end());
        auto* projectile = foundProjectile->second.get();

        projectile->setCaster(caster);
    }

    bool PhysicsSystem::toggleCollisionMode()
    {
        ActorMap::iterator found = mActors.find(MWMechanics::getPlayer().mRef);
        if (found != mActors.end())
        {
            bool cmode = found->second->getCollisionMode();
            cmode = !cmode;
            found->second->enableCollisionMode(cmode);
            // NB: Collision body isn't disabled for vanilla TCL compatibility
            return cmode;
        }

        return false;
    }

    void PhysicsSystem::queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            found->second->setVelocity(velocity);
    }

    void PhysicsSystem::clearQueuedMovement()
    {
        for (const auto& [_, actor] : mActors)
        {
            actor->setVelocity(osg::Vec3f());
            actor->setInertialForce(osg::Vec3f());
            actor->clearExternalVelocity();
        }
    }

    void PhysicsSystem::prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations)
    {
        assert(simulations.empty());
        simulations.reserve(mActors.size() + mProjectiles.size());
        const MWBase::World* world = MWBase::Environment::get().getWorld();
        for (const auto& [ref, physicActor] : mActors)
        {
            if (!physicActor->isActive() || physicActor->isRagdoll())
                continue;

            auto ptr = physicActor->getPtr();
            if (!ptr.getClass().isMobile(ptr))
                continue;

            const MWWorld::CellStore& cell = *ptr.getCell();
            auto& stats = ptr.getClass().getCreatureStats(ptr);
            const MWMechanics::MagicEffects& effects = stats.getMagicEffects();

            float waterlevel = -std::numeric_limits<float>::max();
            bool waterCollision = false;
            if (cell.getCell()->hasWater())
            {
                waterlevel = cell.getWaterLevel();
                if (physicActor->getCollisionMode())
                    waterCollision = effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude();
            }

            physicActor->setCanWaterWalk(waterCollision);

            // Slow fall reduces fall speed by a factor of (effect magnitude / 200)
            const float slowFall
                = 1.f - std::clamp(effects.getOrDefault(ESM::MagicEffect::SlowFall).getMagnitude() * 0.005f, 0.f, 1.f);
            const bool isPlayer = ptr == world->getPlayerConstPtr();
            const bool godmode = isPlayer && world->getGodModeState();
            if (stats.isDead())
                physicActor->clearExternalVelocity();
            const bool inert = stats.isDead()
                || (!godmode && stats.getMagicEffects().getOrDefault(ESM::MagicEffect::Paralyze).getModifier() > 0);
            if (isPlayer && (physicActor->isExternalVelocityResetPending() || inert
                    || !Settings::game().mBrutalSlide))
            {
                stats.mBrutalSlideRequested = false;
                stats.mBrutalSliding = false;
                stats.mBrutalSlideLow = false;
                stats.mBrutalSlideDownhill = false;
                stats.mBrutalSlideGroundNormal = osg::Vec3f();
                stats.mBrutalSlideGroundPoint = osg::Vec3f();
            }

            simulations.emplace_back(ActorSimulation{
                physicActor, ActorFrameData{ *physicActor, inert, waterCollision, slowFall, waterlevel, isPlayer } });

            // if the simulation will run, a jump request will be fulfilled. Update mechanics accordingly.
            if (willSimulate && stats.mBrutalSliding)
                ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;
            else if (willSimulate)
                handleJump(ptr);
        }

        for (const auto& [id, projectile] : mProjectiles)
        {
            simulations.emplace_back(ProjectileSimulation{ projectile, ProjectileFrameData{ *projectile } });
        }
    }

    void PhysicsSystem::stepSimulation(
        float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        for (auto& [animatedObject, changed] : mAnimatedObjects)
        {
            if (animatedObject->animateCollisionShapes())
            {
                auto obj = mObjects.find(animatedObject->getPtr().mRef);
                assert(obj != mObjects.end());
                mTaskScheduler->updateSingleAabb(obj->second);
                changed = true;
            }
            else
            {
                changed = false;
            }
        }
        for (auto& [_, object] : mObjects)
            object->resetCollisions();

#ifndef BT_NO_PROFILE
        CProfileManager::Reset();
        CProfileManager::Increment_Frame_Counter();
#endif

        mTimeAccum += dt;

        if (skipSimulation)
            mTaskScheduler->resetSimulation(mActors);
        else
        {
            std::vector<Simulation>& simulations = mSimulations[mSimulationsCounter++ % mSimulations.size()];
            prepareSimulation(mTimeAccum >= mPhysicsDt, simulations);
            // modifies mTimeAccum
            mTaskScheduler->applyQueuedMovements(mTimeAccum, simulations, frameStart, frameNumber, stats);
        }
    }

    void PhysicsSystem::applyObjectImpulse(const MWWorld::Ptr& ptr, const osg::Vec3f& impulse, float mass)
    {
        if (!Settings::game().mBrutalDynamicProps) return;
        auto found = mObjects.find(ptr.mRef);
        if (found == mObjects.end())
        {
            // Stock inventory items are rendered without a physics object.
            // Create their model collision shape only when explicitly promoted.
            addObject(ptr, ptr.getClass().getCorrectedModel(ptr), ptr.getRefData().getBaseNode()->getAttitude());
            found = mObjects.find(ptr.mRef);
        }
        if (found == mObjects.end() || found->second->isAnimated() || !found->second->isSolid())
            throw std::runtime_error("Dynamic prop requires a solid, non-animated collision shape");
        mTaskScheduler->applyObjectImpulse(found->second, impulse, mass);
    }

    void PhysicsSystem::addReleasedEquipment(const MWWorld::Ptr& ptr)
    {
        if (!Settings::game().mBrutalEquipmentPhysics) return;
        const auto& release = ptr.getRefData().getReleasedEquipment();
        if (!release || !release->valid())
            throw std::runtime_error("Invalid released equipment descriptor");
        if (mObjects.contains(ptr.mRef))
            return;
        std::vector<osg::Vec3f> vertices;
        auto hull = std::make_unique<btConvexHullShape>();
        for (const auto& drawable : release->mDrawables)
        {
            const osg::Matrixf residual(drawable.mTransform.data());
            for (const auto& value : drawable.mVertices)
            {
                vertices.push_back(osg::Vec3f(value[0], value[1], value[2]) * residual);
                hull->addPoint(Misc::Convert::toBullet(vertices.back()), false);
            }
        }
        hull->setMargin(.05f);
        hull->recalcLocalAabb();
        auto shape = osg::ref_ptr<Resource::BulletShape>(new Resource::BulletShape);
        shape->mCollisionShape.reset(hull.release());
        // Like ordinary loose inventory models, this visual-derived dynamic
        // hull is never a static navigation obstacle. The existing scene and
        // world gates then also skip navigator update/removal on pickup/unload.
        // CollisionType_World below still explicitly owns physical contacts.
        shape->mVisualCollisionType = Resource::VisualCollisionType::Default;
        auto instance = Resource::makeInstance(shape);
        auto object = std::make_shared<Object>(ptr, instance, ptr.getRefData().getBaseNode()->getAttitude(),
            CollisionType_World, mTaskScheduler.get());
        mTaskScheduler->addReleasedObject(object, vertices, release->mMass);
        mObjects.emplace(ptr.mRef, std::move(object));
    }

    void PhysicsSystem::initializeReleasedEquipment(const MWWorld::Ptr& item, const MWWorld::Ptr& actor,
        const osg::Vec3f& linear, const osg::Vec3f& angular)
    {
        const auto object = mObjects.find(item.mRef);
        const auto ragdoll = mRagdolls.find(actor.mRef);
        if (object == mObjects.end() || ragdoll == mRagdolls.end())
            throw std::runtime_error("Released equipment or owner body is missing");
        mTaskScheduler->setReleasedObjectMotion(object->second, linear, angular, ragdoll->second->getEquipmentGripBodies());
    }

    bool PhysicsSystem::getRagdollEquipmentFrame(const MWWorld::Ptr& actor, bool left,
        osg::Matrixf& frame, osg::Vec3f& linear, osg::Vec3f& angular) const
    {
        const auto found = mRagdolls.find(actor.mRef);
        return found != mRagdolls.end() && found->second->getEquipmentFrame(left, frame, linear, angular);
    }

    std::vector<osg::Vec3f> PhysicsSystem::getReleasedEquipmentState(const MWWorld::Ptr& item) const
    {
        const auto found = mObjects.find(item.mRef);
        return found == mObjects.end() ? std::vector<osg::Vec3f>() : mTaskScheduler->readReleasedObject(found->second);
    }

    // Derived from sunspotdark/morrowind-modern-physics
    // (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40), GPLv3.
    void PhysicsSystem::moveDynamicObjects()
    {
        // Raw bodies, render histories and their residual clock are read under
        // one scheduler lock; cell moves below may replace/remove references.
        const auto updates = mTaskScheduler->readDynamicObjectTransforms();
        auto world = MWBase::Environment::get().getWorld();
        for (const auto& update : updates)
            if (update.ptr.isInCell() && update.ptr.getRefData().getBaseNode())
                world->syncDynamicObject(update.ptr, Misc::Convert::toOsg(update.authoritative.getOrigin()),
                    Misc::Convert::toOsg(update.authoritative.getRotation()),
                    Misc::Convert::toOsg(update.rendered.getOrigin()),
                    Misc::Convert::toOsg(update.rendered.getRotation()), update.authoritativeChanged);
        mTaskScheduler->disableDynamicObjects(!Settings::game().mBrutalDynamicProps,
            !Settings::game().mBrutalEquipmentPhysics);
    }

    void PhysicsSystem::createNpcRagdoll(const MWWorld::Ptr& ptr,
        const std::map<std::string, osg::Matrixf, std::less<>>& bones, const osg::Vec3f& kick)
    {
        if (mRagdolls.contains(ptr.mRef))
            throw std::runtime_error("NPC already has a ragdoll");
        Actor* actor = getActor(ptr);
        if (!actor)
            throw std::runtime_error("NPC has no active physics actor");
        // Publish the new body at a batch boundary, so the previous frame cannot
        // step it before its initial velocity/point impulse has been captured.
        mTaskScheduler->waitForRagdollStep();
        // NPC death hands off to the ragdoll directly, with or without enhanced
        // drives. Keep the actor's translation continuous in both cases.
        const osg::Vec3f inherited = actor->getSimulationVelocity();
        const auto sample = mRagdollAnimation.find(ptr.mRef);
        const auto motion = Settings::game().mRagdollEnhancement && sample != mRagdollAnimation.end()
            ? sample->second.motion : Ragdoll::BoneMotion{};
        const auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr);
        Ragdoll::UnsupportedExtremityBones unsupportedBones;
        const auto surfaceVertices = animation ? animation->getRagdollExtremityVertices(&unsupportedBones)
                                              : Ragdoll::ExtremityVertices{};
        const auto* rig = findCreatureRig(ptr);
        auto ragdoll = rig
            ? std::make_unique<Ragdoll>(ptr, *rig, bones, inherited, kick, mTaskScheduler.get())
            : std::make_unique<Ragdoll>(
                ptr, bones, inherited, kick, mTaskScheduler.get(), std::nullopt, motion, surfaceVertices, unsupportedBones);
        mRagdollAnimation.erase(ptr.mRef);
        actor->setRagdoll(true);
        mRagdolls.emplace(ptr.mRef, std::move(ragdoll));
        MWBase::Environment::get().getWorld()->releaseNpcRagdollEquipment(ptr);
    }

    void PhysicsSystem::stopNpcRagdoll(const MWWorld::Ptr& ptr)
    {
        mSuperKickSources.erase(ptr.mRef);
        mSuperKickVictims.erase(ptr.mRef);
        const auto found = mRagdolls.find(ptr.mRef);
        if (found == mRagdolls.end())
            return;
        mPushRagdolls.erase(ptr.mRef);
        mRagdolls.erase(found); // remove constraints before bodies, while scheduler is alive
        if (auto* actor = getActor(ptr))
        {
            actor->setRagdoll(false);
            actor->updatePosition();
        }
        if (auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr))
            animation->endRagdollPose();
    }

    namespace
    {
        // Dark Messiah's kick reactions (ACT_PUSH_*): the side the kick came from, head or
        // body by where the kicker aims, and a random strength among small/medium/long.
        std::string chooseKickReaction(const MWWorld::Ptr& ptr, const MWWorld::Ptr& attacker,
            const osg::Vec3f& direction, float height)
        {
            const float yaw = ptr.getRefData().getPosition().rot[2];
            const osg::Vec3f forward(std::sin(yaw), std::cos(yaw), 0.f), right(std::cos(yaw), -std::sin(yaw), 0.f);
            const osg::Vec3f toAttacker = -direction; // the kick pushes away from its source
            const float front = toAttacker * forward, side = toAttacker * right;
            const char* from = std::abs(front) >= std::abs(side) ? (front >= 0.f ? "front" : "back")
                                                                  : (side >= 0.f ? "right" : "left");
            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
            const int roll = Misc::Rng::rollDice(100, prng);
            if (roll >= 85)
                return std::string("brutal_push_") + from + "_long";
            if (roll >= 60)
                return std::string("brutal_push_") + from + "_medium";
            bool head = false;
            if (!attacker.isEmpty())
            {
                // DM picks the kick height from the kicker's view pitch. The player's eye is at
                // a Morrowind NPC's head height, so a level view kicks the body; looking up
                // over the target's head lands a high kick.
                const auto& source = attacker.getRefData().getPosition();
                const osg::Vec3f target = ptr.getRefData().getPosition().asVec3();
                const float distance = osg::Vec2f(target.x() - source.pos[0], target.y() - source.pos[1]).length();
                const float eye = source.pos[2] + (attacker == MWMechanics::getPlayer() ? 124.7f : height * .9f);
                head = eye - std::tan(source.rot[0]) * distance > target.z() + std::max(height, 128.f) + 16.f;
            }
            return std::string("brutal_push_") + (head ? "head_" : "body_") + from;
        }
    }

    bool PhysicsSystem::applyNpcKick(const MWWorld::Ptr& ptr, const osg::Vec3f& kick, const MWWorld::Ptr& attacker, bool continuing,
        bool knockDown, bool slideTrip)
    {
        return applyNpcKickImpl(ptr, kick, attacker, continuing, knockDown, false, slideTrip);
    }

    bool PhysicsSystem::applyNpcKickImpl(const MWWorld::Ptr& ptr, const osg::Vec3f& kick,
        const MWWorld::Ptr& attacker, bool continuing, bool knockDown, bool superKick, bool slideTrip)
    {
        auto world=MWBase::Environment::get().getWorld();
        auto* actor=getActor(ptr);
        auto* animation=world->getAnimation(ptr);
        const bool isCreature = ptr.getType()==ESM::REC_CREA;
        if ((ptr.getType()!=ESM::REC_NPC_ && !isCreature) || ptr==MWMechanics::getPlayer() || !actor || !animation
            || ptr.getClass().getCreatureStats(ptr).isDead() || animation->isRagdollPose()) return false;
        const auto* rig = findCreatureRig(ptr);
        // A heavy creature answers more slowly; one with no legs, wings or fins cannot be moved at all.
        const auto* record = isCreature ? ptr.get<ESM::Creature>()->mBase : nullptr;
        const bool immobile = record && !(record->mFlags
            & (ESM::Creature::Walks | ESM::Creature::Flies | ESM::Creature::Swims | ESM::Creature::Bipedal));
        // A continuing push has already been scaled when it began.
        const auto* baseNode = ptr.getRefData().getBaseNode();
        const float weight = rig && !continuing ? rig->kickScale(baseNode ? baseNode->getScale().x() : 1.f) : 1.f;
        // The vertical component lifts only a ledge ragdoll: a standing recoil stays grounded.
        const osg::Vec3f push = immobile ? osg::Vec3f() : osg::Vec3f(kick.x(),kick.y(),0.f)*weight;
        osg::Vec3f direction=immobile ? osg::Vec3f(kick.x(),kick.y(),0.f) : push;
        if (direction.normalize()<.01f)
        {
            if (!superKick) return false;
            direction = osg::Vec3f(0, 1, 0); // A spent projectile can still knock a witnessed victim down.
        }
        const auto position=ptr.getRefData().getPosition().asVec3();
        const auto half=actor->getRenderingHalfExtents();
        // Source human hull is 72 units tall. Scale the authored 60/40 probes
        // to this world NPC, independently of the first-person camera/assets.
        const float units=2.f*half.z()/72.f;
        const int mask=CollisionType_World|CollisionType_HeightMap|CollisionType_Door;
        const osg::Vec3f chest=position+osg::Vec3f(0,0,half.z());
        bool fullKnockdown = superKick || (knockDown && !slideTrip);
        bool ledge=Settings::game().mBrutalRagdoll && (fullKnockdown || ((knockDown || actor->getOnGround() || continuing)
            && !world->isSwimming(ptr) && !world->isFlying(ptr)));
        if (isCreature && (!rig || (immobile && !superKick))) ledge=false;
        if (ledge && !knockDown && castSphere(chest,chest+direction*(60.f*units),half.x()*.8f,mask).mHit) ledge=false;
        const osg::Vec3f side(-direction.y(),direction.x(),0);
        // A knockdown falls on open ground; only a kick needs the void behind the target.
        if (ledge && !knockDown)
            for (float lateral : {-0.7f,0.f,0.7f})
            {
                const auto p=position+direction*(60.f*units)+side*(half.x()*lateral);
                const auto support=castRay(p+osg::Vec3f(0,0,8),p-osg::Vec3f(0,0,40.f*units),mask);
                if (support.mHit && support.mHitNormal.z()>.5f) { ledge=false; break; }
            }
        Ragdoll::BoneMatrices bones;
        if (ledge && rig)
        {
            for (const auto& part:rig->mParts)
            { auto bone=animation->getBoneWorldMatrix(part.mBone); if (!bone) {ledge=false;break;} bones.emplace(part.mBone,*bone); }
        }
        else if (ledge) for (const auto& name:Ragdoll::getRequiredBones())
        { auto bone=animation->getBoneWorldMatrix(name); if (!bone) {ledge=false;break;} bones.emplace(name,*bone); }
        if (!ledge)
        {
            if (superKick) return false; // Missing bones must not turn a rejected power kick into an ordinary push.
            if (!continuing)
            {
                if (!immobile) actor->addExternalVelocity(push);
                // Creatures keep their own hit animations: no group is imposed on them.
                MWBase::Environment::get().getMechanicsManager()->staggerActor(
                    ptr, isCreature ? std::string() : chooseKickReaction(ptr, attacker, direction, 2.f * half.z()));
                // Source rechecks support throughout a push, not only at contact.
                if (!isCreature || (rig && !immobile))
                    mKickWatches[ptr.mRef]={ptr,attacker,direction,0.f,kick.z()*weight/push.length()};
            }
            return false;
        }
        // A fresh kick has not entered external velocity. A continuing push
        // already has: transfer its remaining component into a torso impulse,
        // subtracting it from inherited COM motion so momentum is not doubled.
        mTaskScheduler->waitForRagdollStep();
        bool clear=false;
        const auto pending=actor->consumeExternalVelocity(clear);
        const auto inherited=(clear ? osg::Vec3f() : actor->getSimulationVelocity())+pending-(continuing ? push : osg::Vec3f());
        const bool lowTrip = slideTrip && knockDown && !superKick && !continuing;
        // The sliding boot removes support at the posed shins. Ordinary kicks
        // and continuing ledge pushes still enter through the chest.
        const osg::Vec3f rootBone = rig ? osg::Vec3f(bones.at(rig->mParts.front().mBone).getTrans()) : osg::Vec3f();
        const osg::Vec3f humanContact = rig ? osg::Vec3f() : osg::Vec3f(lowTrip
            ? (bones.at("Bip01 L Calf").getTrans() + bones.at("Bip01 L Foot").getTrans()
                  + bones.at("Bip01 R Calf").getTrans() + bones.at("Bip01 R Foot").getTrans()) * .25f
                - direction * 5.f // physical calf capsule radius
            : bones.at("Bip01 Spine1").getTrans() - direction * half.x());
        const osg::Vec3f contact = rig
            ? osg::Vec3f((lowTrip ? osg::Vec3f(position + osg::Vec3f(0.f, 0.f, half.z() * .3f)) : rootBone) - direction * half.x())
            : humanContact;
        MWBase::Environment::get().getMechanicsManager()->staggerActor(ptr);
        animation->beginRagdollPose();
        try {
            const auto sample = mRagdollAnimation.find(ptr.mRef);
            const auto motion = Settings::game().mRagdollEnhancement && sample != mRagdollAnimation.end()
                ? sample->second.motion : Ragdoll::BoneMotion{};
            Ragdoll::UnsupportedExtremityBones unsupportedBones;
            const auto surfaceVertices = animation->getRagdollExtremityVertices(&unsupportedBones);
            const auto profile = lowTrip ? Ragdoll::KickProfile::LowTrip : Ragdoll::KickProfile::Chest;
            auto body = rig
                ? std::make_unique<Ragdoll>(ptr,*rig,bones,inherited,kick*weight,mTaskScheduler.get(),contact,profile)
                : std::make_unique<Ragdoll>(ptr,bones,inherited,kick,mTaskScheduler.get(),contact,motion,
                    surfaceVertices,unsupportedBones,profile);
            if (fullKnockdown) body->guardSuperKickSpawn();
            mRagdollAnimation.erase(ptr.mRef);
            actor->clearExternalVelocity(); actor->setInertialForce({}); actor->setOnGround(false); actor->setRagdoll(true);
            mRagdolls.emplace(ptr.mRef,std::move(body));
            mKickWatches.erase(ptr.mRef);
            auto& state=mPushRagdolls[ptr.mRef]; state.peakZ=state.previousZ=position.z(); state.attacker=attacker;
            state.knockDown=knockDown;
            // Every living ragdoll is a disarm, including ordinary knockdowns.
            world->releaseNpcRagdollEquipment(ptr);
        } catch (...) {animation->endRagdollPose();throw;}
        if (!attacker.isEmpty() && !knockDown)
            MWBase::Environment::get().getLuaManager()->npcKickRagdoll(attacker, ptr);
        return true;
    }

    bool PhysicsSystem::applyNpcSuperKick(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity,
        const MWWorld::Ptr& attacker, std::uint64_t kickId)
    {
        if (!Settings::game().mBrutalRagdoll || ptr.isEmpty() || (ptr.getType() != ESM::REC_NPC_ && !findCreatureRig(ptr)) || ptr == MWMechanics::getPlayer()
            || !kickId || kickId > 9007199254740991ULL || mSuperKickIds.contains(kickId)
            || mSuperKickSources.contains(ptr.mRef)) return false;
        for (int axis = 0; axis < 3; ++axis) if (!std::isfinite(velocity[axis])) return false;
        if (velocity.length2() > 1000.f * 1000.f
            || osg::Vec2f(velocity.x(), velocity.y()).length() < 100.f) return false;
        auto* actor = getActor(ptr);
        auto world = MWBase::Environment::get().getWorld();
        auto* animation = world->getAnimation(ptr);
        if (!actor || !actor->isActive() || !animation || !ptr.getRefData().isEnabled()) return false;
        mTaskScheduler->waitForRagdollStep();
        if (!mRagdolls.contains(ptr.mRef))
        {
            if (!ptr.getClass().getCreatureStats(ptr).isDead())
            {
                if (!applyNpcKickImpl(ptr, velocity, attacker, false, true, true)) return false;
            }
            else
            {
                // Foot damage can kill immediately before the queued super-kick
                // action. Hand that current pose to physics without waiting for
                // an ordinary death-animation frame, then launch the same body.
                Ragdoll::BoneMatrices bones;
                std::vector<std::string> names;
                if (const auto* rig = findCreatureRig(ptr))
                    for (const auto& part : rig->mParts) names.push_back(part.mBone);
                else names = Ragdoll::getRequiredBones();
                for (const auto& name : names)
                {
                    const auto bone = animation->getBoneWorldMatrix(name);
                    if (!bone) return false;
                    bones.emplace(name, *bone);
                }
                animation->beginRagdollPose();
                try { createNpcRagdoll(ptr, bones, osg::Vec3f()); }
                catch (...) { animation->endRagdollPose(); throw; }
                mRagdolls.at(ptr.mRef)->guardSuperKickSpawn();
            }
        }
        auto& body = *mRagdolls.at(ptr.mRef);
        const auto* source = attacker.isEmpty() ? nullptr : getActor(attacker);
        if (!body.startSuperKick(kickId, kickId, velocity, source ? source->getCollisionObject() : nullptr)) return false;
        mSuperKickIds.insert(kickId);
        mSuperKickSources.emplace(ptr.mRef, SuperKickSource{ attacker, kickId });
        mSuperKickVictims.erase(ptr.mRef);
        if (auto push = mPushRagdolls.find(ptr.mRef); push != mPushRagdolls.end())
        {
            push->second = PushRagdoll{};
            push->second.attacker = attacker;
            push->second.peakZ = push->second.previousZ = body.getDynamics()[0].z();
            push->second.knockDown = true;
        }
        world->releaseNpcRagdollEquipment(ptr);
        refreshSuperKickActors();
        return true;
    }

    std::map<std::string, double> PhysicsSystem::getNpcSuperKickState(const MWWorld::Ptr& ptr) const
    {
        const auto body = mRagdolls.find(ptr.mRef);
        if (body == mRagdolls.end()) return {};
        auto result = body->second->getSuperKickState();
        if (const auto source = mSuperKickSources.find(ptr.mRef); source != mSuperKickSources.end())
        {
            result["secondary"] = 0;
            result["knockdowns"] = source->second.knockdowns;
            result["rejectedContacts"] = source->second.rejectedContacts;
        }
        else if (const auto victim = mSuperKickVictims.find(ptr.mRef); victim != mSuperKickVictims.end())
        {
            result["active"] = 0;
            result["secondary"] = 1;
            result["kickId"] = double(victim->second);
            result["receivedHits"] = 1;
        }
        return result;
    }

    void PhysicsSystem::refreshSuperKickActors()
    {
        // Retaining a corpse's diagnostics must not scan every active NPC for
        // the rest of the session. The next accepted kick refreshes candidates.
        if (!mTaskScheduler->hasActiveSuperKicks()) return;
        std::vector<PhysicsTaskScheduler::SuperKickActor> candidates;
        if (!mSuperKickSources.empty())
            for (const auto& [ref, actor] : mActors)
            {
                const auto ptr = actor->getPtr();
                if (!actor->isActive() || actor->isRagdoll() || ptr == MWMechanics::getPlayer()
                    || !ptr.getRefData().isEnabled()
                    || ptr.getClass().getCreatureStats(ptr).isDead()) continue;
                candidates.push_back({ actor->getCollisionObject(), actor->getSimulationVelocity(),
                    ptr.getCellRef().getRefNum() });
            }
        mTaskScheduler->setSuperKickActors(candidates);
    }

    void PhysicsSystem::processSuperKickContacts()
    {
        if (mSuperKickSources.empty()) return;
        const auto contacts = mTaskScheduler->takeSuperKickContacts();
        for (const auto& contact : contacts)
        {
            const auto source = std::find_if(mSuperKickSources.begin(), mSuperKickSources.end(),
                [&](const auto& item) { return item.second.kickId == contact.token; });
            if (source == mSuperKickSources.end()) continue;
            const auto attacker = source->second.attacker;
            MWWorld::Ptr target;
            for (const auto& [ref, actor] : mActors)
                if (actor->getCollisionObject() == contact.actor)
                { target = actor->getPtr(); break; }
            bool applied = false;
            if (!target.isEmpty() && target.getClass().isActor() && target != MWMechanics::getPlayer()
                && target.getRefData().isEnabled() && !target.getClass().getCreatureStats(target).isDead())
                if (const auto* actor = getActor(target); actor && actor->isActive() && !actor->isRagdoll())
                    applied = applyNpcKickImpl(target, contact.transfer, attacker, false, true, true);
            // World equipment release may move references. Resolve the source
            // again instead of holding iterators across a main-thread World call.
            const auto current = std::find_if(mSuperKickSources.begin(), mSuperKickSources.end(),
                [&](const auto& item) { return item.second.kickId == contact.token; });
            if (current != mSuperKickSources.end())
            {
                if (applied) ++current->second.knockdowns;
                else ++current->second.rejectedContacts;
            }
            if (applied) mSuperKickVictims[target.mRef] = contact.token;
        }
        refreshSuperKickActors();
    }

    bool PhysicsSystem::detachNpcRagdollHead(const MWWorld::Ptr& ptr, const osg::Vec3f& cutVelocity,
        const osg::Vec3f& headAngularImpulse)
    {
        if (ptr == MWMechanics::getPlayer() || !ptr.getClass().isNpc()
            || !ptr.getClass().getCreatureStats(ptr).isDead())
            return false;
        const auto found = mRagdolls.find(ptr.mRef);
        return found != mRagdolls.end() && found->second->detachHead(cutVelocity, headAngularImpulse);
    }

    bool PhysicsSystem::detachNpcRagdollTorso(const MWWorld::Ptr& ptr, const osg::Vec3f& cutVelocity,
        const osg::Vec3f& upperSeparation, const std::string& savedPose)
    {
        if (ptr == MWMechanics::getPlayer() || !ptr.getClass().isNpc()
            || !ptr.getClass().getCreatureStats(ptr).isDead()) return false;
        const auto found = mRagdolls.find(ptr.mRef);
        if (found == mRagdolls.end() || !found->second->detachTorso(cutVelocity, upperSeparation, savedPose)) return false;
        if (!savedPose.empty())
            if (auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr))
                for (const auto& [bone, matrix] : found->second->getBonePoses(1.f))
                    animation->setBoneWorldMatrix(bone, matrix);
        return true;
    }

    std::string PhysicsSystem::getNpcBisectionPoseData(const MWWorld::Ptr& ptr) const
    {
        const auto found = mRagdolls.find(ptr.mRef);
        return found == mRagdolls.end() ? std::string() : found->second->getBisectionPoseData();
    }

    bool PhysicsSystem::detachNpcRagdollLimbs(const MWWorld::Ptr& ptr, unsigned mask,
        const osg::Vec3f& cutVelocity, const std::string& savedPose)
    {
        if (ptr == MWMechanics::getPlayer() || !ptr.getClass().isNpc()
            || !ptr.getClass().getCreatureStats(ptr).isDead()) return false;
        const auto found = mRagdolls.find(ptr.mRef);
        if (found == mRagdolls.end() || !found->second->detachLimbs(mask, cutVelocity, savedPose)) return false;
        if (!savedPose.empty())
            if (auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr))
                for (const auto& [bone, matrix] : found->second->getBonePoses(1.f))
                    animation->setBoneWorldMatrix(bone, matrix);
        return true;
    }

    std::string PhysicsSystem::getNpcLimbSeveringPoseData(const MWWorld::Ptr& ptr) const
    {
        const auto found = mRagdolls.find(ptr.mRef);
        return found == mRagdolls.end() ? std::string() : found->second->getLimbSeveringPoseData();
    }

    unsigned PhysicsSystem::getNpcSeveredLimbsMask(const MWWorld::Ptr& ptr) const
    {
        const auto found = mRagdolls.find(ptr.mRef);
        return found == mRagdolls.end() ? 0 : found->second->getSeveredLimbsMask();
    }

    MWWorld::Ptr PhysicsSystem::kickNpcRagdoll(const osg::Vec3f& origin, const osg::Vec3f& forward, float reach,
        float minimumDot, const osg::Vec3f& velocity, bool& headHit, osg::Vec3f& hitPoint, std::size_t& hitPart)
    {
        headHit = false;
        Ragdoll* best = nullptr;
        std::size_t bestPart = 0;
        float bestDistance = -1.f;
        for (const auto& [ref, ragdoll] : mRagdolls)
        {
            std::size_t part = 0;
            osg::Vec3f point;
            const float distance = ragdoll->findKickPart(origin, forward, reach, minimumDot, part, point);
            if (distance >= 0.f && (!best || distance < bestDistance))
            {
                best = ragdoll.get();
                bestPart = part;
                bestDistance = distance;
                hitPoint = point;
            }
        }
        if (!best)
            return {};
        headHit = best->isDetachedHeadPart(bestPart);
        if (!best->applyKick(bestPart, velocity, hitPoint)) return {};
        hitPart = bestPart;
        return best->getPtr();
    }

    std::vector<osg::Vec3f> PhysicsSystem::getNpcRagdollDynamics(const MWWorld::Ptr& ptr) const
    {
        auto it=mRagdolls.find(ptr.mRef); if (it==mRagdolls.end()) return {};
        auto out=it->second->getDynamics();
        auto push=mPushRagdolls.find(ptr.mRef);
        out.emplace_back(push!=mPushRagdolls.end(),push==mPushRagdolls.end()?0.f:push->second.age,it->second->initialRotationError());
        out.emplace_back(it->second->initialMatrixError(),0,0);
        out.push_back(it->second->getLandingState());
        out.emplace_back(push==mPushRagdolls.end()?0.f:push->second.settled,
            push==mPushRagdolls.end()?0.f:push->second.unsupported,
            push!=mPushRagdolls.end() && push->second.supported);
        const auto head = it->second->getHeadState();
        out.insert(out.end(), head.begin(), head.end());
        return out;
    }

    std::map<std::string, double> PhysicsSystem::getNpcRagdollEnhancementState(const MWWorld::Ptr& ptr) const
    {
        const auto found = mRagdolls.find(ptr.mRef);
        return found == mRagdolls.end() ? std::map<std::string, double>() : found->second->getEnhancementState();
    }

    bool PhysicsSystem::applyNpcRagdollImpact(const MWWorld::Ptr& ptr, std::size_t part,
        const osg::Vec3f& point, const osg::Vec3f& impulse, std::uint64_t id)
    {
        const auto found = mRagdolls.find(ptr.mRef);
        return found != mRagdolls.end() && found->second->applyImpact(part, point, impulse, id);
    }

    void PhysicsSystem::sampleNpcRagdollAnimation(float duration)
    {
        if (!Settings::game().mRagdollEnhancement)
        { mRagdollAnimation.clear(); return; }
        if (!std::isfinite(duration) || duration <= 0.f)
        { mRagdollAnimation.clear(); return; }
        auto world = MWBase::Environment::get().getWorld();
        for (const auto& [ref, actor] : mActors)
        {
            const auto ptr = actor->getPtr();
            if (ptr.getType() != ESM::REC_NPC_ || ptr == MWMechanics::getPlayer()
                || actor->isRagdoll() || !actor->isActive())
            { mRagdollAnimation.erase(ref); continue; }
            if (ptr.getClass().getCreatureStats(ptr).isDead())
            {
                // Scripted death observation can queue activation on a later
                // frame. Retain the last live velocity briefly, without sampling
                // a canned death animation or reviving stale loaded-corpse motion.
                if (const auto found = mRagdollAnimation.find(ref); found != mRagdollAnimation.end())
                {
                    found->second.deadAge += duration;
                    if (found->second.deadAge > .25f) mRagdollAnimation.erase(found);
                }
                continue;
            }
            auto* animation = world->getAnimation(ptr);
            if (!animation) { mRagdollAnimation.erase(ref); continue; }
            Ragdoll::BoneMatrices bones;
            for (const auto& name : Ragdoll::getRequiredBones())
            {
                const auto bone = animation->getBoneWorldMatrix(name);
                if (!bone) { bones.clear(); break; }
                bones.emplace(name, *bone);
            }
            if (bones.empty()) { mRagdollAnimation.erase(ref); continue; }
            // Read the scene root through the same live world-path convention
            // as getBoneWorldMatrix, rather than mixing record/physics coordinates
            // with the evaluated rendered bones.
            const auto* root = ptr.getRefData().getBaseNode();
            if (!root) { mRagdollAnimation.erase(ref); continue; }
            const auto paths = root->getParentalNodePaths();
            if (paths.empty()) { mRagdollAnimation.erase(ref); continue; }
            const osg::Matrixf rootWorld = osg::computeLocalToWorld(paths.front());
            bool finiteRoot = true;
            for (int i = 0; i < 16; ++i)
                finiteRoot = finiteRoot && std::isfinite(rootWorld.ptr()[i]);
            if (!finiteRoot) { mRagdollAnimation.erase(ref); continue; }
            const auto renderedRootPosition = rootWorld.getTrans();
            auto& sample = mRagdollAnimation[ref];
            if (sample.deadAge > 0.f) sample.bones.clear(); // resurrection starts a fresh sample pair
            sample.deadAge = 0.f;
            sample.motion = sample.bones.empty() ? Ragdoll::BoneMotion{}
                : Ragdoll::sampleAnimationMotion(sample.bones, bones,
                    renderedRootPosition - sample.renderedRootPosition, duration);
            sample.bones = std::move(bones);
            sample.renderedRootPosition = renderedRootPosition;
        }
    }

    void PhysicsSystem::moveNpcRagdolls(float duration)
    {
        if (!Settings::game().mBrutalRagdoll)
        {
            if (!mRagdolls.empty()) mTaskScheduler->waitForRagdollStep();
            std::vector<MWWorld::Ptr> actors;
            for (const auto& [_, body] : mRagdolls) actors.push_back(body->getPtr());
            for (const auto& actor : actors)
            {
                stopNpcRagdoll(actor);
                MWBase::Environment::get().getMechanicsManager()->forceStateUpdate(actor);
            }
            mKickWatches.clear();
            return;
        }

        auto world = MWBase::Environment::get().getWorld();
        if (!Settings::game().mBrutalDismemberment)
        {
            std::vector<MWWorld::Ptr> cuts;
            for (const auto& [_, body] : mRagdolls)
                if (body->isTorsoDetached() || body->getSeveredLimbsMask()
                    || (!body->getHeadState().empty() && body->getHeadState().front().x() != 0.f))
                    cuts.push_back(body->getPtr());
            if (!cuts.empty()) mTaskScheduler->waitForRagdollStep();
            for (const auto& actor : cuts)
            {
                stopNpcRagdoll(actor);
                MWBase::Environment::get().getMechanicsManager()->forceStateUpdate(actor);
            }
        }
        std::vector<KickWatch> watches;
        for (auto it=mKickWatches.begin();it!=mKickWatches.end();)
        {
            auto& watch=it->second; watch.age+=duration;
            auto* actor=getActor(watch.actor);
            if (!actor || watch.age>1.f || actor->isExternalVelocityResetPending()
                || watch.actor.getClass().getCreatureStats(watch.actor).isDead()
                || actor->getExternalVelocity()*watch.direction<20.f)
                it=mKickWatches.erase(it);
            else { watches.push_back(watch); ++it; }
        }
        for (const auto& watch:watches)
            if (auto* actor=getActor(watch.actor))
            {
                auto remaining=watch.direction*std::max(0.f,actor->getExternalVelocity()*watch.direction);
                remaining.z()=remaining.length()*watch.lift;
                applyNpcKick(watch.actor,remaining,watch.attacker,true);
            }
        // Otherwise the same frame may select either the old completed pose
        // (alpha=1) or a newly completed batch (residual alpha), depending on
        // worker timing. Include bodies just created by a ledge watch, wait once
        // for the whole group, then select one deterministic residual clock.
        if (!mRagdolls.empty()) mTaskScheduler->waitForRagdollStep();
        processSuperKickContacts();
        struct Update { MWWorld::Ptr ptr; osg::Vec3f position;
            std::vector<std::pair<std::string, osg::Matrixf>> poses; };
        std::vector<Update> updates;
        std::vector<MWWorld::Ptr> cancel;
        std::vector<MWWorld::Ptr> recover;
        for (const auto& [_, ragdoll] : mRagdolls)
        {
            ragdoll->setWater(mWaterEnabled, mWaterHeight);
            ragdoll->configureEnhancement();
            const auto ptr = ragdoll->getPtr();
            auto* animation = world->getAnimation(ptr);
            if ((!ptr.getClass().getCreatureStats(ptr).isDead() && !mPushRagdolls.contains(ptr.mRef)) || !animation || !animation->isRagdollPose())
            {
                cancel.push_back(ptr);
                continue;
            }
            auto poses = ragdoll->getBonePoses();
            auto position = ragdoll->getActorPosition(ptr.getRefData().getPosition().asVec3());
            if (position && ptr.getClass().getCreatureStats(ptr).isDead())
            {
                // A corpse's standing pelvis offset places its reference below
                // the floor. That offset cannot be reused with the ordinary
                // completed death pose reconstructed by a saved-game load.
                // Reanchor only the scene reference; the same world bone poses
                // are reapplied below, leaving every rendered limb unchanged.
                position->z() = ragdoll->retainedBottom();
            }
            if (auto it=mPushRagdolls.find(ptr.mRef); it!=mPushRagdolls.end())
            {
                auto& push=it->second; push.age+=duration;
                auto& stats=ptr.getClass().getCreatureStats(ptr);
                const auto dynamics=ragdoll->getDynamics();
                const float z=dynamics[0].z();
                const int mask=CollisionType_World|CollisionType_HeightMap|CollisionType_Door;
                const auto floor=castRay(dynamics[0],osg::Vec3f(dynamics[0].x(),dynamics[0].y(),ragdoll->bottom()-12.f),mask);
                const bool supported=floor.mHit && floor.mHitNormal.z()>.5f && ragdoll->bottom()-floor.mHitPos.z()<12.f;
                push.supported=supported;
                if (supported && push.landed)
                {
                    // A later departure starts at its actual last supported
                    // height, not the peak of an earlier completed fall.
                    push.peakZ=z; push.downSpeed=0.f;
                }
                else
                {
                    push.peakZ=std::max(push.peakZ,z);
                    push.downSpeed=std::max(push.downSpeed,-dynamics[1].z());
                }
                push.unsupported=supported?0.f:push.unsupported+duration;
                // Rearm only after real unsupported flight. Rest contact jitter
                // cannot consume or repeatedly charge the same landing.
                if (push.unsupported>.05f) push.landed=false;
                if (!push.landed && push.downSpeed>80.f && dynamics[1].z()>-30.f && supported)
                {
                    push.landed=true;
                    const float height=std::max(0.f,push.peakZ-z);
                    const float damage=MWMechanics::calculateRagdollFallDamage(ptr,height);
                    ragdoll->recordLanding(height,damage);
                    if (damage>0.f && !stats.isDead()) ptr.getClass().onHit(ptr,{{"health",damage}},ESM::RefId(),push.attacker,true,MWMechanics::DamageSourceType::Unspecified);
                }
                if (position) { position->x()=dynamics[0].x();position->y()=dynamics[0].y();position->z()=ragdoll->bottom(); }
                if (supported && ragdoll->speed()<35.f && ragdoll->angularSpeed()<1.5f) push.settled+=duration;
                else push.settled=0.f;
                if (stats.isDead()) mPushRagdolls.erase(it); // retain the same body/momentum as a corpse
                // In water there is nothing to come to rest on: the actor swims again.
                else if (push.age>=2.f && (push.settled>=.35f || (mWaterEnabled && z<mWaterHeight))) recover.push_back(ptr);
            }
            if (position)
                updates.push_back({ptr, *position, std::move(poses)});
        }
        for (const auto& ptr : cancel)
            stopNpcRagdoll(ptr);
        for (const auto& update : updates)
        {
            // Cell moves can replace references or unload a corpse; never keep map iterators across World calls.
            const auto ptr = world->moveObject(update.ptr, update.position, false, false);
            if (!mRagdolls.contains(ptr.mRef))
                continue;
            if (auto* animation = world->getAnimation(ptr); animation && animation->isRagdollPose())
                for (const auto& [bone, matrix] : update.poses)
                    animation->setBoneWorldMatrix(bone, matrix);
        }
        for (const auto& ptr:recover)
        {
            auto* animation=world->getAnimation(ptr);
            std::string_view standUp;
            if (animation && Settings::game().mDarkMessiahStandUp && !Settings::game().mBrutalClassicNpcAnimations)
                if (const auto found=mRagdolls.find(ptr.mRef); found!=mRagdolls.end())
                {
                    // SCHED_MM_STAND_UP_FRONT/BACK. Both clips start lying with the head behind the
                    // actor's final facing, face down (front) or face up (back). Turn the actor to
                    // face the ragdoll's feet, then reapply the same world bone poses so the short
                    // recovery blend starts from exactly what is on screen.
                    // A copy: rotating the actor resamples the ragdoll's pose list.
                    const std::vector<std::pair<std::string, osg::Matrixf>> poses=found->second->getLastBonePoses();
                    const auto at=[&](std::string_view name) -> std::optional<osg::Vec3f> {
                        for (const auto& [bone,matrix]:poses) if (bone==name) return matrix.getTrans();
                        return std::nullopt; };
                    const auto pelvis=at("Bip01 Pelvis"), head=at("Bip01 Head");
                    const auto left=at("Bip01 L UpperArm"), right=at("Bip01 R UpperArm");
                    if (pelvis && head && left && right)
                    {
                        const osg::Vec3f up=*head-*pelvis;
                        osg::Vec3f feet(-up.x(),-up.y(),0.f);
                        const osg::Vec3f front=(*left-*right)^up;
                        const std::string_view group=front.z()<0.f ? "brutal_standup_front" : "brutal_standup_back";
                        if (feet.normalize()>1e-3f && animation->hasAnimation(group))
                        {
                            world->rotateObject(ptr,osg::Vec3f(0.f,0.f,std::atan2(feet.x(),feet.y())));
                            for (const auto& [bone,matrix]:poses) animation->setBoneWorldMatrix(bone,matrix);
                            standUp=group;
                        }
                    }
                }
            if (animation) animation->endRagdollPose(standUp.empty() ? .6f : .25f);
            stopNpcRagdoll(ptr);
            if (auto* actor=getActor(ptr)) {actor->setOnGround(false);actor->setInertialForce({});}
            MWBase::Environment::get().getMechanicsManager()->forceStateUpdate(ptr);
            if (!standUp.empty()) MWBase::Environment::get().getMechanicsManager()->staggerActor(ptr,standUp);
        }
    }

    std::vector<osg::Vec3f> PhysicsSystem::getNpcRagdollState(const MWWorld::Ptr& ptr) const
    {
        const auto found = mRagdolls.find(ptr.mRef);
        if (found == mRagdolls.end())
            return {};
        const auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr);
        if (!animation)
            return {};
        const auto& poses = found->second->getLastBonePoses();
        std::vector<osg::Vec3f> state;
        state.emplace_back(static_cast<float>(found->second->partCount()), found->second->maxJointError(),
            found->second->retainedBottom());
        for (const auto& [bone, matrix] : poses)
        {
            state.push_back(matrix.getTrans());
            const auto rendered = animation->getBoneWorldMatrix(bone);
            state.push_back(rendered ? osg::Vec3f(rendered->getTrans()) : osg::Vec3f());
        }
        return state;
    }

    void PhysicsSystem::moveActors()
    {
        auto* player = getActor(MWMechanics::getPlayer());
        const auto world = MWBase::Environment::get().getWorld();

        // copy new ptr position in temporary vector. player is handled separately as its movement might change active
        // cell.
        mActorsPositions.clear();
        if (!mActors.empty())
            mActorsPositions.reserve(mActors.size() - 1);
        for (const auto& [ptr, physicActor] : mActors)
        {
            if (physicActor.get() == player || physicActor->isRagdoll())
                continue;
            mActorsPositions.emplace_back(physicActor->getPtr(), physicActor->getSimulationPosition());
        }

        for (const auto& [ptr, pos] : mActorsPositions)
            world->moveObject(ptr, pos, false, false);

        if (player != nullptr)
            world->moveObject(player->getPtr(), player->getSimulationPosition(), false, false);
    }

    void PhysicsSystem::updateAnimatedCollisionShape(const MWWorld::Ptr& object)
    {
        ObjectMap::iterator found = mObjects.find(object.mRef);
        if (found != mObjects.end())
            if (found->second->animateCollisionShapes())
                mTaskScheduler->updateSingleAabb(found->second);
    }

    void PhysicsSystem::debugDraw()
    {
        if (mDebugDrawEnabled)
            mTaskScheduler->debugDraw();
    }

    bool PhysicsSystem::isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const
    {
        const auto physActor = mActors.find(actor.mRef);
        if (physActor != mActors.end())
            return physActor->second->getStandingOnPtr() == object;
        return false;
    }

    void PhysicsSystem::getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        for (const auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == object)
                out.emplace_back(actor->getPtr());
        }
    }

    bool PhysicsSystem::isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const
    {
        auto found = mObjects.find(object.mRef);
        if (found != mObjects.end())
            return found->second->collidedWith(type);
        return false;
    }

    void PhysicsSystem::getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        std::vector<MWWorld::Ptr> collisions = getCollisions(object, CollisionType_World, CollisionType_Actor);
        out.insert(out.end(), collisions.begin(), collisions.end());
    }

    void PhysicsSystem::disableWater()
    {
        if (mWaterEnabled)
        {
            mWaterEnabled = false;
            updateWater();
        }
    }

    void PhysicsSystem::enableWater(float height)
    {
        if (!mWaterEnabled || mWaterHeight != height)
        {
            mWaterEnabled = true;
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::setWaterHeight(float height)
    {
        if (mWaterHeight != height)
        {
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::updateWater()
    {
        if (mWaterCollisionObject)
        {
            mTaskScheduler->removeCollisionObject(mWaterCollisionObject.get());
        }

        if (!mWaterEnabled)
        {
            mWaterCollisionObject.reset();
            return;
        }

        mWaterCollisionObject = std::make_unique<btCollisionObject>();
        mWaterCollisionShape = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), mWaterHeight);
        mWaterCollisionObject->setCollisionShape(mWaterCollisionShape.get());
        mTaskScheduler->addCollisionObject(
            mWaterCollisionObject.get(), CollisionType_Water, CollisionType_Actor | CollisionType_Projectile);
    }

    bool PhysicsSystem::isAreaOccupiedByOtherActor(
        const MWWorld::LiveCellRefBase* actor, const osg::Vec3f& position, const float radius) const
    {
        const btCollisionObject* ignoredObject = nullptr;
        if (const auto it = mActors.find(actor); it != mActors.end())
            ignoredObject = it->second->getCollisionObject();
        const btVector3 bulletPosition = Misc::Convert::toBullet(position);
        const btVector3 aabbMin = bulletPosition - btVector3(radius, radius, radius);
        const btVector3 aabbMax = bulletPosition + btVector3(radius, radius, radius);
        const int mask = MWPhysics::CollisionType_Actor;
        const int group = MWPhysics::CollisionType_AnyPhysical;
        HasSphereCollisionCallback callback(bulletPosition, radius, mask, group, ignoredObject);
        mTaskScheduler->aabbTest(aabbMin, aabbMax, callback);
        return callback.getResult();
    }

    void PhysicsSystem::reportStats(unsigned int frameNumber, osg::Stats& stats) const
    {
        stats.setAttribute(frameNumber, "Physics Actors", static_cast<double>(mActors.size()));
        stats.setAttribute(frameNumber, "Physics Objects", static_cast<double>(mObjects.size()));
        stats.setAttribute(frameNumber, "Physics Projectiles", static_cast<double>(mProjectiles.size()));
        stats.setAttribute(frameNumber, "Physics HeightFields", static_cast<double>(mHeightFields.size()));
    }

    void PhysicsSystem::reportCollision(const btVector3& position, const btVector3& normal)
    {
        if (mDebugDrawEnabled)
            mDebugDrawer->addCollision(position, normal);
    }

    ActorFrameData::ActorFrameData(
        Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer)
        : mPosition()
        , mKineticMovement(isPlayer && (Settings::game().mKineticPlayerMovement
              || (Settings::game().mBrutalSlide && actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalSlideLow))
              && !actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalClinchActive)
        , mBrutalSliding(mKineticMovement && actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalSliding)
        , mBrutalSlideLow(isPlayer && actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalSlideLow)
        , mBrutalSlideFrictionBlend(mBrutalSliding ? 1.f
              : mKineticMovement && mBrutalSlideLow
                    && actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalSlidePhase == 3
              ? std::clamp(actor.getPtr().getClass().getCreatureStats(actor.getPtr()).mBrutalSlideBlend, 0.f, 1.f)
              : 0.f)
        , mStandingOn(nullptr)
        , mIsOnGround(actor.getOnGround())
        , mIsOnSlope(actor.getOnSlope())
        , mWalkingOnWater(false)
        , mInert(inert)
        , mCollisionObject(actor.getCollisionObject())
        , mSwimLevel(waterlevel
              - (actor.getRenderingHalfExtents().z() * 2
                  * MWBase::Environment::get()
                        .getESMStore()
                        ->get<ESM::GameSetting>()
                        .find("fSwimHeightScale")
                        ->mValue.getFloat()))
        , mSlowFall(slowFall)
        , mRotation()
        , mMovement(actor.velocity())
        , mWaterlevel(waterlevel)
        , mHalfExtentsZ(actor.getHalfExtents().z())
        , mOldHeight(0)
        , mStuckFrames(0)
        , mFlying(MWBase::Environment::get().getWorld()->isFlying(actor.getPtr()))
        , mWasOnGround(actor.getOnGround())
        , mIsAquatic(actor.getPtr().getClass().isPureWaterCreature(actor.getPtr()))
        , mWaterCollision(waterCollision)
        , mSkipCollisionDetection(!actor.getCollisionMode())
        , mIsPlayer(isPlayer)
    {
        if (mBrutalSliding)
            mMovement.z() = 0.f;
    }

    ProjectileFrameData::ProjectileFrameData(Projectile& projectile)
        : mPosition(projectile.getPosition())
        , mMovement(projectile.velocity())
        , mCaster(projectile.getCasterCollisionObject())
        , mCollisionObject(projectile.getCollisionObject())
        , mProjectile(&projectile)
    {
    }

    WorldFrameData::WorldFrameData()
        : mIsInStorm(MWBase::Environment::get().getWorld()->isInStorm())
        , mStormDirection(MWBase::Environment::get().getWorld()->getStormDirection())
    {
    }

    LOSRequest::LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2)
        : mResult(false)
        , mStale(false)
        , mAge(0)
    {
        // we use raw actor pointer pair to uniquely identify request
        // sort the pointer value in ascending order to not duplicate equivalent requests, eg. getLOS(A, B) and
        // getLOS(B, A)
        auto* raw1 = a1.lock().get();
        auto* raw2 = a2.lock().get();
        assert(raw1 != raw2);
        if (raw1 < raw2)
        {
            mActors = { a1, a2 };
            mRawActors = { raw1, raw2 };
        }
        else
        {
            mActors = { a2, a1 };
            mRawActors = { raw2, raw1 };
        }
    }

    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept
    {
        return lhs.mRawActors == rhs.mRawActors;
    }
}
