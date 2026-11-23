#ifndef OPENMW_MWPHYSICS_SUPERKICK_H
#define OPENMW_MWPHYSICS_SUPERKICK_H

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/NarrowPhaseCollision/btManifoldPoint.h>

#include "collisiontype.hpp"

namespace MWPhysics::SuperKick
{
    constexpr btScalar Lifetime = 10;
    constexpr btScalar MinimumContactSpeed = 80;
    constexpr btScalar MomentumShare = btScalar(.22);
    constexpr btScalar MaximumTransferSpeed = 250;
    constexpr int WorldMask = CollisionType_World | CollisionType_Door | CollisionType_HeightMap;

    inline bool finite(const btVector3& v)
    {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    }

    struct Witness
    {
        const btCollisionObject* actor = nullptr;
        btVector3 point{ 0, 0, 0 }, normal{ 0, 0, 0 };
        btScalar fraction = 1;
        unsigned part = 0;
        bool overlap = false;
    };

    // The real limb shape is queried against the real character shape. This
    // helper never adds a proxy or changes either object's collision filter.
    // Call only with the dynamics world locked, after each Bullet substep.
    struct Trace final : btCollisionWorld::ConvexResultCallback,
                         btCollisionWorld::ContactResultCallback
    {
        const std::unordered_set<const btCollisionObject*>& actors;
        const btCollisionObject* limb;
        btVector3 displacement;
        std::vector<Witness> hits;
        btScalar worldFraction = 1;
        unsigned blocked = 0;

        Trace(const std::unordered_set<const btCollisionObject*>& targets,
            const btCollisionObject* body, const btVector3& movement)
            : actors(targets), limb(body), displacement(movement)
        {
        }

        bool needsCollision(btBroadphaseProxy* proxy) const override
        {
            const auto* object = static_cast<const btCollisionObject*>(proxy->m_clientObject);
            if (object == limb || !object->hasContactResponse()) return false;
            if (proxy->m_collisionFilterGroup & CollisionType_Actor)
                return (proxy->m_collisionFilterMask & CollisionType_Actor) && actors.contains(object);
            return (proxy->m_collisionFilterGroup & WorldMask)
                && (proxy->m_collisionFilterMask & CollisionType_Dynamic);
        }

        void add(const btCollisionObject* actor, const btVector3& point, const btVector3& normal,
            btScalar fraction, bool overlap)
        {
            if (!finite(point) || !finite(normal) || normal.length2() < btScalar(.5)) return;
            const Witness witness{ actor, point, normal.normalized(), fraction, 0, overlap };
            const auto found = std::find_if(hits.begin(), hits.end(),
                [actor](const Witness& hit) { return hit.actor == actor; });
            if (found == hits.end()) hits.push_back(witness);
            else if (fraction < found->fraction) *found = witness;
        }

        btScalar addSingleResult(btCollisionWorld::LocalConvexResult& result, bool normalInWorld) override
        {
            const btVector3 normal = normalInWorld ? result.m_hitNormalLocal
                : result.m_hitCollisionObject->getWorldTransform().getBasis() * result.m_hitNormalLocal;
            if (actors.contains(result.m_hitCollisionObject))
                add(result.m_hitCollisionObject, result.m_hitPointLocal, normal, result.m_hitFraction, false);
            // An initial supporting contact does not block a limb lifting away
            // from the floor. A wall approached by this limb does clip its path.
            else if (displacement.dot(normal) < btScalar(-1e-5))
                worldFraction = std::min(worldFraction, result.m_hitFraction);
            return 1; // Collect every candidate, then clip against the nearest wall.
        }

        btScalar addSingleResult(btManifoldPoint& contact, const btCollisionObjectWrapper* a, int, int,
            const btCollisionObjectWrapper* b, int, int) override
        {
            if (contact.getDistance() > 0) return 0;
            const bool first = a->getCollisionObject() == limb;
            const auto* other = (first ? b : a)->getCollisionObject();
            if (actors.contains(other))
                add(other, first ? contact.getPositionWorldOnB() : contact.getPositionWorldOnA(),
                    first ? contact.m_normalWorldOnB : -contact.m_normalWorldOnB, 1, true);
            return 0;
        }
    };

    inline Trace trace(btCollisionWorld& world, btCollisionObject& limb, const btTransform& from,
        const btTransform& to, const std::unordered_set<const btCollisionObject*>& actors)
    {
        Trace result(actors, &limb, to.getOrigin() - from.getOrigin());
        if (!limb.getCollisionShape()->isConvex()) return result;
        world.convexSweepTest(static_cast<const btConvexShape*>(limb.getCollisionShape()), from, to, result);
        // Initial penetration and a grazing end contact are not always reported
        // by a convex cast. An exact narrow-phase endpoint test covers both.
        world.contactTest(&limb, result);
        result.hits.erase(std::remove_if(result.hits.begin(), result.hits.end(), [&](const Witness& hit) {
            if (hit.fraction > result.worldFraction + btScalar(1e-4))
            { ++result.blocked; return true; }
            struct Occlusion final : btCollisionWorld::ClosestRayResultCallback
            {
                using btCollisionWorld::ClosestRayResultCallback::ClosestRayResultCallback;
                bool needsCollision(btBroadphaseProxy* proxy) const override
                {
                    const auto* object = static_cast<const btCollisionObject*>(proxy->m_clientObject);
                    return object->hasContactResponse() && (proxy->m_collisionFilterGroup & WorldMask)
                        && (proxy->m_collisionFilterMask & CollisionType_Dynamic);
                }
            } obstruction(from.getOrigin(), hit.point);
            world.rayTest(from.getOrigin(), hit.point, obstruction);
            if (obstruction.hasHit() && obstruction.m_closestHitFraction < btScalar(.999))
            { ++result.blocked; return true; }
            return false;
        }), result.hits.end());
        return result;
    }

    inline btVector3 transfer(const btVector3& velocity)
    {
        btVector3 result = velocity * MomentumShare;
        const btScalar speed = result.length();
        if (speed > MaximumTransferSpeed) result *= MaximumTransferSpeed / speed;
        return result;
    }
}

#endif
