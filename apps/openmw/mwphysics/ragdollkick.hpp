#ifndef OPENMW_MWPHYSICS_RAGDOLLKICK_H
#define OPENMW_MWPHYSICS_RAGDOLLKICK_H

#include "ragdollenhancement.hpp"
#include <optional>
#include <vector>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace MWPhysics::RagdollKick
{
    // One contact transaction, never a per-step force. The component receives
    // exactly M*deltaVelocity. A small point impulse supplies local articulation;
    // subtracting its share from the common impulse keeps total momentum exact.
    struct Plan
    {
        std::vector<btRigidBody*> bodies;
        btRigidBody* hit = nullptr;
        btVector3 commonVelocity{ 0, 0, 0 }, pointImpulse{ 0, 0, 0 }, arm{ 0, 0, 0 };
        btVector3 totalImpulse{ 0, 0, 0 }, pointAngularImpulse{ 0, 0, 0 };
        btScalar mass = 0;

        void apply() const
        {
            for (auto* body : bodies)
            {
                body->activate(true);
                body->applyCentralImpulse(commonVelocity / body->getInvMass());
            }
            hit->applyImpulse(pointImpulse, arm);
        }
    };

    inline std::optional<Plan> prepare(const std::vector<btRigidBody*>& bodies,
        btRigidBody* hit, const btVector3& point, const btVector3& deltaVelocity)
    {
        using RagdollEnhancement::finite;
        if (!hit || bodies.empty() || !finite(point) || !finite(deltaVelocity)
            || !finite(deltaVelocity.length2()) || deltaVelocity.length2() > btScalar(2000) * 2000)
            return {};
        Plan plan;
        plan.bodies = bodies;
        plan.hit = hit;
        bool containsHit = false;
        for (auto* body : bodies)
        {
            if (!body || !std::isfinite(body->getInvMass()) || body->getInvMass() <= 0
                || body->getLinearFactor() != btVector3(1, 1, 1)) return {};
            containsHit |= body == hit;
            plan.mass += 1 / body->getInvMass();
        }
        if (!containsHit || !std::isfinite(plan.mass) || plan.mass <= 0) return {};
        plan.totalImpulse = deltaVelocity * plan.mass;
        if (!finite(plan.totalImpulse) || !finite(plan.totalImpulse.length2())
            || plan.totalImpulse.length2() > btScalar(200000) * 200000) return {};
        const btScalar speed = deltaVelocity.length();
        plan.pointImpulse = speed > btScalar(1e-6)
            ? deltaVelocity * (btMin(btScalar(.15), btScalar(80) / speed) / hit->getInvMass())
            : btVector3(0, 0, 0);
        plan.arm = point - hit->getCenterOfMassPosition();
        plan.pointAngularImpulse = plan.arm.cross(plan.pointImpulse);
        btVector3 angular = (hit->getInvInertiaTensorWorld() * plan.pointAngularImpulse) * hit->getAngularFactor();
        if (!finite(plan.arm) || !finite(plan.pointAngularImpulse) || !finite(angular)
            || !finite(angular.length2())) return {};
        // Bound the contact's own angular increment, independent of frame rate,
        // slow motion, body mass and whichever extremity the boot reaches first.
        if (angular.length2() > 9)
        {
            const btScalar scale = 3 / angular.length();
            plan.pointImpulse *= scale;
            plan.pointAngularImpulse *= scale;
            angular *= scale;
        }
        plan.commonVelocity = deltaVelocity - plan.pointImpulse / plan.mass;
        for (auto* body : bodies)
        {
            const btVector3 linear = body->getLinearVelocity() + plan.commonVelocity
                + (body == hit ? plan.pointImpulse * body->getInvMass() : btVector3(0, 0, 0));
            const btVector3 nextAngular = body->getAngularVelocity()
                + (body == hit ? angular : btVector3(0, 0, 0));
            if (!finite(linear) || !finite(linear.length2()) || !finite(nextAngular)
                || !finite(nextAngular.length2())) return {};
        }
        return plan;
    }
}
#endif
