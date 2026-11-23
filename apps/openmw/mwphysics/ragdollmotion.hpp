#ifndef OPENMW_MWPHYSICS_RAGDOLLMOTION_H
#define OPENMW_MWPHYSICS_RAGDOLLMOTION_H

#include "ragdollenhancement.hpp"

namespace MWPhysics::RagdollEnhancement
{
    struct AnimationMotion
    {
        btVector3 linear{0, 0, 0}; // animation relative to actor translation
        btVector3 angular{0, 0, 0}; // world angular velocity
    };

    // Sample actual animated motion, then add the current physics actor velocity
    // at activation. Subtracting actor translation avoids inheriting it twice.
    inline AnimationMotion animationMotion(const btVector3& previousCenter, const btVector3& center,
        const btQuaternion& previousRotation, const btQuaternion& rotation,
        const btVector3& actorTranslation, btScalar dt)
    {
        if (!finite(previousCenter) || !finite(center) || !finite(actorTranslation)
            || !validRotation(previousRotation) || !validRotation(rotation)
            || !std::isfinite(dt) || dt < btScalar(1e-4) || dt > btScalar(.1)
            || actorTranslation.length2() > btScalar(128 * 128))
            return {};
        const btVector3 displacement = center - previousCenter - actorTranslation;
        // Scene relocation or a replaced skeleton is not animation velocity.
        if (displacement.length2() > btScalar(128 * 128))
            return {};
        return {clampMagnitude(displacement / dt, 600),
            clampMagnitude(rotationVector(rotation * previousRotation.inverse()) / dt, 20)};
    }
}
#endif
