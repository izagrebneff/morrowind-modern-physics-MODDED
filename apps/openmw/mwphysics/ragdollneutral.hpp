#ifndef OPENMW_MWPHYSICS_RAGDOLLNEUTRAL_H
#define OPENMW_MWPHYSICS_RAGDOLLNEUTRAL_H
#include <array>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

namespace MWPhysics
{
    // Authored editable standard-humanoid profile. Actor axes R=+X,F=+Y,U=+Z;
    // visual long-bone +X follows its segment. Not captured from a death pose.
    struct RagdollNeutralProfile
    {
        std::array<btQuaternion, 11> neutral;
        std::array<btQuaternion, 11> limitCenter;
        RagdollNeutralProfile()
        {
            const auto rotation = [](const btMatrix3x3& basis) { btQuaternion q; basis.getRotation(q); return q; };
            const auto rx = [](btScalar angle) { return btQuaternion(btVector3(1, 0, 0), btRadians(angle)); };
            const auto ry = [](btScalar angle) { return btQuaternion(btVector3(0, 1, 0), btRadians(angle)); };
            const auto upper = rotation(btMatrix3x3(0, 0, -1, 0, 1, 0, 1, 0, 0));
            const auto arm = rotation(btMatrix3x3(0, 0, -1, 0, -1, 0, -1, 0, 0));
            const auto leg = rotation(btMatrix3x3(0, 0, 1, 0, 1, 0, -1, 0, 0));
            neutral[0] = upper;
            neutral[1] = rx(-5) * upper;
            neutral[2] = rx(-20) * upper;
            for (int side = 0; side < 2; ++side)
            {
                const btScalar sign = side == 0 ? 1 : -1;
                neutral[3 + side * 2] = rx(20) * ry(sign * 10) * arm;
                neutral[4 + side * 2] = rx(20) * ry(sign * 10) * rx(55) * arm;
                neutral[7 + side * 2] = rx(30) * ry(sign * 5) * leg;
                neutral[8 + side * 2] = rx(30) * ry(sign * 5) * rx(-45) * leg;
            }
            limitCenter = neutral;
            for (int side = 0; side < 2; ++side)
            {
                const btScalar sign = side == 0 ? 1 : -1;
                limitCenter[3 + side * 2] = rx(20) * ry(sign * 45) * arm;
                limitCenter[4 + side * 2] = rx(20) * ry(sign * 10) * rx(70) * arm;
                limitCenter[8 + side * 2] = rx(30) * ry(sign * 5) * rx(-70) * leg;
            }
        }
    };
}
#endif
