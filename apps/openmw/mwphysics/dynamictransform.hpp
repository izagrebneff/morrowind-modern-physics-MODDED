#ifndef OPENMW_MWPHYSICS_DYNAMICTRANSFORM_H
#define OPENMW_MWPHYSICS_DYNAMICTRANSFORM_H

#include <LinearMath/btTransform.h>

#include <algorithm>
#include <cmath>

namespace MWPhysics
{
    // Render-only history of completed outer physics steps. Reading it never
    // changes the authoritative body, velocities, collision shape or save pose.
    class DynamicTransformHistory
    {
    public:
        explicit DynamicTransformHistory(const btTransform& transform)
            : mPrevious(transform), mCurrent(transform) {}

        void reset(const btTransform& transform) { mPrevious = mCurrent = transform; }
        void advance(const btTransform& transform) { mPrevious = mCurrent; mCurrent = transform; }
        const btTransform& current() const { return mCurrent; }

        btTransform rendered(btScalar alpha) const
        {
            if (!std::isfinite(alpha))
                return mCurrent;
            alpha = std::clamp(alpha, btScalar(0), btScalar(1));
            const auto from = mPrevious.getRotation();
            auto to = mCurrent.getRotation();
            if (from.dot(to) < 0)
                to = btQuaternion(-to.x(), -to.y(), -to.z(), -to.w());
            return btTransform(from.slerp(to, alpha), mPrevious.getOrigin().lerp(mCurrent.getOrigin(), alpha));
        }

    private:
        btTransform mPrevious, mCurrent;
    };

    // Interpolate the center of mass first: a rotating offset root follows an
    // arc around it, rather than a straight line between two offset endpoints.
    inline btTransform dynamicItemRoot(btTransform centerOfMass, const btVector3& localCenter, btScalar scaleRatio)
    {
        centerOfMass.setOrigin(centerOfMass * (-localCenter * scaleRatio));
        return centerOfMass;
    }
}

#endif
