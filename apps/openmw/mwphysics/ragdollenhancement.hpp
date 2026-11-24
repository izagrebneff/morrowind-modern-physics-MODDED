#ifndef OPENMW_MWPHYSICS_RAGDOLLENHANCEMENT_H
#define OPENMW_MWPHYSICS_RAGDOLLENHANCEMENT_H

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace MWPhysics::RagdollEnhancement
{
    // All dimensional parameters use Bullet's existing game-length/mass/second units.
    // These are moments, not impulses: applyTorque() must not receive another dt factor.
    struct SoftStopSettings
    {
        btScalar widthRad = btScalar(0.2);
        // Cap on dU/d(radial angle); SO(3) geometry can make the world moment
        // larger. Clamping that vector would destroy the potential gradient.
        btScalar springAtLimit = 0;
        btScalar damping = 0;
        btScalar maxDamping = 0;
        bool approachOnly = false;
    };

    struct ToneSettings
    {
        btScalar stiffness = 0;
        btScalar damping = 0;
        btScalar maxSpring = 0;
        btScalar maxDamping = 0;
    };

    inline bool finite(btScalar value) { return std::isfinite(value); }
    inline bool finite(const btVector3& value)
    {
        return finite(value.x()) && finite(value.y()) && finite(value.z());
    }
    inline bool finite(const btQuaternion& value)
    {
        return finite(value.x()) && finite(value.y()) && finite(value.z()) && finite(value.w());
    }

    inline bool validRotation(const btQuaternion& value)
    {
        return finite(value) && finite(value.length2()) && value.length2() > btScalar(1e-12);
    }

    inline bool valid(const SoftStopSettings& settings)
    {
        return finite(settings.widthRad) && settings.widthRad >= 0 && finite(settings.springAtLimit)
            && settings.springAtLimit >= 0 && finite(settings.damping) && settings.damping >= 0
            && finite(settings.maxDamping) && settings.maxDamping >= 0;
    }
    inline bool valid(const ToneSettings& settings)
    {
        return finite(settings.stiffness) && settings.stiffness >= 0 && finite(settings.damping)
            && settings.damping >= 0 && finite(settings.maxSpring) && settings.maxSpring >= 0
            && finite(settings.maxDamping) && settings.maxDamping >= 0;
    }

    inline btVector3 clampMagnitude(const btVector3& value, btScalar maximum)
    {
        if (!finite(value) || !finite(maximum) || maximum <= 0)
            return btVector3(0, 0, 0);
        // Scale first so finite but unusually large input cannot overflow length2().
        const btScalar largest = std::max({ btFabs(value.x()), btFabs(value.y()), btFabs(value.z()) });
        if (largest <= 0)
            return btVector3(0, 0, 0);
        const btVector3 scaled = value / largest;
        const btScalar scaledLength = scaled.length();
        if (largest <= maximum / scaledLength)
            return value;
        return scaled * (maximum / scaledLength);
    }

    inline btQuaternion canonicalRotation(btQuaternion value)
    {
        value.normalize();
        bool negate = value.w() < 0;
        // Resolve the exact half-turn q/-q ambiguity identically, without normalizing xyz.
        if (value.w() == 0)
        {
            int axis = 0;
            for (int i = 1; i < 3; ++i)
                if (btFabs(value[i]) > btFabs(value[axis]))
                    axis = i;
            negate = value[axis] < 0;
        }
        return negate ? -value : value;
    }

    inline btVector3 rotationVector(const btQuaternion& rotation)
    {
        if (!validRotation(rotation))
            return btVector3(0, 0, 0);
        const btQuaternion q = canonicalRotation(rotation);
        const btVector3 vector(q.x(), q.y(), q.z());
        const btScalar sine = vector.length();
        if (sine < btScalar(1e-6))
            return vector * 2;
        return vector * (2 * btAtan2(sine, q.w()) / sine);
    }

    struct TorqueComponents
    {
        btVector3 spring = btVector3(0, 0, 0);
        btVector3 damping = btVector3(0, 0, 0);
        btVector3 total() const { return spring + damping; }
    };

    struct SoftWidth
    {
        double value;
        double derivativeLimit;
    };

    // C1 soft minimum of requested width and .45*limit. The narrow transition
    // keeps opposite zones disjoint, is exact outside +/-10% of requestedWidth,
    // and is at most 2.5% narrower at the former min() derivative kink.
    inline SoftWidth softWidth(double requestedWidth, double limit)
    {
        if (!std::isfinite(requestedWidth) || !std::isfinite(limit) || requestedWidth <= 0 || limit <= 0)
            return { 0, 0 };
        const double boundedWidth = .45 * limit;
        const double transition = .1 * requestedWidth;
        const double difference = boundedWidth - requestedWidth;
        if (difference <= -transition) return { boundedWidth, .45 };
        if (difference >= transition) return { requestedWidth, 0 };
        return { (requestedWidth + boundedWidth) / 2 - difference * difference / (4 * transition) - transition / 4,
            .45 * (.5 - difference / (2 * transition)) };
    }

    // Bound an explicitly integrated damping pair using the actual world
    // inertias and substep. Damping that is dissipative in continuous time can
    // add energy if a high coefficient reverses relative motion within one step.
    // A quarter of the local work budget also covers this profile's maximum
    // graph degree of four: summing all pairs cannot add free-body angular energy.
    // Only damping is scaled; keep the spring components separate at the caller.
    inline btVector3 boundedDampingTorqueWorld(const btVector3& damping, const btVector3& relativeAngularVelocity,
        const btMatrix3x3& inverseInertiaSumWorld, btScalar dt, btScalar* scaleOut = nullptr)
    {
        const btVector3 response = inverseInertiaSumWorld * damping;
        const auto dotDouble = [](const btVector3& a, const btVector3& b) {
            return double(a.x()) * b.x() + double(a.y()) * b.y() + double(a.z()) * b.z();
        };
        const double work = dotDouble(damping, relativeAngularVelocity);
        const double quadratic = dotDouble(damping, response);
        btScalar scale = 0;
        if (finite(damping) && finite(relativeAngularVelocity) && finite(response) && finite(dt) && dt > 0
            && std::isfinite(work) && std::isfinite(quadratic))
        {
            if (damping == btVector3(0, 0, 0)) scale = 1;
            else if (work < 0 && quadratic > 0)
                scale = btScalar(std::min(1., -work / (4. * double(dt) * quadratic)));
        }
        if (scaleOut) *scaleOut = scale;
        return scale > 0 ? damping * scale : btVector3(0, 0, 0);
    }

    inline TorqueComponents softStop(btScalar penetration, btScalar width, btScalar outwardSpeed,
        const btVector3& outwardAxisWorld, const SoftStopSettings& settings)
    {
        TorqueComponents result;
        if (!valid(settings) || !finite(penetration) || !finite(width) || width <= 0
            || !finite(outwardSpeed) || !finite(outwardAxisWorld) || penetration <= 0)
            return result;
        const btScalar x = std::clamp(penetration / width, btScalar(0), btScalar(1));
        const btScalar activation = x * x * (3 - 2 * x);
        result.spring = -outwardAxisWorld * (settings.springAtLimit * x * x);
        const btScalar speed = settings.approachOnly ? btMax(btScalar(0), outwardSpeed) : outwardSpeed;
        const btScalar damping = std::clamp(settings.damping * activation * speed,
            -settings.maxDamping, settings.maxDamping);
        result.damping = -outwardAxisWorld * damping;
        return result;
    }

    // The only extension of Bullet's joint: expose its existing cone/twist geometry
    // before the hard limit is active. The stock constraint remains the hard stop.
    // All returned world moments act ON CHILD B; apply their negative to parent A.
    class ConeTwist final : public btConeTwistConstraint
    {
    public:
        using btConeTwistConstraint::btConeTwistConstraint;

        btQuaternion relativeRotation() const
        {
            const btQuaternion a = getRigidBodyA().getWorldTransform().getRotation() * getAFrame().getRotation();
            const btQuaternion b = getRigidBodyB().getWorldTransform().getRotation() * getBFrame().getRotation();
            return a.inverse() * b;
        }

        btVector3 allowedTorqueWorld(const btVector3& torque) const
        {
            if (!finite(torque))
                return btVector3(0, 0, 0);
            const btQuaternion a = getRigidBodyA().getWorldTransform().getRotation() * getAFrame().getRotation();
            if (!validRotation(a))
                return btVector3(0, 0, 0);
            btVector3 local = quatRotate(a.inverse(), torque);
            if (getTwistSpan() == 0)
                local.setX(0);
            // Bullet treats swing spans below fixThresh as locked hinge coordinates.
            if (getSwingSpan2() < getFixThreshValue())
                local.setY(0);
            if (getSwingSpan1() < getFixThreshValue())
                local.setZ(0);
            const btVector3 result = quatRotate(a, local);
            return finite(result) ? result : btVector3(0, 0, 0);
        }

        btVector3 neutralTorqueWorld(const btQuaternion& neutralInParentJointFrame,
            const ToneSettings& settings, TorqueComponents* components = nullptr) const
        {
            TorqueComponents result;
            const btQuaternion relative = relativeRotation();
            const btVector3 relativeOmega = getRigidBodyB().getAngularVelocity() - getRigidBodyA().getAngularVelocity();
            if (valid(settings) && validRotation(neutralInParentJointFrame) && validRotation(relative)
                && finite(relativeOmega))
            {
                const btQuaternion a = getRigidBodyA().getWorldTransform().getRotation() * getAFrame().getRotation();
                const btVector3 error = rotationVector(neutralInParentJointFrame.normalized() * relative.inverse());
                result.spring = allowedTorqueWorld(quatRotate(a, clampMagnitude(error * settings.stiffness,
                    settings.maxSpring)));
                result.damping = allowedTorqueWorld(clampMagnitude(-relativeOmega * settings.damping,
                    settings.maxDamping));
            }
            if (components)
                *components = result;
            return result.total();
        }

        btVector3 softStopTorqueWorld(const SoftStopSettings& settings, TorqueComponents* components = nullptr)
        {
            TorqueComponents result;
            const btQuaternion a = getRigidBodyA().getWorldTransform().getRotation() * getAFrame().getRotation();
            const btQuaternion b = getRigidBodyB().getWorldTransform().getRotation() * getBFrame().getRotation();
            const btVector3 omega = getRigidBodyB().getAngularVelocity() - getRigidBodyA().getAngularVelocity();
            if (!valid(settings) || settings.widthRad <= 0 || !validRotation(a) || !validRotation(b) || !finite(omega))
            {
                if (components)
                    *components = result;
                return result.total();
            }
            btQuaternion cone, twist;
            split(b.inverse() * a, cone, twist);
            const auto add = [&](btScalar angle, btScalar limit, const btVector3& outwardAxis) {
                if (!finite(angle) || !finite(limit) || limit <= 0 || limit >= SIMD_PI
                    || !finite(outwardAxis) || outwardAxis.length2() < btScalar(0.5))
                    return;
                // Keep the opposite zones disjoint even for the narrowest valid span.
                const btScalar width = btScalar(softWidth(settings.widthRad, limit).value);
                const TorqueComponents stop = softStop(width + angle - limit, width, outwardAxis.dot(omega),
                    outwardAxis, settings);
                result.damping += stop.damping;
            };
            if (getSwingSpan1() >= getFixThreshValue() && getSwingSpan2() >= getFixThreshValue())
            {
                btScalar angle = 0, limit = 0;
                btVector3 axis(0, 0, 0);
                computeConeLimitInfo(cone, angle, axis, limit);
                if (angle > SIMD_EPSILON)
                {
                    adjustSwingAxisToUseEllipseNormal(axis);
                    // Bullet's positive limit Jacobian is on A; the negative moment
                    // on B points inward, so the same axis is outward for child B.
                    add(angle, limit, quatRotate(b, -axis));
                }
            }
            else
            {
                // Use Bullet's hinge coordinates, not arbitrary Euler components.
                const btVector3 childX = quatRotate(b, btVector3(1, 0, 0));
                const btVector3 parentX = quatRotate(a, btVector3(1, 0, 0));
                if (getSwingSpan1() >= getFixThreshValue())
                {
                    const btVector3 parentY = quatRotate(a, btVector3(0, 1, 0));
                    const btScalar angle = btAtan2(childX.dot(parentY), childX.dot(parentX));
                    add(btFabs(angle), getSwingSpan1(), quatRotate(a, btVector3(0, 0, angle < 0 ? -1 : 1)));
                }
                else if (getSwingSpan2() >= getFixThreshValue())
                {
                    const btVector3 parentZ = quatRotate(a, btVector3(0, 0, 1));
                    const btScalar angle = btAtan2(-childX.dot(parentZ), childX.dot(parentX));
                    add(btFabs(angle), getSwingSpan2(), quatRotate(a, btVector3(0, angle < 0 ? -1 : 1, 0)));
                }
            }
            if (getTwistSpan() > 0)
            {
                btScalar angle = 0;
                btVector3 axis(0, 0, 0);
                computeTwistLimitInfo(twist, angle, axis);
                if (angle > SIMD_EPSILON)
                    add(angle, getTwistSpan(), quatRotate(b, -axis));
            }
            // Bullet's ellipse-normal and twist Jacobians are appropriate for
            // its hard rows, but using them as spring directions creates a
            // nonconservative follower force. Differentiate the scalar soft
            // potential in SO(3), including its direction-dependent width.
            const PotentialSample potential = sampleSoftPotential(a.inverse() * b, settings);
            result.spring = allowedTorqueWorld(quatRotate(a, btVector3(-btScalar(potential.gradient[0]),
                -btScalar(potential.gradient[1]), -btScalar(potential.gradient[2]))));
            result.damping = allowedTorqueWorld(result.damping);
            if (components)
                *components = result;
            return result.total();
        }

        // Energy in the same native units as torque * radians. This read-only
        // query also permits an independent finite-difference gradient oracle.
        double softStopPotential(const btQuaternion& relativeInParentJointFrame, const SoftStopSettings& settings) const
        {
            return sampleSoftPotential(relativeInParentJointFrame, settings).value;
        }

        // Input uses the same A^-1*B convention as relativeRotation(), not Bullet's
        // reversed motor target convention. A positive result rejects an NBP target.
        btScalar limitViolation(const btQuaternion& relativeInParentJointFrame)
        {
            if (!validRotation(relativeInParentJointFrame))
                return BT_LARGE_FLOAT;
            btQuaternion cone, twist;
            split(relativeInParentJointFrame.inverse(), cone, twist);
            btScalar violation = 0;
            if (getSwingSpan1() >= getFixThreshValue() && getSwingSpan2() >= getFixThreshValue())
            {
                btScalar angle = 0, limit = 0;
                btVector3 axis(0, 0, 0);
                computeConeLimitInfo(cone, angle, axis, limit);
                if (!finite(angle) || !finite(limit))
                    return BT_LARGE_FLOAT;
                if (angle > SIMD_EPSILON)
                    violation = btMax(violation, angle - limit);
            }
            else
            {
                const btVector3 direction = quatRotate(relativeInParentJointFrame.normalized(), btVector3(1, 0, 0));
                const btScalar swing1 = btAtan2(direction.y(), direction.x());
                const btScalar swing2 = btAtan2(-direction.z(), direction.x());
                violation = btMax(violation, btFabs(swing1) - btMax(btScalar(0), getSwingSpan1()));
                violation = btMax(violation, btFabs(swing2) - btMax(btScalar(0), getSwingSpan2()));
            }
            if (getTwistSpan() >= 0)
            {
                btScalar angle = 0;
                btVector3 axis(0, 0, 0);
                computeTwistLimitInfo(twist, angle, axis);
                if (!finite(angle))
                    return BT_LARGE_FLOAT;
                violation = btMax(violation, angle - getTwistSpan());
            }
            return btMax(btScalar(0), violation);
        }

    private:
        struct PotentialSample
        {
            double value = 0;
            std::array<double, 3> gradient{ 0, 0, 0 }; // parent joint frame, world-left SO(3) derivative
        };

        PotentialSample sampleSoftPotential(const btQuaternion& rotation, const SoftStopSettings& settings) const
        {
            PotentialSample result;
            if (!valid(settings) || settings.widthRad <= 0 || settings.springAtLimit <= 0 || !validRotation(rotation)
                || !finite(getSwingSpan1()) || !finite(getSwingSpan2()) || !finite(getTwistSpan()))
                return result;
            using Vector = std::array<double, 3>;
            const auto cross = [](const Vector& u, const Vector& v) -> Vector {
                return { u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] };
            };
            const auto add = [](Vector& u, const Vector& v, double scale) {
                for (int i = 0; i < 3; ++i) u[i] += scale * v[i];
            };
            double x = rotation.x(), y = rotation.y(), z = rotation.z(), w = rotation.w();
            const double length = std::sqrt(x * x + y * y + z * z + w * w);
            x /= length; y /= length; z /= length; w /= length;
            const double pi = std::acos(-1.);
            const double spring = settings.springAtLimit;
            const double span1 = getSwingSpan1(), span2 = getSwingSpan2(), twistSpan = getTwistSpan();
            const bool swing1 = span1 >= getFixThreshValue(), swing2 = span2 >= getFixThreshValue();
            const Vector direction{ 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) };
            const Vector childX{ direction[0], 2 * (x * y + w * z), 2 * (x * z - w * y) };
            const double sine = std::hypot(direction[1], direction[2]);
            const double angle = std::atan2(sine, direction[0]);
            const double guardStart = pi * (5. / 6.);
            const double maximumAngle = (guardStart + pi) / 2;
            const auto maximum = [&](double limit) {
                if (!(limit > 0 && limit < pi)) return 0.;
                const double width = softWidth(settings.widthRad, limit).value;
                const double p = width + maximumAngle - limit;
                return p <= 0 ? 0. : p >= width ? spring * (p - 2 * width / 3)
                    : spring * p * p * p / (3 * width * width);
            };
            const double ceiling = maximum(swing1 && swing2 ? std::min(span1, span2)
                : swing1 ? span1 : swing2 ? span2 : 0.) + maximum(twistSpan);
            if (!(ceiling > 0)) return result;

            // The shortest-arc swing/twist chart is singular at antipodal X.
            // Far outside ordinary hard cones (150..180 degrees), blend the
            // entire energy to its finite upper bound. Differentiating the
            // blend keeps the guard conservative, and avoids a torque clamp.
            double guard = 1;
            Vector guardGradient{ 0, 0, 0 };
            if (angle > guardStart)
            {
                const double t = std::clamp((angle - guardStart) / (pi - guardStart), 0., 1.);
                const double r = 1 - t;
                guard = r * r * r * (1 + 3 * t + 6 * t * t);
                const double derivative = -30 * t * t * r * r / (pi - guardStart);
                if (sine > 0)
                    guardGradient = { 0, -derivative * childX[2] / sine, derivative * childX[1] / sine };
            }
            if (guard <= 0) { result.value = ceiling; return result; }

            // A hinge's atan2 chart also becomes singular when its forbidden
            // coordinate is almost a quarter turn off the hinge manifold.
            // This second guard is active only within five degrees of that pole.
            if (swing1 != swing2)
            {
                const int coordinate = swing1 ? 1 : 2;
                const double radius2 = childX[0] * childX[0] + childX[coordinate] * childX[coordinate];
                const double threshold = std::pow(std::sin(pi / 36), 2);
                if (radius2 < threshold)
                {
                    const double t = std::clamp(radius2 / threshold, 0., 1.);
                    const double h = t * t * t * (10 - 15 * t + 6 * t * t);
                    Vector plane{ childX[0], 0, 0 }; plane[coordinate] = childX[coordinate];
                    const Vector radiusGradient = cross(childX, plane);
                    Vector combined = guardGradient;
                    for (int i = 0; i < 3; ++i)
                        combined[i] = h * guardGradient[i] + guard * 60 * t * t * (1 - t) * (1 - t)
                            / threshold * radiusGradient[i];
                    guardGradient = combined;
                    guard *= h;
                }
            }
            if (guard <= 0) { result.value = ceiling; return result; }

            // Returns dU/d(angle), dU/d(limit), including the C1 width derivative.
            const auto radial = [&](double theta, double limit) -> std::array<double, 2> {
                if (!(limit > 0 && limit < pi)) return { 0, 0 };
                double thetaDerivative = 1;
                if (theta > guardStart)
                {
                    // Smooth the absolute-angle cusp at +/-pi as well as the
                    // swing chart pole, without exceeding the radial spring cap.
                    const double t = std::clamp((theta - guardStart) / (pi - guardStart), 0., 1.);
                    const double r = 1 - t;
                    theta = guardStart + (pi - guardStart)
                        * (t - 2.5 * std::pow(t, 4) + 3 * std::pow(t, 5) - std::pow(t, 6));
                    thetaDerivative = r * r * r * (1 + 3 * t + 6 * t * t);
                }
                const SoftWidth zone = softWidth(settings.widthRad, limit);
                const double width = zone.value;
                const double penetration = width + theta - limit;
                if (penetration <= 0) return { 0, 0 };
                const double u = std::min(1., penetration / width);
                const double c = spring * u * u;
                const double widthDerivative = zone.derivativeLimit;
                result.value += penetration >= width ? spring * (penetration - 2 * width / 3)
                    : spring * penetration * penetration * penetration / (3 * width * width);
                return { c * thetaDerivative, penetration >= width ? spring * (-1 + widthDerivative / 3)
                    : c * (-1 + widthDerivative * (1 - 2 * u / 3)) };
            };
            if (swing1 && swing2 && sine > 0)
            {
                // Exactly Bullet's elliptical cone angle/limit, evaluated in
                // double precision so the potential is a useful FD oracle.
                const double limit = sine / std::hypot(direction[1] / span1, direction[2] / span2);
                const auto derivatives = radial(angle, limit);
                const Vector g{ -derivatives[0] / sine,
                    derivatives[1] * limit * direction[1] / (sine * sine) * (1 - limit * limit / (span1 * span1)),
                    derivatives[1] * limit * direction[2] / (sine * sine) * (1 - limit * limit / (span2 * span2)) };
                const Vector childTorque = cross(direction, g);
                const Vector qv{ x, y, z };
                Vector parentTorque = childTorque;
                const Vector c1 = cross(qv, childTorque), c2 = cross(qv, c1);
                add(parentTorque, c1, 2 * w); add(parentTorque, c2, 2);
                add(result.gradient, parentTorque, -1);
            }
            else if (swing1 != swing2)
            {
                const int coordinate = swing1 ? 1 : 2;
                const double sign = swing1 ? 1 : -1;
                const double theta = std::atan2(sign * childX[coordinate], childX[0]);
                const auto derivatives = radial(std::abs(theta), swing1 ? span1 : span2);
                const double radius2 = childX[0] * childX[0] + childX[coordinate] * childX[coordinate];
                if (radius2 > 0)
                {
                    Vector g{ -sign * childX[coordinate] / radius2, 0, 0 };
                    g[coordinate] = sign * childX[0] / radius2;
                    add(result.gradient, cross(childX, g), derivatives[0] * (theta < 0 ? -1 : 1));
                }
            }
            if (twistSpan > 0 && twistSpan < pi)
            {
                const double denominator = x * x + w * w;
                if (denominator > 0)
                {
                    const auto derivatives = radial(2 * std::atan2(std::abs(x), std::abs(w)), twistSpan);
                    const double sign = x * w < 0 ? -1 : 1;
                    add(result.gradient, Vector{ 1, (w * z + x * y) / denominator,
                        (-w * y + x * z) / denominator }, sign * derivatives[0]);
                }
            }
            for (int i = 0; i < 3; ++i)
                result.gradient[i] = guard * result.gradient[i] + (result.value - ceiling) * guardGradient[i];
            result.value = guard * result.value + (1 - guard) * ceiling;
            return result;
        }

        btScalar getFixThreshValue() const
        {
            // Bullet 3.24 exposes a non-const getter although it does not mutate.
            return const_cast<ConeTwist*>(this)->getFixThresh();
        }

        static void split(const btQuaternion& relativeBullet, btQuaternion& cone, btQuaternion& twist)
        {
            const btQuaternion q = canonicalRotation(relativeBullet);
            const btVector3 axis(1, 0, 0);
            btVector3 direction = quatRotate(q, axis);
            direction.normalize();
            cone = shortestArcQuat(axis, direction).normalized();
            twist = canonicalRotation(cone.inverse() * q);
        }
    };

    struct AngularImpulse
    {
        btRigidBody* body = nullptr;
        btVector3 impulse = btVector3(0, 0, 0);
        btScalar maxAddedAngularSpeed = 0;
    };

    inline bool accumulateAngularImpulse(std::vector<AngularImpulse>& accumulated, btRigidBody* body,
        const btVector3& impulse, btScalar maxAddedAngularSpeed)
    {
        if (!body || !finite(impulse) || !finite(maxAddedAngularSpeed) || maxAddedAngularSpeed < 0)
            return false;
        const auto found = std::find_if(accumulated.begin(), accumulated.end(),
            [&](const AngularImpulse& entry) { return entry.body == body; });
        if (found != accumulated.end())
        {
            const btVector3 sum = found->impulse + impulse;
            if (!finite(sum))
                return false;
            found->impulse = sum;
            found->maxAddedAngularSpeed = btMin(found->maxAddedAngularSpeed, maxAddedAngularSpeed);
        }
        else
            accumulated.push_back({ body, impulse, maxAddedAngularSpeed });
        return true;
    }

    inline btScalar commonImpulseScale(const std::vector<AngularImpulse>& accumulated)
    {
        btScalar scale = 1;
        for (const AngularImpulse& entry : accumulated)
        {
            if (!entry.body || !finite(entry.impulse) || !finite(entry.maxAddedAngularSpeed)
                || entry.maxAddedAngularSpeed < 0)
                return 0;
            const btVector3 delta = entry.body->getInvInertiaTensorWorld() * entry.impulse;
            if (!finite(delta) || !finite(delta.length2()))
                return 0;
            const btScalar magnitude = delta.length();
            if (magnitude > entry.maxAddedAngularSpeed && magnitude > 0)
                scale = btMin(scale, entry.maxAddedAngularSpeed / magnitude);
        }
        return scale;
    }

    inline btScalar effectiveInertia(const btRigidBody& a, const btRigidBody& b, const btVector3& unitAxisWorld)
    {
        const btScalar inverse = unitAxisWorld.dot(
            (a.getInvInertiaTensorWorld() + b.getInvInertiaTensorWorld()) * unitAxisWorld);
        return finite(inverse) && inverse > SIMD_EPSILON ? 1 / inverse : 0;
    }
}

#endif
