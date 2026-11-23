#ifndef OPENMW_MWPHYSICS_RAGDOLLSTEP_H
#define OPENMW_MWPHYSICS_RAGDOLLSTEP_H

#include <cstdint>
#include <map>
#include <string>
#include <BulletDynamics/Dynamics/btActionInterface.h>
#include <LinearMath/btVector3.h>
class btTypedConstraint;

namespace MWPhysics
{
    // Engine length/mass/time units throughout. Configuration is copied while the
    // scheduler owns its world lock, never read from Settings on a physics worker.
    struct RagdollControlConfig
    {
        bool enabled = false;
        bool softStops = true;
        bool neutralPose = true;
        bool limbResponse = true;
        bool selfCollision = true;
        float softStrength = 1.f;
        float toneStrength = 1.f;
        float impactStrength = 1.f;
        // Water applies to every body, with or without the anatomical profile.
        bool waterEnabled = false;
        float waterHeight = 0.f;
    };

    struct RagdollImpact
    {
        std::uint64_t id = 0;
        std::size_t part = 0;
        btVector3 point{ 0, 0, 0 };
        btVector3 impulse{ 0, 0, 0 };
        bool alreadyApplied = false;
    };

    class RagdollStepAction : public btActionInterface
    {
    public:
        virtual void beforeStep(btScalar dt) = 0;
        virtual int solverIterations() const = 0;
        virtual void configure(const RagdollControlConfig& config, btCollisionWorld* world) = 0;
        virtual bool impact(const RagdollImpact& event) = 0;
        virtual bool kick(std::size_t part, const btVector3& point, const btVector3& velocity,
            std::uint64_t id) = 0;
        virtual void inherit(const RagdollStepAction& previous, btTypedConstraint* removed,
            btCollisionWorld* world = nullptr) = 0;
        virtual std::map<std::string, double> snapshot() const = 0;
    };
}
#endif
