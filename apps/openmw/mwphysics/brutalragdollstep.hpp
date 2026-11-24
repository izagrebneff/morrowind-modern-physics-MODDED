#ifndef OPENMW_MWPHYSICS_BRUTALRAGDOLLSTEP_H
#define OPENMW_MWPHYSICS_BRUTALRAGDOLLSTEP_H

#include <BulletDynamics/Dynamics/btActionInterface.h>

namespace MWPhysics
{
    // Separate pre-step hook for the real Brutal cone/twist spring torques.
    // Bullet clears accumulated forces at the end of stepSimulation(), so
    // updateAction() is too late to provide a reliable continuous moment.
    class BrutalRagdollStep : public btActionInterface
    {
    public:
        virtual void beforeBrutalStep(btScalar dt) = 0;
    };
}

#endif
