#include "ragdoll.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string_view>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/Dynamics/btActionInterface.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btMotionState.h>

#include <osg/Quat>

#include <components/misc/convert.hpp>
#include <components/misc/constants.hpp>

#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/class.hpp"

#include "collisiontype.hpp"
#include "ragdollenhancement.hpp"
#include "mtphysics.hpp"

namespace MWPhysics
{
    // Receives body transforms from Bullet on the physics thread; the main thread reads them for the bones.
    class RagdollMotionState final : public btMotionState
    {
    public:
        explicit RagdollMotionState(const btTransform& transform)
            : mTransform(transform)
            , mPreviousTransform(transform)
        {
        }

        void getWorldTransform(btTransform& transform) const override
        {
            std::lock_guard lock(mMutex);
            transform = mTransform;
        }

        void setWorldTransform(const btTransform& transform) override
        {
            std::lock_guard lock(mMutex);
            mPreviousTransform = mTransform;
            mTransform = transform;
        }

        /// Between the last two physics steps: 0 is the one before, 1 the latest.
        btTransform getInterpolatedTransform(btScalar factor) const
        {
            std::lock_guard lock(mMutex);
            return btTransform(mPreviousTransform.getRotation().slerp(mTransform.getRotation(), factor),
                mPreviousTransform.getOrigin().lerp(mTransform.getOrigin(), factor));
        }

    private:
        mutable std::mutex mMutex;
        btTransform mTransform;
        btTransform mPreviousTransform;
    };

    // Friction in the joints: each step, some of the way jointed parts turn relative to each other is taken out.
    // Without it a body collapses like a sack of liquid; with it, limbs still flop but settle instead of sloshing.
    class JointFriction final : public btActionInterface
    {
    public:
        std::vector<std::pair<btRigidBody*, btRigidBody*>> mPairs;
        struct EnhancedLink
        {
            RagdollEnhancement::ConeTwist* mJoint;
            RagdollEnhancement::SoftStopSettings mSoft;
        };
        std::vector<EnhancedLink> mEnhanced;

        void updateAction(btCollisionWorld*, btScalar timeStep) override
        {
            // Real Brutal Morrowind soft-stop torque: spring back smoothly
            // before Bullet's hard cone/twist constraint reaches its limit.
            // The joints and settings originate from Brutal's ConeTwist model,
            // not from the old fork's purely viscous joint friction.
            for (const EnhancedLink& link : mEnhanced)
            {
                auto& a = link.mJoint->getRigidBodyA();
                auto& b = link.mJoint->getRigidBodyB();
                if (!a.isActive() && !b.isActive())
                    continue;
                const btVector3 torque = link.mJoint->softStopTorqueWorld(link.mSoft);
                if (!RagdollEnhancement::finite(torque))
                    continue;
                a.applyTorque(-torque);
                b.applyTorque(torque);
            }
            constexpr btScalar rate = 14; // per second: about a fifth of the turning gone each step at 60 Hz
            const btScalar fraction = 1 - std::exp(-rate * timeStep);
            for (const auto& [a, b] : mPairs)
            {
                if (!a->isActive() && !b->isActive())
                    continue;
                const btVector3 relative = b->getAngularVelocity() - a->getAngularVelocity();
                if (relative.length2() < 1e-4f)
                    continue;
                // The lighter part gives more.
                const btScalar massA = a->getInvMass() > 0 ? 1 / a->getInvMass() : 0;
                const btScalar massB = b->getInvMass() > 0 ? 1 / b->getInvMass() : 0;
                if (massA + massB <= 0)
                    continue;
                const btVector3 change = relative * fraction;
                a->setAngularVelocity(a->getAngularVelocity() + change * (massB / (massA + massB)));
                b->setAngularVelocity(b->getAngularVelocity() - change * (massA / (massA + massB)));
            }
        }

        void debugDraw(btIDebugDraw*) override {}
    };

    // Restrict the descent of dead flyers, never living creatures. In the
    // previous working build this was the profile used for netches/cliff racers.
    class FlyingCorpseFall final : public btActionInterface
    {
    public:
        std::vector<btRigidBody*> mBodies;

        void updateAction(btCollisionWorld*, btScalar timeStep) override
        {
            const btScalar fraction = 1 - std::exp(-3.0f * timeStep);
            constexpr btScalar maxFallSpeed = 950.f;
            for (btRigidBody* body : mBodies)
            {
                // The adaptive freeze makes limbs static; never add velocity to
                // an anchored body after it has settled.
                if (body->isStaticOrKinematicObject())
                    continue;
                btVector3 velocity = body->getLinearVelocity();
                if (velocity.z() < -maxFallSpeed)
                {
                    velocity.setZ(-maxFallSpeed + (velocity.z() + maxFallSpeed) * (1 - fraction));
                    body->setLinearVelocity(velocity);
                }
            }
        }

        void debugDraw(btIDebugDraw*) override {}
    };

    struct Ragdoll::Part final : public PtrHolder
    {
        explicit Part(const MWWorld::Ptr& ptr)
            : PtrHolder(ptr, osg::Vec3f())
        {
        }

        std::string mBone;
        std::unique_ptr<btCollisionShape> mShape;
        std::unique_ptr<RagdollMotionState> mMotionState;
        btRigidBody* mBody = nullptr; // owned by mCollisionObject
        btScalar mOriginalMass = 0; // restored after dragging a frozen corpse

        void setBody(std::unique_ptr<btRigidBody> body)
        {
            mBody = body.get();
            mCollisionObject = std::move(body);
        }
        btTransform mBoneFromBody; // the bone's transform in the body's frame (without scale)
        osg::Vec3f mBoneScale;
        osg::Vec3f mBoxHalfExtents; // for a single box body
        float mRelativeDensity = 1.05f; // to water: a creature's body sinks slowly (people's: see sPartDefs)

        float getRelativeDensity() const override { return mRelativeDensity; }
    };

    namespace
    {
        struct PartDef
        {
            std::string_view mBone;
            std::string_view mEnd; // where the part ends; empty for the head
            float mRadius;
            float mMass;
            int mParent; // index in sPartDefs, -1 for none
            // Joint limits (degrees): swing around two axes, twist around the bone
            float mSwing1;
            float mSwing2;
            float mTwist;
            // Relative to water: the chest floats high (the lungs), the rest only just, so a body floats face
            // down, near the surface, its head and limbs drooping a little.
            float mDensity;
        };

        // A human-sized body (about 128 units tall). Knees and elbows swing mostly one way, so one swing span
        // is kept small; shoulders and hips are loose.
        constexpr PartDef sPartDefs[] = {
            { "Bip01 Pelvis", "Bip01 Spine1", 11.f, 12.f, -1, 0, 0, 0, 0.8f },
            { "Bip01 Spine1", "Bip01 Neck", 12.f, 15.f, 0, 25, 25, 15, 0.65f },
            { "Bip01 Head", "", 8.f, 5.f, 1, 40, 40, 40, 0.95f },
            { "Bip01 L UpperArm", "Bip01 L Forearm", 4.5f, 3.f, 1, 80, 80, 40, 0.98f },
            { "Bip01 L Forearm", "Bip01 L Hand", 4.f, 2.f, 3, 70, 10, 20, 0.98f },
            { "Bip01 R UpperArm", "Bip01 R Forearm", 4.5f, 3.f, 1, 80, 80, 40, 0.98f },
            { "Bip01 R Forearm", "Bip01 R Hand", 4.f, 2.f, 5, 70, 10, 20, 0.98f },
            { "Bip01 L Thigh", "Bip01 L Calf", 6.5f, 7.f, 0, 60, 40, 15, 0.98f },
            { "Bip01 L Calf", "Bip01 L Foot", 5.f, 4.f, 7, 70, 10, 10, 0.98f },
            { "Bip01 R Thigh", "Bip01 R Calf", 6.5f, 7.f, 0, 60, 40, 15, 0.98f },
            { "Bip01 R Calf", "Bip01 R Foot", 5.f, 4.f, 9, 70, 10, 10, 0.98f },
        };

        // A coherent initial rigid-body velocity field: the torso pitches away from the
        // blow, the feet lag, and the entire body turns slightly. Unlike independent limb
        // kicks this keeps the starting joint velocities approximately compatible.
        struct FatalImpact
        {
            btVector3 linear;
            btVector3 angular{ 0, 0, 0 };
            bool active = false;

            FatalImpact(const MWWorld::Ptr& actor, const osg::Vec3f& velocity, const osg::Vec3f& kick,
                bool humanoid)
                : linear(Misc::Convert::toBullet(velocity))
            {
                btVector3 direction(kick.x(), kick.y(), 0);
                const btScalar strength = direction.length();
                if (strength < 1.f)
                    return;
                active = true;
                direction /= strength;
                const bool magic = actor.getClass().getCreatureStats(actor).hasMagicDeathKick();
                // Retain a little horizontal locomotion for a physical hit, but never inherit
                // upward movement from jumping. Magic translates the whole body much faster.
                linear = direction * (magic ? strength : strength * 0.62f)
                    + btVector3(velocity.x(), velocity.y(), 0) * (magic ? 0.f : 0.18f);
                linear.setZ(std::min(velocity.z(), 0.f) - 15.f);
                const btScalar pitch = magic ? (humanoid ? 2.35f : 1.65f)
                    : (humanoid ? 2.45f : 1.6f) + std::min<btScalar>(strength / btScalar(280), btScalar(1)) * 0.7f;
                const float yaw = actor.getRefData().getPosition().rot[2];
                const btVector3 forward(std::sin(yaw), std::cos(yaw), 0);
                const float side = static_cast<float>(forward.cross(direction).z());
                const float turn = (side < -0.05f ? -1.f : 1.f) * (0.4f + 0.35f * std::abs(side));
                angular = btVector3(0, 0, 1).cross(direction) * pitch + btVector3(0, 0, turn);
            }

            void apply(btRigidBody& body, const btVector3& pivot, const btVector3& extra = btVector3(0, 0, 0)) const
            {
                btVector3 v = linear + extra;
                if (active)
                {
                    btVector3 rotational = angular.cross(body.getWorldTransform().getOrigin() - pivot);
                    constexpr btScalar maxDifferential = 180.f;
                    if (rotational.length2() > maxDifferential * maxDifferential)
                        rotational *= maxDifferential / rotational.length();
                    v += rotational;
                    // No initially upward-moving limb; gravity and ordinary contacts remain active.
                    v.setZ(std::min(v.z(), btScalar(0)));
                    body.setAngularVelocity(angular);
                }
                body.setLinearVelocity(v);
            }
        };

        btTransform withoutScale(const osg::Matrixf& matrix, osg::Vec3f& scale)
        {
            osg::Vec3f translation;
            osg::Quat rotation;
            osg::Quat scaleOrientation;
            matrix.decompose(translation, rotation, scale, scaleOrientation);
            return btTransform(Misc::Convert::toBullet(rotation), Misc::Convert::toBullet(translation));
        }
    }

    const std::vector<std::string>& Ragdoll::getRequiredBones()
    {
        static const std::vector<std::string> bones = { "Bip01 Pelvis", "Bip01 Spine1", "Bip01 Neck", "Bip01 Head",
            "Bip01 L UpperArm", "Bip01 L Forearm", "Bip01 L Hand", "Bip01 R UpperArm", "Bip01 R Forearm",
            "Bip01 R Hand", "Bip01 L Thigh", "Bip01 L Calf", "Bip01 L Foot", "Bip01 R Thigh", "Bip01 R Calf",
            "Bip01 R Foot" };
        return bones;
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const BoneMatrices& bones, const osg::Vec3f& velocity,
        const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler, const BoneVelocities& motion)
        : mTaskScheduler(scheduler)
    {
        const auto position = [&](std::string_view bone) { return bones.find(bone)->second.getTrans(); };
        const FatalImpact impact(actor, velocity, kick, true);
        const btVector3 pivot = Misc::Convert::toBullet(position("Bip01 Pelvis"));

        for (const PartDef& def : sPartDefs)
        {
            auto part = std::make_shared<Part>(actor);
            part->mBone = def.mBone;

            const osg::Vec3f start = position(def.mBone);
            osg::Vec3f end;
            if (!def.mEnd.empty())
                end = position(def.mEnd);
            else
            {
                // The head goes on from the neck.
                osg::Vec3f up = start - position("Bip01 Neck");
                if (up.normalize() < 1e-3f)
                    up = osg::Vec3f(0, 0, 1);
                end = start + up * 14.f;
            }
            btVector3 axis = Misc::Convert::toBullet(end - start);
            const btScalar length = axis.length();
            axis = length > 1e-3f ? axis / length : btVector3(0, 0, 1);

            // A capsule along the bone (capsules lie along their Y axis).
            const btTransform bodyWorld(
                shortestArcQuat(btVector3(0, 1, 0), axis), Misc::Convert::toBullet((start + end) * 0.5f));
            part->mShape = std::make_unique<btCapsuleShape>(
                def.mRadius, std::max(length - 2 * def.mRadius, btScalar(0.1)));
            part->mBoneFromBody
                = bodyWorld.inverse() * withoutScale(bones.find(def.mBone)->second, part->mBoneScale);

            part->mRelativeDensity = def.mDensity;
            makeBody(*part, bodyWorld, def.mMass, def.mRadius);

            impact.apply(*part->mBody, pivot);
            // Add a limited part of the last animated relative velocity without
            // double-counting actor locomotion, which FatalImpact already handles.
            if (const auto found = motion.find(part->mBone); found != motion.end())
                part->mBody->setLinearVelocity(part->mBody->getLinearVelocity()
                    + Misc::Convert::toBullet(found->second) * btScalar(0.35f));

            mParts.push_back(std::move(part));
        }
        addToWorld();

        // Joints where each part meets its parent, twisting around the part's own bone.
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            const PartDef& def = sPartDefs[i];
            if (def.mParent >= 0)
                addJoint(def.mParent, i, position(def.mBone), def.mSwing1, def.mSwing2, def.mTwist);
        }
        addJointFriction();
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const btTransform& bodyWorld, const osg::Vec3f& halfExtents,
        float mass, const osg::Matrixf& baseWorld, PhysicsTaskScheduler* scheduler, const osg::Vec3f& velocity)
        : mTaskScheduler(scheduler)
    {
        auto part = std::make_shared<Part>(actor);
        part->mBoxHalfExtents = halfExtents;
        part->mShape = std::make_unique<btBoxShape>(Misc::Convert::toBullet(halfExtents));
        part->mBoneFromBody = bodyWorld.inverse() * withoutScale(baseWorld, part->mBoneScale);
        makeBody(*part, bodyWorld, mass, 0);
        const auto& kick = actor.getClass().getCreatureStats(actor).getDeathKick();
        const FatalImpact impact(actor, kick.length2() > 1.f ? osg::Vec3f() : velocity, kick, false);
        impact.apply(*part->mBody, bodyWorld.getOrigin());
        mParts.push_back(std::move(part));
        addToWorld();
    }

    std::unique_ptr<Ragdoll> Ragdoll::fromSkeleton(const MWWorld::Ptr& actor, const std::vector<SkinnedBone>& bones,
        const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler)
    {
        // A part for each bone that moves a fair share of the mesh (legs, body, head, tail...), at most a body's
        // worth; smaller ones (toes, jaws, tail tips) just go along with the part above them.
        float largest = 0;
        for (const SkinnedBone& bone : bones)
            largest = std::max(largest, bone.mRadius);
        std::vector<size_t> chosen;
        for (size_t i = 0; i < bones.size(); ++i)
            if (bones[i].mRadius >= largest * 0.2f && bones[i].mRadius > 1.f)
                chosen.push_back(i);
        constexpr size_t maxParts = 12;
        if (chosen.size() > maxParts)
        {
            std::vector<size_t> bySize = chosen;
            std::nth_element(bySize.begin(), bySize.begin() + maxParts - 1, bySize.end(),
                [&](size_t a, size_t b) { return bones[a].mRadius > bones[b].mRadius; });
            const float cutoff = bones[bySize[maxParts - 1]].mRadius;
            std::erase_if(chosen, [&](size_t i) { return bones[i].mRadius < cutoff; });
            chosen.resize(std::min(chosen.size(), maxParts));
        }
        if (chosen.size() < 2)
            return nullptr;

        std::unique_ptr<Ragdoll> ragdoll(new Ragdoll(scheduler));
        // Procedural creature limb capsules frequently overlap at rest. Disable
        // all internal contacts, not only directly jointed contacts: otherwise
        // Bullet pushes the rig against itself and it spins uncontrollably.
        ragdoll->mDisableInternalCollision = true;
        std::vector<int> partOfBone(bones.size(), -1);
        for (size_t i : chosen)
        {
            const SkinnedBone& bone = bones[i];
            auto part = std::make_shared<Part>(actor);
            part->mBone = bone.mName;
            const btTransform boneWorld = withoutScale(bone.mWorld, part->mBoneScale);
            const float scale = part->mBoneScale.x();

            // The bone's share of the mesh lies around the center of its bounds. Away from the bone it is a
            // limb, as long as twice that and as thick as the bounds allow; around the bone, a lump.
            const btVector3 center = boneWorld * Misc::Convert::toBullet(bone.mCenter * scale);
            const float radius = bone.mRadius * scale;
            const float half = bone.mCenter.length() * scale;
            btTransform bodyWorld;
            float thickness;
            if (half > radius * 0.35f)
            {
                const btVector3 axis = (center - boneWorld.getOrigin()).normalized();
                thickness = std::clamp(std::sqrt(std::max(radius * radius - half * half, 0.f)) * 0.9f,
                    std::max(radius * 0.2f, 1.5f), radius);
                part->mShape = std::make_unique<btCapsuleShape>(thickness, std::max(2 * half - 2 * thickness, 0.1f));
                bodyWorld = btTransform(shortestArcQuat(btVector3(0, 1, 0), axis), center);
            }
            else
            {
                thickness = std::max(radius * 0.7f, 1.5f);
                part->mShape = std::make_unique<btSphereShape>(thickness);
                bodyWorld = btTransform(boneWorld.getRotation(), center);
            }
            part->mBoneFromBody = bodyWorld.inverse() * boneWorld;

            // Flesh is about as dense as water (~70 units to a meter).
            btVector3 aabbMin;
            btVector3 aabbMax;
            part->mShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
            const btVector3 size = aabbMax - aabbMin;
            const float volume = static_cast<float>(size.x() * size.y() * size.z()) * 0.6f / (70.f * 70.f * 70.f);
            makeBody(*part, bodyWorld, std::clamp(volume * 1000.f, 0.3f, 200.f), thickness);


            partOfBone[i] = static_cast<int>(ragdoll->mParts.size());
            ragdoll->mParts.push_back(std::move(part));
        }
        // Joined to the part above; any parts with none (bits not under the main body) to the biggest.
        // Some modded creatures have malformed skeleton parent links (invalid
        // indices or cycles). Never index outside the rig or spin forever while
        // resolving a simulated ancestor; fall back to the main body instead.
        const auto parentPart = [&](size_t bone) {
            int parent = bones[bone].mParent;
            for (size_t depth = 0; depth < bones.size(); ++depth)
            {
                if (parent < 0 || static_cast<size_t>(parent) >= bones.size())
                    break;
                if (partOfBone[static_cast<size_t>(parent)] >= 0)
                    return partOfBone[static_cast<size_t>(parent)];
                parent = bones[static_cast<size_t>(parent)].mParent;
            }
            return -1;
        };
        int root = 0;
        for (size_t i = 0; i < chosen.size(); ++i)
            if (parentPart(chosen[i]) < 0 && bones[chosen[i]].mRadius > bones[chosen[root]].mRadius)
                root = static_cast<int>(i);
        const FatalImpact impact(actor, velocity, kick, false);
        const btVector3 pivot = ragdoll->mParts[static_cast<size_t>(root)]->mBody->getWorldTransform().getOrigin();
        for (const auto& part : ragdoll->mParts)
            impact.apply(*part->mBody, pivot);
        ragdoll->addToWorld();

        for (size_t i = 0; i < chosen.size(); ++i)
        {
            int parent = parentPart(chosen[i]);
            if (parent < 0 && static_cast<int>(i) != root)
                parent = root;
            if (parent >= 0)
                ragdoll->addJoint(parent, i, bones[chosen[i]].mWorld.getTrans(), 30, 30, 15);
        }
        ragdoll->mMainPartIndex = static_cast<size_t>(root);
        ragdoll->addJointFriction();

        // Bind every original skinned bone to the nearest simulated ancestor.
        // Skipping tiny wing bones caused insect skins to stretch away from
        // their torso when the physical parts dropped vertically.
        for (size_t i = 0; i < bones.size(); ++i)
        {
            int driver = partOfBone[i];
            if (driver < 0)
            {
                int parent = bones[i].mParent;
                for (size_t depth = 0; depth < bones.size(); ++depth)
                {
                    if (parent < 0 || static_cast<size_t>(parent) >= bones.size())
                        break;
                    if (partOfBone[static_cast<size_t>(parent)] >= 0)
                    {
                        driver = partOfBone[static_cast<size_t>(parent)];
                        break;
                    }
                    parent = bones[static_cast<size_t>(parent)].mParent;
                }
            }
            if (driver < 0)
                driver = root;

            const size_t selectedBone = chosen[static_cast<size_t>(driver)];
            const osg::Matrixf relative
                = bones[i].mWorld * osg::Matrixf::inverse(bones[selectedBone].mWorld);
            ragdoll->mSkinnedBoneBindings.push_back(
                { bones[i].mName, static_cast<size_t>(driver), relative });
        }
        return ragdoll;
    }

    std::unique_ptr<Ragdoll> Ragdoll::fromRigidPieces(const MWWorld::Ptr& actor, const std::vector<RigidPiece>& pieces,
        const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler)
    {
        // The bigger pieces come loose (legs, shell, head...), at most a handful; the rest stay on them.
        const auto size = [&](size_t i) { return (pieces[i].mMax - pieces[i].mMin).length() * 0.5f; };
        float largest = 0;
        for (size_t i = 0; i < pieces.size(); ++i)
            largest = std::max(largest, size(i));
        std::vector<size_t> chosen;
        for (size_t i = 0; i < pieces.size(); ++i)
            if (size(i) >= largest * 0.12f && size(i) > 1.f)
                chosen.push_back(i);
        constexpr size_t maxParts = 16;
        if (chosen.size() > maxParts)
        {
            std::vector<size_t> bySize = chosen;
            std::nth_element(bySize.begin(), bySize.begin() + maxParts - 1, bySize.end(),
                [&](size_t a, size_t b) { return size(a) > size(b); });
            const float cutoff = size(bySize[maxParts - 1]);
            std::erase_if(chosen, [&](size_t i) { return size(i) < cutoff; });
            chosen.resize(std::min(chosen.size(), maxParts));
        }
        // A small piece with no loose piece above it would have nothing to go along with (it would hang in the
        // air where the body was): it comes loose too, taking any small ones below it along.
        std::vector<bool> loose(pieces.size(), false);
        for (size_t i : chosen)
            loose[i] = true;
        for (size_t i = 0; i < pieces.size(); ++i)
        {
            if (loose[i])
                continue;
            bool carried = false;
            int parent = pieces[i].mParent;
            // Modded rigid-piece hierarchies can contain invalid indices or
            // cycles. Bound traversal by the number of pieces and use a valid
            // parent only; otherwise release this piece as a safe fallback.
            for (size_t depth = 0; depth < pieces.size() && !carried; ++depth)
            {
                if (parent < 0 || static_cast<size_t>(parent) >= pieces.size())
                    break;
                const size_t parentIndex = static_cast<size_t>(parent);
                carried = loose[parentIndex];
                parent = pieces[parentIndex].mParent;
            }
            if (!carried)
                loose[i] = true;
        }
        chosen.clear();
        for (size_t i = 0; i < pieces.size(); ++i)
            if (loose[i])
                chosen.push_back(i);
        if (chosen.size() < 2)
            return nullptr;

        std::unique_ptr<Ragdoll> ragdoll(new Ragdoll(scheduler));
        ragdoll->mDisableInternalCollision = true;
        osg::Vec3f middle;
        std::vector<btTransform> bodies;
        for (size_t i : chosen)
        {
            const RigidPiece& piece = pieces[i];
            auto part = std::make_shared<Part>(actor);
            part->mBone = piece.mName;
            const btTransform nodeWorld = withoutScale(piece.mWorld, part->mBoneScale);
            const osg::Vec3f scale = part->mBoneScale;
            const osg::Vec3f center = (piece.mMin + piece.mMax) * 0.5f;
            osg::Vec3f half = (piece.mMax - piece.mMin) * 0.5f;
            half = osg::Vec3f(std::max(half.x() * scale.x(), 1.f), std::max(half.y() * scale.y(), 1.f),
                std::max(half.z() * scale.z(), 1.f));
            part->mBoxHalfExtents = half;
            part->mShape = std::make_unique<btBoxShape>(Misc::Convert::toBullet(half));
            const btTransform bodyWorld(nodeWorld.getRotation(),
                nodeWorld * Misc::Convert::toBullet(osg::Vec3f(
                    center.x() * scale.x(), center.y() * scale.y(), center.z() * scale.z())));
            part->mBoneFromBody = bodyWorld.inverse() * nodeWorld;
            // Shells and limbs: about half as dense as water, a box being roomier than what it holds.
            const float volume = 8.f * half.x() * half.y() * half.z() / (70.f * 70.f * 70.f);
            makeBody(*part, bodyWorld, std::clamp(volume * 500.f, 0.2f, 100.f),
                std::min({ half.x(), half.y(), half.z() }));
            middle += Misc::Convert::toOsg(bodyWorld.getOrigin());
            bodies.push_back(bodyWorld);
            ragdoll->mParts.push_back(std::move(part));
        }
        middle /= static_cast<float>(chosen.size());

        // Falling apart: the pieces spread a little from the middle, but never get a vertical launch.
        const FatalImpact impact(actor, velocity, kick, false);
        for (size_t i = 0; i < ragdoll->mParts.size(); ++i)
        {
            osg::Vec3f out = Misc::Convert::toOsg(bodies[i].getOrigin()) - middle;
            out.z() = 0;
            if (out.normalize() > 0)
                out *= 25.f;
            impact.apply(*ragdoll->mParts[i]->mBody, Misc::Convert::toBullet(middle), Misc::Convert::toBullet(out));
        }
        ragdoll->addToWorld();
        return ragdoll;
    }

    Ragdoll::Ragdoll(PhysicsTaskScheduler* scheduler)
        : mTaskScheduler(scheduler)
    {
    }

    void Ragdoll::makeBody(Part& part, const btTransform& bodyWorld, float mass, float thickness)
    {
        // Bodies should carry far more inertia than the old lightweight pieces.
        // Keep their separately modelled buoyancy (relative density) unchanged.
        constexpr float massMultiplier = 2.75f;
        const float simulatedMass = mass * massMultiplier;
        part.mOriginalMass = simulatedMass;
        btVector3 inertia(0, 0, 0);
        part.mShape->calculateLocalInertia(simulatedMass, inertia);
        part.mMotionState = std::make_unique<RagdollMotionState>(bodyWorld);
        btRigidBody::btRigidBodyConstructionInfo info(
            simulatedMass, part.mMotionState.get(), part.mShape.get(), inertia);
        info.m_friction = 0.8f;
        info.m_restitution = 0.f;
        info.m_linearDamping = 0.15f;
        info.m_angularDamping = 0.5f;
        // In game units (~70 per meter), like the other simulated objects.
        info.m_linearSleepingThreshold = 4.f;
        info.m_angularSleepingThreshold = 1.f;
        auto body = std::make_unique<btRigidBody>(info);
        if (thickness > 0)
        {
            body->setCcdMotionThreshold(thickness * 0.5f);
            body->setCcdSweptSphereRadius(thickness * 0.8f);
            // Round parts would roll away down any slope. Rolling resistance has to grow with the size (holding
            // still takes a torque of weight x radius), so this holds them on slopes up to about 30 degrees.
            body->setRollingFriction(thickness);
            body->setSpinningFriction(thickness * 0.3f);
        }
        body->setUserPointer(&part);
        part.setBody(std::move(body));
    }

    void Ragdoll::addToWorld()
    {
        mFlyingCorpse = !mParts.empty() && mParts.front()->getPtr().getClass().canFly(mParts.front()->getPtr());
        // Creature limbs, wings and feelers often start inside overlapping collision
        // capsules. Internal contacts can violently spin the entire ragdoll.
        // Disable creature-to-self contacts, while preserving contacts with
        // terrain, actors, objects and other corpses. Humanoid NPCs retain
        // their existing self-collision behaviour.
        if (mFlyingCorpse || mDisableInternalCollision)
            for (size_t i = 0; i < mParts.size(); ++i)
                for (size_t j = i + 1; j < mParts.size(); ++j)
                {
                    mParts[i]->mBody->setIgnoreCollisionCheck(mParts[j]->mBody, true);
                    mParts[j]->mBody->setIgnoreCollisionCheck(mParts[i]->mBody, true);
                }

        // Self-collision is now enabled: hands/forearms, legs and the torso cannot
        // freely pass through non-adjacent parts of the same ragdoll.
        // Directly jointed parts still ignore each other via addConstraint(..., true)
        // in PhysicsTaskScheduler, because their capsules overlap at the joints.
        // Other ragdolls and loose items remain normally collidable.
        for (const auto& part : mParts)
            mTaskScheduler->addRigidBody(part->mBody, CollisionType_Dynamic,
                CollisionType_DynamicSupport | CollisionType_Actor | CollisionType_Dynamic | CollisionType_Projectile);

        if (mFlyingCorpse)
        {
            // Restore the previous controlled aerial fall: gravity stays strong
            // enough for visible descent, and the velocity cap avoids spikes.
            setGravityScale(1.20f);
            auto controller = std::make_unique<FlyingCorpseFall>();
            for (const auto& part : mParts)
            {
                part->mBody->setDamping(0.65f, 0.38f);
                controller->mBodies.push_back(part->mBody);
            }
            mFlyingFallController = std::move(controller);
            mTaskScheduler->addAction(mFlyingFallController.get());
        }
    }

    void Ragdoll::setGravityScale(float scale)
    {
        const btVector3 gravity(0, 0, -Constants::GravityConst * Constants::UnitsPerMeter * scale);
        for (const auto& part : mParts)
            mTaskScheduler->setBodyGravity(part->mBody, gravity);
    }

    void Ragdoll::addJoint(size_t parentIndex, size_t childIndex, const osg::Vec3f& position, float swing1,
        float swing2, float twist)
    {
        // Twisting around the child part's own length.
        btRigidBody& parent = *mParts[parentIndex]->mBody;
        btRigidBody& child = *mParts[childIndex]->mBody;
        const btTransform& childWorld = child.getWorldTransform();
        btVector3 childAxis = childWorld.getBasis() * btVector3(0, 1, 0);
        const btTransform jointWorld(
            shortestArcQuat(btVector3(1, 0, 0), childAxis), Misc::Convert::toBullet(position));
        auto joint = std::make_unique<RagdollEnhancement::ConeTwist>(
            parent, child, parent.getWorldTransform().inverse() * jointWorld, childWorld.inverse() * jointWorld);
        // Firm limits (little give, quickly corrected), so joints don't stretch.
        const float flyingScale = mFlyingCorpse ? 1.4f : 1.f;
        joint->setLimit(btRadians(swing1 * flyingScale), btRadians(swing2 * flyingScale),
            btRadians(twist * flyingScale), 0.9f, 0.55f, 0.9f);
        // Brutal Morrowind solves ragdoll joint anchors more tightly than the
        // generic world solver.  In particular, a heavy landing or strong
        // impact must not pull the shoulders and neck away from the torso.
        // Keep a smaller budget for irregular creature rigs and flying corpses
        // to avoid amplifying their overlapping procedural body geometry.
        // Procedural creature rigs need extra constraint resolution under
        // uneven limb masses, but still less than the stable humanoid budget.
        // Flying corpses retain their lighter solver and controlled descent.
        const int jointSolverIterations = mFlyingCorpse ? 28 : (mDisableInternalCollision ? 48 : 100);
        joint->setOverrideNumSolverIterations(jointSolverIterations);
        mTaskScheduler->addConstraint(joint.get());
        mJoints.push_back(std::move(joint));
    }

    void Ragdoll::addJointFriction()
    {
        auto friction = std::make_unique<JointFriction>();
        for (const auto& joint : mJoints)
        {
            friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
            auto* enhanced = static_cast<RagdollEnhancement::ConeTwist*>(joint.get());
            btRigidBody& parent = enhanced->getRigidBodyA();
            btRigidBody& child = enhanced->getRigidBodyB();
            if (parent.getInvMass() <= 0 || child.getInvMass() <= 0)
                continue;
            const btVector3 axis = child.getWorldTransform().getBasis().getColumn(2);
            const btScalar inertia = RagdollEnhancement::effectiveInertia(parent, child, axis);
            const btScalar mass = 1 / child.getInvMass();
            const btScalar lever = btMax(btScalar(8),
                (child.getWorldTransform().getBasis() * enhanced->getBFrame().getOrigin()).length());
            const btScalar gravityTorque = mass * btScalar(627.2) * lever;
            // Same soft-stop width/torque approach as Brutal Morrowind;
            // conservative bound for mixed humanoid and creature rigs.
            RagdollEnhancement::SoftStopSettings settings{
                btRadians(12), gravityTorque * btScalar(.5) / btScalar(20),
                inertia * btScalar(60), gravityTorque, false };
            friction->mEnhanced.push_back({ enhanced, settings });
        }
        mJointFriction = std::move(friction);
        mTaskScheduler->addAction(mJointFriction.get());
    }

    std::optional<osg::Vec3f> Ragdoll::getActorPosition(const osg::Vec3f& actorPosition)
    {
        if (mLastPoses.empty())
            return std::nullopt;
        const osg::Vec3f main = mLastMainBonePosition;
        // Keep the actor following the creature's main mass, not an arbitrary
        // wing bone that may precede the torso in the source mesh skeleton.
        if (!mActorOffset)
            mActorOffset = osg::Vec2f(actorPosition.x() - main.x(), actorPosition.y() - main.y());
        if (mFlyingCorpse)
        {
            if (!mFlyingActorOffsetZ)
                mFlyingActorOffsetZ = actorPosition.z() - main.z();
            return osg::Vec3f(main.x() + mActorOffset->x(), main.y() + mActorOffset->y(),
                main.z() + *mFlyingActorOffsetZ);
        }
        return osg::Vec3f(main.x() + mActorOffset->x(), main.y() + mActorOffset->y(), main.z());
    }

    MWWorld::Ptr Ragdoll::getPtr() const
    {
        return mParts.front()->getPtr();
    }

    bool Ragdoll::hasComeApart(float duration)
    {
        // A bad fit shows straight away; after a few seconds it's settled (or being dragged around).
        constexpr float checkTime = 3.f;
        if (mAge > checkTime || mParts.empty())
            return false;
        mAge += duration;
        mCheckTimer += duration;

        std::vector<btVector3> positions;
        positions.reserve(mParts.size());
        for (const auto& part : mParts)
        {
            btTransform transform;
            part->mMotionState->getWorldTransform(transform);
            positions.push_back(transform.getOrigin());
        }
        if (mLastPositions.empty())
        {
            for (const btVector3& position : positions)
                mStartDistances.push_back(static_cast<float>((position - positions.at(mMainPartIndex)).length()));
            mLastPositions = std::move(positions);
            mCheckTimer = 0;
            return false;
        }
        // Over a while, not every frame: the parts only move each physics step.
        constexpr float checkInterval = 0.25f;
        if (mCheckTimer < checkInterval)
            return false;

        // Faster than anything falls or is thrown, or a joint pulled far apart.
        constexpr float maxSpeed = 2500.f;
        for (size_t i = 0; i < positions.size(); ++i)
        {
            if ((positions[i] - mLastPositions[i]).length() > maxSpeed * mCheckTimer)
                return true;
            if (isJointed()
                && (positions[i] - positions.at(mMainPartIndex)).length()
                    > mStartDistances[i] * 2.f + (mFlyingCorpse ? 250.f : 80.f))
                return true;
        }
        mLastPositions = std::move(positions);
        mCheckTimer = 0;
        return false;
    }

    void Ragdoll::setAtRest()
    {
        // Restored corpses already have their saved pose; do not let them
        // restart a fall or jitter every time a cell is loaded.
        if (mFrozen)
            return;
        std::vector<std::pair<btRigidBody*, btScalar>> bodies;
        bodies.reserve(mParts.size());
        for (const auto& part : mParts)
            bodies.emplace_back(part->mBody, part->mOriginalMass);
        mTaskScheduler->setRagdollBodiesFrozen(bodies, true);
        mFrozen = true;
    }

    void Ragdoll::updateFreezeTimer(float duration, bool beingDragged)
    {
        if (mFrozen || beingDragged || duration <= 0.f || mParts.empty())
            return;

        // Motion-state snapshots are synchronized with the physics simulation.
        // Check translation AND rotation of every limb, not just the pelvis.
        mFreezeAge += duration;
        mFreezeSampleTime += duration;
        constexpr float sampleInterval = 0.25f;
        if (mFreezeSampleTime < sampleInterval)
            return;
        const float sampleDuration = mFreezeSampleTime;
        mFreezeSampleTime = 0.f;

        std::vector<btTransform> current;
        current.reserve(mParts.size());
        for (const auto& part : mParts)
        {
            btTransform transform;
            part->mMotionState->getWorldTransform(transform);
            current.push_back(transform);
        }

        if (mPreviousFreezeTransforms.size() != current.size())
        {
            mPreviousFreezeTransforms = current;
            mJitterStartPositions.clear();
            mJitterWindowTime = 0.f;
            mStableTime = 0.f;
            return;
        }

        // Below roughly 2 world units/s and 0.08 rad/s for EVERY part
        // continuously for 0.5s: a genuinely settled corpse.
        constexpr float maxStableSpeed = 2.f;
        constexpr float maxStableAngularSpeed = 0.08f;
        bool allStill = true;
        for (size_t i = 0; i < current.size(); ++i)
        {
            const btScalar travel = (current[i].getOrigin() - mPreviousFreezeTransforms[i].getOrigin()).length();
            const btScalar rotation = current[i].getRotation().angleShortestPath(
                mPreviousFreezeTransforms[i].getRotation());
            if (travel > maxStableSpeed * sampleDuration || rotation > maxStableAngularSpeed * sampleDuration)
            {
                allStill = false;
                break;
            }
        }
        mStableTime = allStill ? mStableTime + sampleDuration : 0.f;
        mPreviousFreezeTransforms = current;

        // Detect wedged bodies that shake in place but make no net progress.
        // Compare a TWO-SECOND window, so falling or sliding down stairs does
        // not look like stationary high-frequency jitter.
        bool locallyStuck = false;
        if (mJitterStartPositions.size() != current.size())
        {
            mJitterStartPositions.clear();
            for (const btTransform& transform : current)
                mJitterStartPositions.push_back(transform.getOrigin());
            mJitterWindowTime = 0.f;
        }
        else
        {
            mJitterWindowTime += sampleDuration;
            if (mJitterWindowTime >= 2.f)
            {
                const float pelvisTravel = static_cast<float>(
                    (current.at(mMainPartIndex).getOrigin() - mJitterStartPositions.at(mMainPartIndex)).length());
                float totalTravel = 0.f;
                for (size_t i = 0; i < current.size(); ++i)
                    totalTravel += static_cast<float>(
                        (current[i].getOrigin() - mJitterStartPositions[i]).length());
                const float meanTravel = totalTravel / current.size();
                locallyStuck = pelvisTravel < 12.f && meanTravel < 20.f;
                mJitterStartPositions.clear();
                for (const btTransform& transform : current)
                    mJitterStartPositions.push_back(transform.getOrigin());
                mJitterWindowTime = 0.f;
            }
        }

        const bool settled = mFreezeAge >= 3.f && mStableTime >= 0.5f;
        const bool wedged = mFreezeAge >= 6.f && locallyStuck;
        if (!settled && !wedged)
            return;

        // Fix the complete body at once: simply sleeping it is not enough,
        // because joint contacts or terrain penetration can wake it repeatedly.
        std::vector<std::pair<btRigidBody*, btScalar>> bodies;
        bodies.reserve(mParts.size());
        for (const auto& part : mParts)
            bodies.emplace_back(part->mBody, part->mOriginalMass);
        mTaskScheduler->setRagdollBodiesFrozen(bodies, true);
        mFrozen = true;
    }

    void Ragdoll::unfreezeForDragging()
    {
        // Reset the settling and wedging measurements for the next drop.
        mFreezeAge = 0.f;
        mFreezeSampleTime = 0.f;
        mStableTime = 0.f;
        mJitterWindowTime = 0.f;
        mPreviousFreezeTransforms.clear();
        mJitterStartPositions.clear();
        if (!mFrozen)
            return;

        std::vector<std::pair<btRigidBody*, btScalar>> bodies;
        bodies.reserve(mParts.size());
        for (const auto& part : mParts)
            bodies.emplace_back(part->mBody, part->mOriginalMass);
        mTaskScheduler->setRagdollBodiesFrozen(bodies, false);
        // Re-registering Bullet bodies as dynamic restores world gravity. Flyers
        // must immediately resume their custom aerial gravity after being grabbed.
        if (mFlyingCorpse)
            setGravityScale(1.20f);
        mFrozen = false;
    }

    void Ragdoll::restoreFlightGravity()
    {
        if (mFlyingCorpse && !mFrozen)
            setGravityScale(1.20f);
    }

    void Ragdoll::liftStrayParts(const std::function<float(const osg::Vec3f&)>& groundAt, float depth)
    {
        for (const auto& part : mParts)
        {
            btTransform bodyWorld;
            part->mMotionState->getWorldTransform(bodyWorld);
            const osg::Vec3f position = Misc::Convert::toOsg(bodyWorld.getOrigin());
            const float ground = groundAt(position);
            if (position.z() >= ground - depth)
                continue;
            btVector3 aabbMin;
            btVector3 aabbMax;
            part->mShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
            bodyWorld.getOrigin().setZ(ground + aabbMax.length() + 2.f);
            part->mMotionState->setWorldTransform(bodyWorld);
            part->mMotionState->setWorldTransform(bodyWorld); // no interpolation from under the ground
            mTaskScheduler->placeBody(part->mBody, bodyWorld);
        }
    }

    std::optional<osg::Vec3f> Ragdoll::getImpactCenter() const
    {
        if (mParts.empty() || mMainPartIndex >= mParts.size())
            return std::nullopt;
        btTransform bodyWorld;
        mParts[mMainPartIndex]->mMotionState->getWorldTransform(bodyWorld);
        const osg::Vec3f center = Misc::Convert::toOsg(bodyWorld.getOrigin());
        if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()))
            return std::nullopt;
        return center;
    }

    std::optional<std::pair<osg::Vec3f, float>> Ragdoll::getMeleeImpactPoint(
        const osg::Vec3f& from, const osg::Vec3f& to, float padding) const
    {
        const osg::Vec3f segment = to - from;
        const float lengthSquared = segment.length2();
        if (!(lengthSquared > 1e-4f) || !(padding > 0.f))
            return std::nullopt;

        std::optional<std::pair<osg::Vec3f, float>> result;
        float bestDistanceSquared = std::numeric_limits<float>::max();
        for (const auto& part : mParts)
        {
            if (!part->mBody || part->mBody->getInvMass() <= btScalar(0))
                continue;
            btTransform transform;
            part->mMotionState->getWorldTransform(transform);

            btVector3 localCenter;
            btScalar boundingRadius = 0.f;
            part->mShape->getBoundingSphere(localCenter, boundingRadius);
            if (!std::isfinite(boundingRadius) || boundingRadius < btScalar(0))
                continue;
            const osg::Vec3f center = Misc::Convert::toOsg(transform * localCenter);
            if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()))
                continue;

            const float along = std::clamp(((center - from) * segment) / lengthSquared, 0.f, 1.f);
            const osg::Vec3f onSwing = from + segment * along;
            const float distanceSquared = (center - onSwing).length2();
            const float effectiveRadius = padding + std::min(static_cast<float>(boundingRadius), 45.f);
            if (distanceSquared > effectiveRadius * effectiveRadius)
                continue;

            // Prefer an early hit in the swing, then the limb closest to its path.
            if (!result || along < result->second
                || (along == result->second && distanceSquared < bestDistanceSquared))
            {
                result = std::make_pair(center, along);
                bestDistanceSquared = distanceSquared;
            }
        }
        return result;
    }

    void Ragdoll::strike(const osg::Vec3f& point, const osg::Vec3f& impulse)
    {
        const auto finiteVec = [](const osg::Vec3f& v) {
            return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        };
        if (!finiteVec(point) || !finiteVec(impulse))
            return;
        Part* nearest = nullptr;
        float nearestDistance = std::numeric_limits<float>::max();
        for (const auto& part : mParts)
        {
            btTransform bodyWorld;
            part->mMotionState->getWorldTransform(bodyWorld);
            // Choose the part whose *collision surface* is closest to the hit,
            // not the part with the closest centre. Long upper arms and legs
            // otherwise absorb hits meant for the head or torso.
            const btVector3 local = bodyWorld.invXform(Misc::Convert::toBullet(point));
            btScalar distance = (local).length2();
            switch (part->mShape->getShapeType())
            {
                case CAPSULE_SHAPE_PROXYTYPE:
                {
                    const auto& capsule = *static_cast<const btCapsuleShape*>(part->mShape.get());
                    btVector3 axis(0, 0, 0);
                    axis[capsule.getUpAxis()] = btClamped(
                        local[capsule.getUpAxis()], -capsule.getHalfHeight(), capsule.getHalfHeight());
                    const btScalar surfaceDistance = btMax(btScalar(0),
                        (local - axis).length() - capsule.getRadius());
                    distance = surfaceDistance * surfaceDistance;
                    break;
                }
                case BOX_SHAPE_PROXYTYPE:
                {
                    const btVector3 half = static_cast<const btBoxShape*>(part->mShape.get())
                                               ->getHalfExtentsWithMargin();
                    const btVector3 closest(btClamped(local.x(), -half.x(), half.x()),
                        btClamped(local.y(), -half.y(), half.y()), btClamped(local.z(), -half.z(), half.z()));
                    distance = (local - closest).length2();
                    break;
                }
                case SPHERE_SHAPE_PROXYTYPE:
                {
                    const btScalar surfaceDistance = btMax(btScalar(0),
                        local.length() - static_cast<const btSphereShape*>(part->mShape.get())->getRadius());
                    distance = surfaceDistance * surfaceDistance;
                    break;
                }
                default:
                    break;
            }
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearest = part.get();
            }
        }
        if (nearest != nullptr)
            mTaskScheduler->applyRagdollImpulse(
                nearest->mBody, Misc::Convert::toBullet(impulse), Misc::Convert::toBullet(point));
    }

    std::vector<std::shared_ptr<PtrHolder>> Ragdoll::getParts() const
    {
        return { mParts.begin(), mParts.end() };
    }

    float Ragdoll::getMass() const
    {
        float mass = 0;
        for (const auto& part : mParts)
            if (part->mBody->getInvMass() > 0)
                mass += static_cast<float>(1 / part->mBody->getInvMass());
        return mass;
    }

    std::optional<osg::Vec3f> Ragdoll::getGrabPoint(const osg::Vec3f& eye) const
    {
        if (mParts.size() != 1)
            return std::nullopt;
        const Part& part = *mParts.front();
        btTransform bodyWorld;
        part.mMotionState->getWorldTransform(bodyWorld);
        // The nearest point of the box, a little inside so the pin isn't right on the edge.
        btVector3 local = bodyWorld.inverse() * Misc::Convert::toBullet(eye);
        const osg::Vec3f& half = part.mBoxHalfExtents;
        local.setX(std::clamp<btScalar>(local.x(), -half.x() * 0.8f, half.x() * 0.8f));
        local.setY(std::clamp<btScalar>(local.y(), -half.y() * 0.8f, half.y() * 0.8f));
        local.setZ(std::clamp<btScalar>(local.z(), -half.z() * 0.8f, half.z() * 0.8f));
        return Misc::Convert::toOsg(bodyWorld * local);
    }

    Ragdoll::~Ragdoll()
    {
        if (mFlyingFallController != nullptr)
            mTaskScheduler->removeAction(mFlyingFallController.get());
        if (mJointFriction != nullptr)
            mTaskScheduler->removeAction(mJointFriction.get());
        for (const auto& joint : mJoints)
            mTaskScheduler->removeConstraint(joint.get());
        for (const auto& part : mParts)
            mTaskScheduler->removeCollisionObject(part->mBody);
    }

    std::vector<std::pair<std::string, osg::Matrixf>> Ragdoll::getBonePoses(float interpolation) const
    {
        std::vector<std::pair<std::string, osg::Matrixf>> poses;
        poses.reserve(mParts.size());
        for (const auto& part : mParts)
        {
            const btTransform bodyWorld = part->mMotionState->getInterpolatedTransform(interpolation);
            const btTransform bone = bodyWorld * part->mBoneFromBody;
            poses.emplace_back(part->mBone,
                osg::Matrixf::scale(part->mBoneScale) * osg::Matrixf::rotate(Misc::Convert::toOsg(bone.getRotation()))
                    * osg::Matrixf::translate(Misc::Convert::toOsg(bone.getOrigin())));
        }
        if (!poses.empty())
            mLastMainBonePosition = poses.at(mMainPartIndex).second.getTrans();
        if (!mSkinnedBoneBindings.empty())
        {
            std::vector<std::pair<std::string, osg::Matrixf>> allBones;
            allBones.reserve(mSkinnedBoneBindings.size());
            // Parent-first original hierarchy; every tiny wingtip/feelers follows
            // its simulated driver rather than stretching as the body drops.
            for (const auto& binding : mSkinnedBoneBindings)
                allBones.emplace_back(binding.mName,
                    binding.mRelativeToDriver * poses.at(binding.mDriverPart).second);
            poses = std::move(allBones);
        }
        mLastPoses = poses;
        return poses;
    }

    void Ragdoll::updatePtr(const MWWorld::Ptr& updated)
    {
        for (const auto& part : mParts)
            part->updatePtr(updated);
    }

    std::shared_ptr<PtrHolder> Ragdoll::findPart(const osg::Vec3f& eye, const osg::Vec3f& direction) const
    {
        std::shared_ptr<PtrHolder> best;
        float bestDistance = std::numeric_limits<float>::max();
        for (const auto& part : mParts)
        {
            btTransform bodyWorld;
            part->mMotionState->getWorldTransform(bodyWorld);
            const osg::Vec3f toCenter = Misc::Convert::toOsg(bodyWorld.getOrigin()) - eye;
            const float along = std::max(toCenter * direction, 0.f);
            const float distance = (toCenter - direction * along).length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = part;
            }
        }
        return best;
    }
}
