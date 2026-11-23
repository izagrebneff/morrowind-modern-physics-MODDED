#ifndef OPENMW_MWPHYSICS_RAGDOLLCONTROL_H
#define OPENMW_MWPHYSICS_RAGDOLLCONTROL_H

#include "ragdollstep.hpp"
#include "ragdollkick.hpp"
#include "ragdollenhancement.hpp"
#include "limbsevering.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <LinearMath/btIDebugDraw.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseInterface.h>
#include <BulletCollision/BroadphaseCollision/btOverlappingPairCache.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

namespace MWPhysics
{
    struct RagdollJointControl
    {
        RagdollEnhancement::ConeTwist* joint = nullptr;
        std::size_t parent = 0, child = 0;
        btTransform baselineA, baselineB, enhancedA, enhancedB;
        btQuaternion neutral{ 0, 0, 0, 1 };
        btVector3 baselineSpans{ 0, 0, 0 }, enhancedSpans{ 0, 0, 0 };
        RagdollEnhancement::SoftStopSettings soft;
        RagdollEnhancement::ToneSettings tone;
    };

    struct RagdollShapeControl
    {
        enum class Kind { Hand, Foot };
        btRigidBody* body = nullptr;
        std::size_t part = 0;
        Kind kind = Kind::Hand;
        btCollisionShape* baseline = nullptr;
        btCollisionShape* enhanced = nullptr;
        btScalar mass = 0;
        btVector3 baselineInertia{ 0, 0, 0 }, enhancedInertia{ 0, 0, 0 };
        std::vector<btVector3> surfaceVertices;
        std::size_t hullVertices = 0;
    };

    // Post-step action retains the original friction/head-separation duties.
    // The scheduler explicitly calls beforeStep while owning its world lock.
    class JointFriction final : public RagdollStepAction
    {
    public:
        std::vector<std::pair<btRigidBody*, btRigidBody*>> mPairs;
        std::vector<btRigidBody*> mBodies;
        std::vector<RagdollJointControl> mLinks;
        std::vector<RagdollShapeControl> mShapes;
        std::vector<std::pair<btRigidBody*, btScalar>> mWaterBodies; // body, density relative to water
        std::vector<btVector3> mStepTorques;
        btRigidBody* mDetachedHead = nullptr;
        std::vector<btRigidBody*> mHeadCollisionPending;
        std::atomic<unsigned int> mHeadCollisionPairsEnabled{ 0 };
        bool mTorsoDetached = false;
        unsigned mSeveredLimbsMask = 0;
        // Populated only before the locked controller handoff, cleared by inherit.
        std::vector<btTypedConstraint*> mRemovedConstraints;
        std::vector<std::pair<std::size_t, std::size_t>> mFragmentCollisionPending;
        std::atomic<unsigned int> mFragmentCollisionPairsEnabled{ 0 };
        RagdollControlConfig mConfig;
        std::uint64_t mGeneration = 0;
        std::uint64_t mSerial = 0;
        std::uint64_t mImpactCount = 0, mDuplicateCount = 0;
        std::uint64_t mInvalidCount = 0, mImpactCapHits = 0;
        std::uint64_t mDampingCapHits = 0;
        btScalar mMinDampingScale = 1;
        double mSimulatedAge = 0;
        btVector3 mLastImpulse{ 0, 0, 0 }, mLastPrimaryAngularImpulse{ 0, 0, 0 };
        btScalar mLastProceduralH = 0, mLastImpactScale = 1;
        std::uint64_t mFragmentKickCount = 0;
        btScalar mLastKickMass = 0, mLastKickLocalSpeed = 0;
        btVector3 mLastKickVelocity{ 0, 0, 0 }, mLastKickPoint{ 0, 0, 0 };
        std::size_t mLastKickPart = 0;
        btScalar mMaxSoftTorque = 0, mMaxToneTorque = 0, mMaxAddedAngularSpeed = 0;
        btScalar mMaxSelfPenetration = 0, mCurrentSelfPenetration = 0;
        btScalar mInitialAngularViolation = 0;
        std::array<std::uint64_t, 64> mSeen{};
        std::size_t mSeenCursor = 0;
        bool mConfigured = false;
        bool mCollisionFiltersDirty = false;
        std::vector<std::pair<std::size_t, std::size_t>> mCollisionPairs, mPendingCollisionPairs;

        bool sameFragment(std::size_t a, std::size_t b) const
        {
            if (mDetachedHead && ((mBodies[a] == mDetachedHead) != (mBodies[b] == mDetachedHead))) return false;
            if (mTorsoDetached && ((a >= 1 && a <= 6) != (b >= 1 && b <= 6))) return false;
            return LimbSevering::fragment(a, mSeveredLimbsMask) == LimbSevering::fragment(b, mSeveredLimbsMask);
        }

        void allowCollision(btRigidBody* a, btRigidBody* b, bool enabled)
        {
            // Bullet's ignore list is not a set; don't insert duplicate exclusions.
            if (a->checkCollideWith(b) != enabled)
            { a->setIgnoreCollisionCheck(b, !enabled); mCollisionFiltersDirty = true; }
            if (b->checkCollideWith(a) != enabled)
            { b->setIgnoreCollisionCheck(a, !enabled); mCollisionFiltersDirty = true; }
        }

        void flushCollisionFilters(btCollisionWorld* world)
        {
            if (!world || !mCollisionFiltersDirty) return;
            // Changing an ignore flag does not remove Bullet's cached contact
            // manifold. Without this cleanup the solver can keep applying its
            // old contacts even after the pair has been disabled.
            auto* cache = world->getBroadphase()->getOverlappingPairCache();
            for (auto* body : mBodies)
                if (auto* proxy = body->getBroadphaseHandle())
                    cache->cleanProxyFromPairs(proxy, world->getDispatcher());
            mCollisionFiltersDirty = false;
        }

        static btScalar separation(btRigidBody* a, btRigidBody* b)
        {
            btVoronoiSimplexSolver simplex;
            btGjkEpaPenetrationDepthSolver penetration;
            btGjkPairDetector detector(static_cast<btConvexShape*>(a->getCollisionShape()),
                static_cast<btConvexShape*>(b->getCollisionShape()), &simplex, &penetration);
            btDiscreteCollisionDetectorInterface::ClosestPointInput input;
            input.m_transformA = a->getWorldTransform();
            input.m_transformB = b->getWorldTransform();
            btPointCollector result;
            detector.getClosestPoints(input, result, nullptr);
            return result.m_hasResult ? result.m_distance : BT_LARGE_FLOAT;
        }

        void configure(const RagdollControlConfig& config, btCollisionWorld* world) override
        {
            if (!std::isfinite(config.softStrength) || !std::isfinite(config.toneStrength)
                || !std::isfinite(config.impactStrength) || config.softStrength < 0
                || config.toneStrength < 0 || config.impactStrength < 0)
            {
                ++mInvalidCount;
                throw std::runtime_error("Ragdoll strengths must be finite and nonnegative");
            }
            const bool shapesChanged = !mConfigured || config.enabled != mConfig.enabled;
            const bool changed = !mConfigured || config.enabled != mConfig.enabled
                || config.softStops != mConfig.softStops || config.selfCollision != mConfig.selfCollision;
            const bool wake = !mConfigured || changed || config.neutralPose != mConfig.neutralPose
                || config.limbResponse != mConfig.limbResponse || config.softStrength != mConfig.softStrength
                || config.toneStrength != mConfig.toneStrength || config.impactStrength != mConfig.impactStrength;
            mConfig = config;
            if (!changed)
            {
                flushCollisionFilters(world);
                if (wake) for (auto* body : mBodies) body->activate(true);
                return;
            }
            if (shapesChanged)
                for (const auto& shape : mShapes)
                {
                    auto* selected = config.enabled ? shape.enhanced : shape.baseline;
                    if (shape.body->getCollisionShape() == selected)
                        continue;
                    shape.body->setCollisionShape(selected);
                    shape.body->setMassProps(shape.mass,
                        config.enabled ? shape.enhancedInertia : shape.baselineInertia);
                    shape.body->updateInertiaTensor();
                    if (world && shape.body->getBroadphaseHandle())
                        world->updateSingleAabb(shape.body);
                    // The existing cleanup also drops cached algorithms/manifolds
                    // using the old shape. configure owns the scheduler's world lock.
                    mCollisionFiltersDirty = true;
                }
            for (auto& link : mLinks)
            {
                const auto& spans = config.enabled ? link.enhancedSpans : link.baselineSpans;
                link.joint->setFrames(config.enabled ? link.enhancedA : link.baselineA,
                    config.enabled ? link.enhancedB : link.baselineB);
                link.joint->setLimit(spans.x(), spans.y(), spans.z(),
                    config.enabled && config.softStops ? btScalar(1) : btScalar(.9),
                    config.enabled ? btScalar(.15) : btScalar(.6), btScalar(1));
                // Positional joint recovery is a real velocity bias in Bullet.
                // Correct supported anchor drift over several substeps instead
                // of turning a small floor correction into a launch velocity.
                for (int axis = 0; axis < 3; ++axis)
                    link.joint->setParam(BT_CONSTRAINT_ERP, config.enabled ? btScalar(.25) : btScalar(.9), axis);
                if (config.enabled && link.joint->limitViolation(link.neutral) > btScalar(1e-3))
                    throw std::runtime_error("Ragdoll neutral target is outside its anatomical joint limits");
                mInitialAngularViolation = btMax(mInitialAngularViolation, link.joint->limitViolation(link.joint->relativeRotation()));
            }
            for (auto* body : mBodies)
                body->setRestitution(config.enabled ? btScalar(0) : btScalar(.05));
            mCollisionPairs.clear();
            mPendingCollisionPairs.clear();
            for (std::size_t a = 0; a < mBodies.size(); ++a)
                for (std::size_t b = a + 1; b < mBodies.size(); ++b)
                {
                    if (mBodies[a] == mDetachedHead || mBodies[b] == mDetachedHead) continue;
                    if (!sameFragment(a, b)) continue; // Independently rearmed after physical clearance.
                    allowCollision(mBodies[a], mBodies[b], false);
                    if (!config.enabled || !config.selfCollision || !collisionCandidate(a, b)) continue;
                    // Start contacts only from non-overlapping proxies. Already-folded
                    // animation poses are handled by anatomical limits, not ejection.
                    if (separation(mBodies[a], mBodies[b]) >= btScalar(-.15))
                    {
                        allowCollision(mBodies[a], mBodies[b], true);
                        mCollisionPairs.emplace_back(a, b);
                    }
                    else mPendingCollisionPairs.emplace_back(a, b);
                }
            mConfigured = true;
            flushCollisionFilters(world);
            // A deliberate toggle can wake an island once; error at rest never does.
            for (auto* body : mBodies) body->activate(true);
        }

        int solverIterations() const override { return mConfig.enabled ? 120 : 20; }

        void beforeStep(btScalar dt) override
        {
            if (!std::isfinite(dt) || dt <= 0) { ++mInvalidCount; return; }
            // The physical clock also advances with the enhancement disabled or
            // asleep. Render updates and configuration changes do not advance it.
            mSimulatedAge += dt;
            applyWater(dt);
            if (!mConfig.enabled) return;
            // Evaluate every link from the same pre-step velocities, preserving
            // the degree-four damping bound and independence of link order.
            mStepTorques.assign(mBodies.size(), btVector3(0, 0, 0));
            for (auto& link : mLinks)
            {
                auto& a = *mBodies[link.parent];
                auto& b = *mBodies[link.child];
                if (!a.isActive() && !b.isActive()) continue;
                RagdollEnhancement::TorqueComponents softParts, toneParts;
                if (mConfig.softStops)
                {
                    auto soft = link.soft;
                    soft.springAtLimit *= mConfig.softStrength;
                    soft.damping *= mConfig.softStrength;
                    soft.maxDamping *= mConfig.softStrength;
                    link.joint->softStopTorqueWorld(soft, &softParts);
                }
                if (mConfig.neutralPose)
                {
                    auto tone = link.tone;
                    tone.stiffness *= mConfig.toneStrength;
                    tone.damping *= mConfig.toneStrength;
                    tone.maxSpring *= mConfig.toneStrength;
                    tone.maxDamping *= mConfig.toneStrength;
                    link.joint->neutralTorqueWorld(link.neutral, tone, &toneParts);
                }
                const auto spring = softParts.spring + toneParts.spring;
                const auto requestedDamping = softParts.damping + toneParts.damping;
                if (!RagdollEnhancement::finite(spring) || !RagdollEnhancement::finite(requestedDamping))
                { ++mInvalidCount; continue; }
                btScalar dampingScale;
                const auto damping = RagdollEnhancement::boundedDampingTorqueWorld(requestedDamping,
                    b.getAngularVelocity() - a.getAngularVelocity(),
                    a.getInvInertiaTensorWorld() + b.getInvInertiaTensorWorld(), dt, &dampingScale);
                if (dampingScale < 1)
                {
                    ++mDampingCapHits;
                    mMinDampingScale = btMin(mMinDampingScale, dampingScale);
                }
                mMaxSoftTorque = btMax(mMaxSoftTorque, (softParts.spring + softParts.damping * dampingScale).length());
                mMaxToneTorque = btMax(mMaxToneTorque, (toneParts.spring + toneParts.damping * dampingScale).length());
                const auto torque = spring + damping;
                if (!RagdollEnhancement::finite(torque)) { ++mInvalidCount; continue; }
                mStepTorques[link.child] += torque;
                mStepTorques[link.parent] -= torque;
            }
            // Bullet 3.24 omits external torque from friction-row RHS setup.
            // applyTorque would therefore leave a tau*dt/I slip even when the
            // contact has enough static friction to hold. Integrate the paired
            // moments exactly once here, so all contact rows see their velocity.
            // Subsequent native body damping also damps this increment.
            for (std::size_t i = 0; i < mBodies.size(); ++i)
                mBodies[i]->applyTorqueImpulse(mStepTorques[i] * dt);
        }

        bool impact(const RagdollImpact& event) override
        {
            using namespace RagdollEnhancement;
            if (!mConfig.enabled) return false;
            if (event.part >= mBodies.size() || !event.id || !finite(event.point)
                || !finite(event.impulse) || event.impulse.length2() > btScalar(200000) * 200000)
            { ++mInvalidCount; return false; }
            if (std::find(mSeen.begin(), mSeen.end(), event.id) != mSeen.end())
            { ++mDuplicateCount; return false; }
            btRigidBody* hit = mBodies[event.part];
            const btVector3 arm = event.point - hit->getCenterOfMassPosition();
            // Match Bullet's point-impulse factors, but validate before mutating
            // either velocity. Finite inputs alone do not prevent cross-product
            // overflow for a malformed point far outside the struck body.
            const btVector3 primaryH = arm.cross(event.impulse * hit->getLinearFactor());
            const btVector3 deltaLinear = event.impulse * hit->getLinearFactor() * hit->getInvMass();
            const btVector3 deltaAngular = (hit->getInvInertiaTensorWorld() * primaryH) * hit->getAngularFactor();
            const btVector3 nextLinear = hit->getLinearVelocity() + deltaLinear;
            const btVector3 nextAngular = hit->getAngularVelocity() + deltaAngular;
            if (!finite(arm) || !finite(primaryH) || !finite(primaryH.length2())
                || (!event.alreadyApplied && (!finite(deltaLinear) || !finite(deltaAngular)
                    || !finite(nextLinear) || !finite(nextAngular)
                    || !finite(nextLinear.length2()) || !finite(nextAngular.length2()))))
            { ++mInvalidCount; return false; }
            mSeen[mSeenCursor++ % mSeen.size()] = event.id;
            mLastImpulse = event.impulse;
            mLastPrimaryAngularImpulse = primaryH;
            mLastProceduralH = 0;
            mLastImpactScale = 1;
            if (!event.alreadyApplied)
                hit->applyImpulse(event.impulse, arm);
            for (std::size_t i = 0; i < mBodies.size(); ++i)
                if (sameFragment(event.part, i)) mBodies[i]->activate(true);
            ++mImpactCount;
            if (!mConfig.limbResponse || hit == mDetachedHead) return true;
            std::vector<AngularImpulse> impulses;
            impulses.reserve(5);
            std::uint64_t random = mGeneration * 0x9e3779b97f4a7c15ULL ^ event.id;
            for (const auto& link : mLinks)
            {
                if (link.child != 3 && link.child != 5 && link.child != 7 && link.child != 9) continue;
                if (!sameFragment(event.part, link.child)) continue;
                auto* child = mBodies[link.child];
                auto* parent = mBodies[link.parent];
                const auto pivot = child->getWorldTransform() * link.joint->getBFrame().getOrigin();
                btVector3 lever = child->getCenterOfMassPosition() - pivot;
                if (!finite(lever)) { ++mInvalidCount; continue; }
                if (lever.length2() < btScalar(.01)) continue;
                random ^= random >> 12; random ^= random << 25; random ^= random >> 27;
                const btScalar variation = btScalar(.9) + btScalar(.2) * btScalar(random & 0xffff) / 65535;
                const btScalar gain = btScalar(.012) * mConfig.impactStrength * variation;
                const btVector3 candidate = lever.cross(-event.impulse) * gain;
                if (!finite(candidate)) { ++mInvalidCount; continue; }
                const btVector3 raw = link.joint->allowedTorqueWorld(candidate);
                if (!finite(raw.length2())) { ++mInvalidCount; continue; }
                if (raw.length2() > btScalar(1800) * 1800) ++mImpactCapHits;
                const btVector3 h = clampMagnitude(raw, btScalar(1800));
                mLastProceduralH += h.length();
                accumulateAngularImpulse(impulses, child, h, btScalar(.8));
                accumulateAngularImpulse(impulses, parent, -h, btScalar(.5));
            }
            const btScalar scale = commonImpulseScale(impulses);
            mLastImpactScale = scale;
            mLastProceduralH *= scale; // Sum of joint |H|, before cancelling parent pairs.
            if (scale < btScalar(1)) ++mImpactCapHits;
            for (const auto& entry : impulses)
            {
                const auto h = entry.impulse * scale;
                mMaxAddedAngularSpeed = btMax(mMaxAddedAngularSpeed,
                    (entry.body->getInvInertiaTensorWorld() * h).length());
                entry.body->applyTorqueImpulse(h);
            }
            return true;
        }

        bool kick(std::size_t part, const btVector3& point, const btVector3& velocity,
            std::uint64_t id) override
        {
            if (part >= mBodies.size() || !id) { ++mInvalidCount; return false; }
            std::vector<btRigidBody*> component;
            for (std::size_t i = 0; i < mBodies.size(); ++i)
                if (sameFragment(part, i)) component.push_back(mBodies[i]);
            const auto plan = RagdollKick::prepare(component, mBodies[part], point, velocity);
            if (!plan) { ++mInvalidCount; return false; }
            // The ordinary point-impact owner records/deduplicates the event and
            // adds its bounded joint response. Primary momentum is committed below
            // once, under the same scheduler lock, through the fragment plan.
            if (mConfig.enabled)
            {
                if (!impact({ id, part, point, plan->totalImpulse, true })) return false;
            }
            else
            {
                if (std::find(mSeen.begin(), mSeen.end(), id) != mSeen.end())
                { ++mDuplicateCount; return false; }
                mSeen[mSeenCursor++ % mSeen.size()] = id;
            }
            plan->apply();
            mLastPrimaryAngularImpulse = plan->pointAngularImpulse;
            mLastKickMass = plan->mass;
            mLastKickLocalSpeed = plan->pointImpulse.length() * mBodies[part]->getInvMass();
            mLastKickVelocity = velocity; mLastKickPoint = point; mLastKickPart = part;
            ++mFragmentKickCount;
            return true;
        }

        void inherit(const RagdollStepAction& previous, btTypedConstraint* removed,
            btCollisionWorld* world = nullptr) override
        {
            // Called exclusively inside the scheduler's detach transaction.
            const auto& old = static_cast<const JointFriction&>(previous);
            mBodies = old.mBodies; mConfig = old.mConfig; mGeneration = old.mGeneration;
            mWaterBodies = old.mWaterBodies;
            mShapes = old.mShapes; // Part owns both shapes across the head-cut controller handoff.
            mSerial = old.mSerial; mSeen = old.mSeen; mSeenCursor = old.mSeenCursor;
            mImpactCount = old.mImpactCount; mDuplicateCount = old.mDuplicateCount;
            mInvalidCount = old.mInvalidCount; mImpactCapHits = old.mImpactCapHits;
            mDampingCapHits = old.mDampingCapHits; mMinDampingScale = old.mMinDampingScale;
            mLastImpulse = old.mLastImpulse; mLastPrimaryAngularImpulse = old.mLastPrimaryAngularImpulse;
            mLastProceduralH = old.mLastProceduralH; mLastImpactScale = old.mLastImpactScale;
            mSimulatedAge = old.mSimulatedAge;
            mFragmentKickCount = old.mFragmentKickCount;
            mLastKickMass = old.mLastKickMass; mLastKickLocalSpeed = old.mLastKickLocalSpeed;
            mLastKickVelocity = old.mLastKickVelocity; mLastKickPoint = old.mLastKickPoint;
            mLastKickPart = old.mLastKickPart;
            mCollisionFiltersDirty = old.mCollisionFiltersDirty;
            mMaxSoftTorque = old.mMaxSoftTorque; mMaxToneTorque = old.mMaxToneTorque;
            mMaxAddedAngularSpeed = old.mMaxAddedAngularSpeed;
            for (const auto& link : old.mLinks)
                if (link.joint != removed && std::find(mRemovedConstraints.begin(), mRemovedConstraints.end(),
                        link.joint) == mRemovedConstraints.end()) mLinks.push_back(link);
            mRemovedConstraints.clear();
            if (mDetachedHead)
                for (auto* body : mBodies)
                    if (body != mDetachedHead) allowCollision(mDetachedHead, body, false);
            if (mTorsoDetached || mSeveredLimbsMask)
            {
                // The surviving links/shapes already have their configured frames
                // and inertia. Do not run a throwable reconfiguration after commit.
                mConfigured = old.mConfigured;
                mCurrentSelfPenetration = old.mCurrentSelfPenetration;
                mMaxSelfPenetration = old.mMaxSelfPenetration;
                for (const auto& pair : old.mCollisionPairs)
                    if (sameFragment(pair.first, pair.second)) mCollisionPairs.push_back(pair);
                for (const auto& pair : old.mPendingCollisionPairs)
                    if (sameFragment(pair.first, pair.second)) mPendingCollisionPairs.push_back(pair);
                for (std::size_t a = 0; a < mBodies.size(); ++a)
                    for (std::size_t b = a + 1; b < mBodies.size(); ++b)
                        if (!sameFragment(a, b))
                        {
                            allowCollision(mBodies[a], mBodies[b], false);
                            mFragmentCollisionPending.emplace_back(a, b);
                        }
            }
            // Keep head-contact rearming atomic with the locked detach transaction.
            flushCollisionFilters(world);
        }

        // Lift by the submerged share of each part, and the thick drag that stops a body in water.
        void applyWater(btScalar dt)
        {
            if (!mConfig.waterEnabled || dt <= 0) return;
            constexpr btScalar gravity = 627.2f, linearDrag = 2.2f, angularDrag = 3.f;
            for (const auto& [body, density] : mWaterBodies)
            {
                if (!body->isActive()) continue;
                btVector3 lo, hi;
                body->getCollisionShape()->getAabb(body->getWorldTransform(), lo, hi);
                const btScalar height = hi.z() - lo.z();
                const btScalar submerged = height > 1e-3f
                    ? btClamped((btScalar(mConfig.waterHeight) - lo.z()) / height, btScalar(0), btScalar(1)) : btScalar(0);
                if (submerged <= 0) continue;
                body->applyCentralImpulse(btVector3(0, 0, gravity * dt * submerged / (density * body->getInvMass())));
                body->setLinearVelocity(body->getLinearVelocity() * btExp(-linearDrag * submerged * dt));
                body->setAngularVelocity(body->getAngularVelocity() * btExp(-angularDrag * submerged * dt));
            }
        }

        void updateAction(btCollisionWorld* world, btScalar dt) override
        {
            for (auto it = mFragmentCollisionPending.begin(); it != mFragmentCollisionPending.end();)
            {
                const auto [a, b] = *it;
                const auto distance = separation(mBodies[a], mBodies[b]);
                if (!std::isfinite(distance) || distance >= BT_LARGE_FLOAT || distance < btScalar(.01))
                { ++it; continue; }
                allowCollision(mBodies[a], mBodies[b], true);
                ++mFragmentCollisionPairsEnabled;
                it = mFragmentCollisionPending.erase(it);
            }
            flushCollisionFilters(world);
            if (mDetachedHead)
                for (auto it = mHeadCollisionPending.begin(); it != mHeadCollisionPending.end();)
                {
                    bool clear = false;
                    if (mConfig.enabled) clear = separation(mDetachedHead, *it) >= btScalar(.01);
                    else
                    {
                        // Without the enhancement: conservative AABB gate.
                        btVector3 amin, amax, bmin, bmax;
                        mDetachedHead->getAabb(amin, amax); (*it)->getAabb(bmin, bmax);
                        for (int axis = 0; axis < 3; ++axis)
                            clear = clear || amin[axis] > bmax[axis] + btScalar(.01)
                                || bmin[axis] > amax[axis] + btScalar(.01);
                    }
                    if (!clear) { ++it; continue; }
                    allowCollision(mDetachedHead, *it, true);
                    mHeadCollisionPairsEnabled.fetch_add(1, std::memory_order_relaxed);
                    it = mHeadCollisionPending.erase(it);
                }
            if (mConfig.enabled)
            {
                for (auto it = mPendingCollisionPairs.begin(); it != mPendingCollisionPairs.end();)
                {
                    auto [a, b] = *it;
                    if (separation(mBodies[a], mBodies[b]) < btScalar(.15)) { ++it; continue; }
                    allowCollision(mBodies[a], mBodies[b], true);
                    mCollisionPairs.push_back(*it);
                    it = mPendingCollisionPairs.erase(it);
                }
                flushCollisionFilters(world);
                mCurrentSelfPenetration = 0;
                for (auto [a, b] : mCollisionPairs)
                    mCurrentSelfPenetration = btMax(mCurrentSelfPenetration, -separation(mBodies[a], mBodies[b]));
                // An excluded overlapping candidate is still a possible visual
                // defect. Diagnostics must not hide it behind a zero contact depth.
                for (auto [a, b] : mPendingCollisionPairs)
                    mCurrentSelfPenetration = btMax(mCurrentSelfPenetration, -separation(mBodies[a], mBodies[b]));
                mMaxSelfPenetration = btMax(mMaxSelfPenetration, mCurrentSelfPenetration);
                return;
            }
            flushCollisionFilters(world);
            // Standard ragdoll friction, mass-weighted on the angular part.
            const btScalar fraction = 1 - std::exp(btScalar(-14) * dt);
            for (const auto& [a, b] : mPairs)
            {
                if (!a->isActive() && !b->isActive()) continue;
                const auto relative = b->getAngularVelocity() - a->getAngularVelocity();
                if (relative.length2() < btScalar(1e-4)) continue;
                const auto ma = a->getInvMass() > 0 ? 1 / a->getInvMass() : 0;
                const auto mb = b->getInvMass() > 0 ? 1 / b->getInvMass() : 0;
                if (ma + mb <= 0) continue;
                const auto change = relative * fraction;
                a->setAngularVelocity(a->getAngularVelocity() + change * (mb / (ma + mb)));
                b->setAngularVelocity(b->getAngularVelocity() - change * (ma / (ma + mb)));
            }
        }

        std::map<std::string, double> snapshot() const override
        {
            unsigned active = 0;
            for (auto* body : mBodies) active += body->isActive();
            std::map<std::string, double> result{ {"enabled", mConfig.enabled}, {"softStops", mConfig.enabled && mConfig.softStops},
                {"neutralPose", mConfig.enabled && mConfig.neutralPose}, {"limbResponse", mConfig.enabled && mConfig.limbResponse},
                {"selfCollision", mConfig.enabled && mConfig.selfCollision}, {"generation", double(mGeneration)},
                {"impactCount", double(mImpactCount)}, {"duplicateCount", double(mDuplicateCount)},
                {"simulatedAge", mSimulatedAge},
                {"fragmentKickCount", double(mFragmentKickCount)}, {"lastKickMass", mLastKickMass},
                {"lastKickPart", double(mLastKickPart)}, {"lastKickLocalSpeed", mLastKickLocalSpeed},
                {"lastKickVelocityX", mLastKickVelocity.x()}, {"lastKickVelocityY", mLastKickVelocity.y()},
                {"lastKickVelocityZ", mLastKickVelocity.z()}, {"lastKickPointX", mLastKickPoint.x()},
                {"lastKickPointY", mLastKickPoint.y()}, {"lastKickPointZ", mLastKickPoint.z()},
                {"fragmentCollisionPairsEnabled", mFragmentCollisionPairsEnabled.load(std::memory_order_relaxed)},
                {"fragmentCollisionPairsPending", double(mFragmentCollisionPending.size())},
                {"limbsMask", mSeveredLimbsMask},
                {"invalidCount", double(mInvalidCount)}, {"impactCapHits", double(mImpactCapHits)},
                {"dampingCapHits", double(mDampingCapHits)}, {"minDampingScale", mMinDampingScale},
                {"lastImpulseX", mLastImpulse.x()}, {"lastImpulseY", mLastImpulse.y()},
                {"lastImpulseZ", mLastImpulse.z()}, {"lastImpulseMagnitude", mLastImpulse.length()},
                {"lastPrimaryAngularImpulseMagnitude", mLastPrimaryAngularImpulse.length()},
                {"lastProceduralAngularImpulse", mLastProceduralH}, {"lastImpactScale", mLastImpactScale},
                {"activeJoints", double(mLinks.size())}, {"collisionPairs", double(mCollisionPairs.size())},
                {"pendingCollisionPairs", double(mPendingCollisionPairs.size())}, {"maxSoftTorque", mMaxSoftTorque},
                {"maxToneTorque", mMaxToneTorque}, {"maxAddedAngularSpeed", mMaxAddedAngularSpeed},
                {"maxSelfPenetration", mMaxSelfPenetration}, {"selfPenetration", mCurrentSelfPenetration},
                {"initialAngularViolation", mInitialAngularViolation}, {"activeBodies", active} };
            unsigned availableHandProxies = 0, activeHandProxies = 0;
            unsigned availableFootProxies = 0, activeFootProxies = 0;
            for (const auto& shape : mShapes)
            {
                const bool enhanced = shape.body->getCollisionShape() == shape.enhanced;
                const bool foot = shape.kind == RagdollShapeControl::Kind::Foot;
                if (foot) { ++availableFootProxies; activeFootProxies += enhanced; }
                else { ++availableHandProxies; activeHandProxies += enhanced; }
                const std::string prefix = std::string(foot ? "foot" : "hand") + std::to_string(shape.part) + ".";
                result[prefix + "active"] = enhanced;
                result[prefix + "baselineShape"] = shape.body->getCollisionShape() == shape.baseline;
                result[prefix + "sourceVertices"] = shape.surfaceVertices.size();
                result[prefix + "hullVertices"] = shape.hullVertices;
                // Project the frozen activation surface through the latest physical
                // body pose. These are not interpolated render-vertex measurements.
                btVector3 meshLowest(0, 0, BT_LARGE_FLOAT);
                for (const auto& vertex : shape.surfaceVertices)
                {
                    const auto world = shape.body->getWorldTransform() * vertex;
                    if (world.z() < meshLowest.z())
                        meshLowest = world;
                }
                const auto& bodyWorld = shape.body->getWorldTransform();
                const btVector3 localDown = bodyWorld.getBasis().transpose() * btVector3(0, 0, -1);
                // A rotated cached local AABB can extend well below a convex
                // surface. Query the actual support point, including its margin.
                const auto support = bodyWorld * static_cast<const btConvexShape*>(
                    shape.body->getCollisionShape())->localGetSupportingVertex(localDown);
                result[prefix + "physicalMeshLowestX"] = meshLowest.x();
                result[prefix + "physicalMeshLowestY"] = meshLowest.y();
                result[prefix + "physicalMeshLowestZ"] = meshLowest.z();
                result[prefix + "physicalProxyBottomZ"] = support.z();
                const auto& expectedInertia = enhanced ? shape.enhancedInertia : shape.baselineInertia;
                const btVector3 expectedInverse(1 / expectedInertia.x(), 1 / expectedInertia.y(),
                    1 / expectedInertia.z());
                result[prefix + "localInertiaError"]
                    = (shape.body->getInvInertiaDiagLocal() - expectedInverse).length();
            }
            result["availableHandProxies"] = availableHandProxies;
            result["activeHandProxies"] = activeHandProxies;
            result["availableFootProxies"] = availableFootProxies;
            result["activeFootProxies"] = activeFootProxies;
            for (const auto& link : mLinks)
            {
                const std::string prefix = "joint" + std::to_string(link.child) + ".";
                const auto current = link.joint->relativeRotation();
                const auto rotation = RagdollEnhancement::rotationVector(current);
                const auto neutral = RagdollEnhancement::rotationVector(link.neutral);
                const auto frame = link.joint->getRigidBodyA().getWorldTransform() * link.joint->getAFrame();
                result[prefix + "rotationX"] = rotation.x();
                result[prefix + "rotationY"] = rotation.y();
                result[prefix + "rotationZ"] = rotation.z();
                result[prefix + "neutralX"] = neutral.x();
                result[prefix + "neutralY"] = neutral.y();
                result[prefix + "neutralZ"] = neutral.z();
                result[prefix + "neutralError"] = RagdollEnhancement::rotationVector(link.neutral * current.inverse()).length();
                result[prefix + "hardSwing1"] = link.joint->getSwingSpan1();
                result[prefix + "hardSwing2"] = link.joint->getSwingSpan2();
                result[prefix + "hardTwist"] = link.joint->getTwistSpan();
                result[prefix + "softWidth"] = mConfig.enabled && mConfig.softStops ? link.soft.widthRad : 0;
                for (int axis = 0; axis < 3; ++axis)
                    result[prefix + "allowedAxis" + std::to_string(axis)] =
                        link.joint->allowedTorqueWorld(frame.getBasis().getColumn(axis)).length2() > btScalar(.5);
            }
            return result;
        }

        void debugDraw(btIDebugDraw* draw) override
        {
            if (!draw || !mConfig.enabled) return;
            for (const auto& link : mLinks)
            {
                const auto frame = link.joint->getRigidBodyA().getWorldTransform() * link.joint->getAFrame();
                for (int axis = 0; axis < 3; ++axis)
                {
                    btVector3 color(0, 0, 0); color[axis] = 1;
                    draw->drawLine(frame.getOrigin(), frame.getOrigin() + frame.getBasis().getColumn(axis) * 8, color);
                }
            }
        }

    private:
        static bool collisionCandidate(std::size_t a, std::size_t b)
        {
            // Keep overlapping proximal shoulder/hip proxies excluded. Explicit
            // head/chest contact is necessary even though they share a joint.
            if (a == 2 || b == 2) return true;
            if ((a <= 1 && (b == 4 || b == 6 || b == 8 || b == 10))) return true;
            if ((a == 3 || a == 4) && (b == 5 || b == 6 || b >= 7)) return true;
            if ((a == 5 || a == 6) && b >= 7) return true;
            return (a == 7 || a == 8) && (b == 9 || b == 10);
        }
    };
}
#endif
