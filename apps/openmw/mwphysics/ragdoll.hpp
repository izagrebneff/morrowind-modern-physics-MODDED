// Derived from sunspotdark/morrowind-modern-physics
// (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40,
// apps/openmw/mwphysics/ragdoll.hpp), GPLv3; see LICENSE.
// Adapted for OpenMW 0.51: standard dead NPCs only, atomic lifecycle and pose snapshots.
#ifndef OPENMW_MWPHYSICS_RAGDOLL_H
#define OPENMW_MWPHYSICS_RAGDOLL_H
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <osg/Matrixf>
#include <osg/Vec3f>
#include <LinearMath/btTransform.h>
#include "ptrholder.hpp"
#include "ragdollstep.hpp"
class btRigidBody;
class btCollisionObject;
class btTypedConstraint;
class btActionInterface;
namespace MWPhysics
{
    class PhysicsTaskScheduler;
    struct CreatureRig;
    class Ragdoll
    {
    public:
        using BoneMatrices = std::map<std::string, osg::Matrixf, std::less<>>;
        using BoneMotion = std::map<std::string, std::pair<osg::Vec3f, osg::Vec3f>, std::less<>>;
        using ExtremityVertices = std::map<std::string, std::vector<osg::Vec3f>, std::less<>>;
        using UnsupportedExtremityBones = std::map<std::string, unsigned, std::less<>>;
        enum class KickProfile { Chest, LowTrip };
        static const std::vector<std::string>& getRequiredBones();
        static BoneMotion sampleAnimationMotion(const BoneMatrices& previous, const BoneMatrices& current,
            const osg::Vec3f& actorTranslation, float duration);
        Ragdoll(const MWWorld::Ptr& actor, const BoneMatrices& bones, const osg::Vec3f& velocity,
            const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler,
            const std::optional<osg::Vec3f>& contact = std::nullopt, const BoneMotion& motion = {},
            const ExtremityVertices& surfaceVertices = {}, const UnsupportedExtremityBones& unsupportedBones = {},
            KickProfile kickProfile = KickProfile::Chest);
        // A creature's body from its planned rig: no anatomical drives, cuts or equipment release.
        Ragdoll(const MWWorld::Ptr& actor, const CreatureRig& rig, const BoneMatrices& bones,
            const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler,
            const std::optional<osg::Vec3f>& contact = std::nullopt, KickProfile kickProfile = KickProfile::Chest);
        ~Ragdoll();
        // Render-only interpolation; omitted alpha uses the scheduler's coherent residual clock.
        // Physical transforms and velocities remain the exact latest completed Bullet step.
        std::vector<std::pair<std::string, osg::Matrixf>> getBonePoses(
            std::optional<float> interpolation = std::nullopt) const;
        std::optional<osg::Vec3f> getActorPosition(const osg::Vec3f& actorPosition);
        const std::vector<std::pair<std::string, osg::Matrixf>>& getLastBonePoses() const { return mLastPoses; }
        void updatePtr(const MWWorld::Ptr& updated);
        MWWorld::Ptr getPtr() const;
        const std::vector<btRigidBody*>& getCollisionBodies() const { return mBodies; }
        std::size_t partCount() const { return mParts.size(); }
        float maxJointError() const;
        std::vector<osg::Vec3f> getDynamics() const;
        float bottom() const;
        float retainedBottom() const;
        void recordLanding(float height, float damage) { ++mLandingCount; mLastFallHeight=height; mLastFallDamage=damage; }
        osg::Vec3f getLandingState() const { return osg::Vec3f(mLandingCount,mLastFallHeight,mLastFallDamage); }
        float initialRotationError() const { return mInitialRotationError; }
        float initialMatrixError() const { return mInitialMatrixError; }
        float speed() const { return mMaxSpeed; }
        float angularSpeed() const { return mMaxAngularSpeed; }
        // Zero cut velocity restores a saved severed state without relaunching.
        bool detachHead(const osg::Vec3f& cutVelocity, const osg::Vec3f& headAngularImpulse);
        bool detachTorso(const osg::Vec3f& cutVelocity, const osg::Vec3f& upperSeparation,
            const std::string& savedPose = {});
        std::string getBisectionPoseData() const;
        bool detachLimbs(unsigned mask, const osg::Vec3f& cutVelocity, const std::string& savedPose = {});
        std::string getLimbSeveringPoseData() const;
        unsigned getSeveredLimbsMask() const { return mSeveredLimbsMask; }
        bool isTorsoDetached() const { return mTorsoDetached; }
        std::vector<osg::Vec3f> getHeadState() const;
        // Nearest unobstructed collision surface in the kick cone, using the
        // current physical pose. Returns its distance, or a negative value.
        float findKickPart(const osg::Vec3f& origin, const osg::Vec3f& forward, float reach, float minimumDot,
            std::size_t& part, osg::Vec3f& point) const;
        bool isDetachedHeadPart(std::size_t part) const;
        // One bounded momentum change for exactly the struck connected fragment.
        bool applyKick(std::size_t part, const osg::Vec3f& velocity, const osg::Vec3f& point);
        int kickCount() const { return mKickCount; }
        std::map<std::string, double> getEnhancementState() const;
        bool applyImpact(std::size_t part, const osg::Vec3f& point, const osg::Vec3f& impulse,
            std::uint64_t id, bool alreadyApplied = false);
        void configureEnhancement();
        void setWater(bool enabled, float height) { mWaterEnabled = enabled; mWaterHeight = height; }
        bool getEquipmentFrame(bool left, osg::Matrixf& frame, osg::Vec3f& linear, osg::Vec3f& angular) const;
        std::vector<btRigidBody*> getEquipmentGripBodies() const;
        bool startSuperKick(std::uint64_t token, std::uint64_t kickId, const osg::Vec3f& velocity,
            const btCollisionObject* attacker);
        std::map<std::string, double> getSuperKickState() const;
        void guardSuperKickSpawn();
    private:
        struct Part;
        static void makeBody(Part& part, const btTransform& bodyWorld, float mass, float thickness);
        void addToWorld();
        void addJointFriction();
        void recordInitialPose(const BoneMatrices& bones);
        void addJoint(size_t parentIndex, size_t childIndex, const osg::Vec3f& position,
            float swing1, float swing2, float twist);
        PhysicsTaskScheduler* mTaskScheduler;
        std::vector<std::unique_ptr<Part>> mParts;
        std::vector<std::unique_ptr<btTypedConstraint>> mJoints;
        std::unique_ptr<btActionInterface> mJointFriction;
        std::vector<btRigidBody*> mBodies;
        std::vector<btTypedConstraint*> mConstraints;
        std::array<btQuaternion, 11> mBoneInBody;
        mutable std::vector<btTransform> mLastTransforms;
        mutable std::vector<std::pair<std::string, osg::Matrixf>> mLastPoses;
        osg::Vec3f mActorOffset;
        osg::Vec3f mInitialRootVelocity, mInheritedVelocity, mImpulse, mContact, mCOM, mCOMVelocity, mInitialCOMVelocity;
        mutable std::vector<btVector3> mLinearVelocities, mAngularVelocities;
        mutable float mMaxSpeed = 0.f, mMaxAngularSpeed = 0.f;
        float mInitialMaxAngularSpeed = 0.f;
        float mInitialPoseError = 0.f, mInitialRotationError=0.f, mInitialMatrixError=0.f;
        float mLandingCount=0.f, mLastFallHeight=0.f, mLastFallDamage=0.f;
        bool mHeadDetached = false;
        bool mTorsoDetached = false;
        unsigned mSeveredLimbsMask = 0;
        std::array<btScalar, 3> mLimbDetachErrors{};
        bool mLimbPoseRestored = false, mLimbPosePayloadRejected = false;
        btScalar mLimbPoseRestoreError = 0;
        std::array<btScalar, 3> mTorsoDetachErrors{};
        bool mTorsoPoseRestored = false, mTorsoPosePayloadRejected = false;
        btScalar mTorsoPoseRestoreError = 0, mTorsoPoseRestoreDisplacement = 0,
            mTorsoPoseRestoreRotation = 0, mTorsoPoseAnchorDistance = 0;
        int mKickCount = 0;
        float mHeadDetachTransformError = 0.f;
        float mHeadDetachLinearVelocityError = 0.f;
        float mHeadDetachAngularVelocityError = 0.f;
        osg::Vec3f mHeadDetachLinearVelocity, mHeadDetachAngularVelocity;
        std::array<osg::Vec3f, 10> mHeadDetachRetainedState{};
        std::array<osg::Vec3f, 8> mHeadDetachAppliedState{};
        std::vector<osg::Vec3f> mHeadDetachPartVelocities;
        UnsupportedExtremityBones mUnsupportedExtremityBones;
        bool mRegistered = false;
        bool mCreature = false;
        float mKickScale = 1.f;
        bool mWaterEnabled = false;
        float mWaterHeight = 0.f;
        std::uint64_t mSuperKickToken = 0;
    };
}
#endif
