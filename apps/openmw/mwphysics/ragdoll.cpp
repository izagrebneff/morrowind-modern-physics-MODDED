#include <components/misc/brutalconstants.hpp>
// Derived from sunspotdark/morrowind-modern-physics
// (https://github.com/sunspotdark/morrowind-modern-physics, commit eb50a40,
// apps/openmw/mwphysics/ragdoll.cpp), GPLv3; see LICENSE.
// Adapted for OpenMW 0.51: standard dead NPCs only, atomic lifecycle and pose snapshots.
#include "ragdoll.hpp"
#include "creaturerig.hpp"
#include <components/esm3/loadcrea.hpp>
#include "../mwworld/livecellref.hpp"
#include "ragdollcontrol.hpp"
#include "ragdollneutral.hpp"
#include "ragdollmotion.hpp"
#include "limbsevering.hpp"
#include <components/settings/values.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/Dynamics/btActionInterface.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btMotionState.h>

#include <osg/Quat>

#include <components/misc/convert.hpp>

#include "collisiontype.hpp"
#include "mtphysics.hpp"

namespace MWPhysics
{
    struct Ragdoll::Part final : public PtrHolder
    {
        explicit Part(const MWWorld::Ptr& ptr)
            : PtrHolder(ptr, osg::Vec3f())
        {
        }

        std::string mBone;
        std::unique_ptr<btCollisionShape> mShape;
        std::unique_ptr<btCollisionShape> mEnhancedShape;
        std::vector<btVector3> mExtremityVertices;
        btRigidBody* mBody = nullptr; // owned by mCollisionObject

        void setBody(std::unique_ptr<btRigidBody> body)
        {
            mBody = body.get();
            mCollisionObject = std::move(body);
        }
        osg::Matrixf mRenderFromBody; // full affine bone offset, including stretch/shear
        osg::Vec3f mBoxHalfExtents; // for a single box body
        float mRelativeDensity = 1.05f; // to water: a creature's body sinks slowly (people's: see sPartDefs)

        ~Part() override { mCollisionObject.reset(); }
    };

    namespace
    {
        // Compact fragment persistence deliberately excludes scale, shear, and
        // individual limb poses. The actor's rigid reference survives reload;
        // its pelvis pose does not, so pelvis-relative offsets would turn a
        // horizontal gap into a vertical one when the lower half is rebuilt.
        constexpr std::size_t BisectionPoseBytes = 4 + 14 * sizeof(float);
        constexpr btScalar BisectionPoseMaximumOffset = 131072;

        std::optional<btTransform> bisectionActorFrame(const MWWorld::Ptr& ptr)
        {
            const auto* root = ptr.getRefData().getBaseNode();
            if (!root) return {};
            const auto& p = root->getPosition();
            auto q = Misc::Convert::toBullet(root->getAttitude());
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())
                || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z())
                || !std::isfinite(q.w()) || !std::isfinite(q.length2()) || q.length2() < btScalar(1e-12))
                return {};
            q.normalize();
            return btTransform(q, Misc::Convert::toBullet(p));
        }

        bool validBisectionAnchor(const btVector3& p, const btQuaternion& q)
        {
            return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z())
                && btFabs(p.x()) <= BisectionPoseMaximumOffset && btFabs(p.y()) <= BisectionPoseMaximumOffset
                && btFabs(p.z()) <= BisectionPoseMaximumOffset && std::isfinite(q.x()) && std::isfinite(q.y())
                && std::isfinite(q.z()) && std::isfinite(q.w()) && std::isfinite(q.length2())
                && btFabs(q.length2() - 1) <= btScalar(.001);
        }

        void appendBisectionFloat(std::string& out, float value)
        {
            const auto bits = std::bit_cast<std::uint32_t>(value);
            for (unsigned shift = 0; shift < 32; shift += 8)
                out.push_back(static_cast<char>((bits >> shift) & 0xff));
        }

        std::optional<std::array<btTransform, 2>> readBisectionPose(
            std::string_view bytes, const btTransform& actorFrame)
        {
            if (bytes.size() != BisectionPoseBytes || bytes.substr(0, 4) != "BSP1") return {};
            std::size_t cursor = 4;
            const auto read = [&]() {
                std::uint32_t bits = 0;
                for (unsigned shift = 0; shift < 32; shift += 8)
                    bits |= std::uint32_t(static_cast<unsigned char>(bytes[cursor++])) << shift;
                return std::bit_cast<float>(bits);
            };
            std::array<btTransform, 2> world;
            for (auto& anchor : world)
            {
                // Explicit ordering: argument evaluation order is unspecified.
                const float x = read(), y = read(), z = read();
                const float qx = read(), qy = read(), qz = read(), qw = read();
                const btVector3 position(x, y, z);
                btQuaternion rotation(qx, qy, qz, qw);
                if (!validBisectionAnchor(position, rotation)) return {};
                rotation.normalize();
                anchor = actorFrame * btTransform(rotation, position);
            }
            return world;
        }

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

        constexpr std::size_t sHeadPart = 2;
        // How strongly a chest kick reaches each part along the joint chain (sPartDefs order):
        // the chest leads, its neighbours follow, the ends of the limbs lag the most.
        constexpr float sChestKickShare[] = { .75f, 1.f, .75f, .75f, .55f, .75f, .55f, .55f, .45f, .55f, .45f };
        static_assert(std::size(sChestKickShare) == std::size(sPartDefs));
        static_assert(sPartDefs[sHeadPart].mBone == "Bip01 Head");

        std::unique_ptr<btConvexHullShape> makeExtremityShape(const btCapsuleShape& capsule,
            const btTransform& bodyWorld, const std::vector<osg::Vec3f>& worldVertices,
            std::vector<btVector3>& surfaceVertices)
        {
            if (worldVertices.empty())
                return {};
            std::vector<btVector3> local;
            local.reserve(worldVertices.size());
            const btTransform worldToBody = bodyWorld.inverse();
            for (const auto& world : worldVertices)
            {
                const btVector3 vertex = worldToBody * Misc::Convert::toBullet(world);
                if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y()) || !std::isfinite(vertex.z()))
                    return {};
                local.push_back(vertex);
            }
            auto hull = std::make_unique<btConvexHullShape>();
            hull->setMargin(btScalar(.15)); // Native game units, outside the sampled hand surface.
            constexpr int longitudes = 16, latitudes = 8;
            // A nearest grid vertex has directional support at least
            // cos(pi/longitudes)*cos(pi/(2*latitudes)). Inflating by its inverse
            // makes the polytope contain the sphere, rather than cut into it.
            // Translating its copies to both cap centres also contains the full
            // original capsule. At radii 4/5 the excess is at most .159/.198 units.
            const btScalar radius = capsule.getRadius()
                / (btCos(SIMD_PI / longitudes) * btCos(SIMD_PI / (2 * latitudes)));
            for (int endpoint : { -1, 1 })
                for (int latitude = 0; latitude <= latitudes; ++latitude)
                {
                    const btScalar angle = SIMD_PI * latitude / latitudes;
                    const btScalar y = latitude == 0 ? btScalar(1)
                        : latitude == latitudes ? btScalar(-1) : btCos(angle);
                    const btScalar ring = latitude == 0 || latitude == latitudes ? btScalar(0) : btSin(angle);
                    const int count = ring == 0 ? 1 : longitudes;
                    for (int longitude = 0; longitude < count; ++longitude)
                    {
                        const btScalar around = SIMD_2_PI * longitude / longitudes;
                        hull->addPoint(btVector3(radius * ring * btCos(around),
                            endpoint * capsule.getHalfHeight() + radius * y,
                            radius * ring * btSin(around)), false);
                    }
                }
            for (const auto& vertex : local)
                hull->addPoint(vertex, false);
            hull->optimizeConvexHull();
            hull->recalcLocalAabb();
            surfaceVertices = std::move(local);
            return hull;
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

    Ragdoll::BoneMotion Ragdoll::sampleAnimationMotion(const BoneMatrices& previous, const BoneMatrices& current,
        const osg::Vec3f& actorTranslation, float duration)
    {
        const auto center = [](const BoneMatrices& bones, const PartDef& def) {
            const osg::Vec3f start = bones.at(std::string(def.mBone)).getTrans();
            if (!def.mEnd.empty()) return (start + bones.at(std::string(def.mEnd)).getTrans()) * .5f;
            auto up = start - bones.at("Bip01 Neck").getTrans();
            if (up.normalize() < 1e-3f) up = osg::Vec3f(0, 0, 1);
            return start + up * 7.f;
        };
        BoneMotion result;
        for (const auto& def : sPartDefs)
        {
            osg::Vec3f p, scale; osg::Quat oldRotation, rotation, stretch;
            previous.at(std::string(def.mBone)).decompose(p, oldRotation, scale, stretch);
            current.at(std::string(def.mBone)).decompose(p, rotation, scale, stretch);
            const auto motion = RagdollEnhancement::animationMotion(Misc::Convert::toBullet(center(previous, def)),
                Misc::Convert::toBullet(center(current, def)), Misc::Convert::toBullet(oldRotation),
                Misc::Convert::toBullet(rotation), Misc::Convert::toBullet(actorTranslation), duration);
            result.emplace(def.mBone, std::pair(Misc::Convert::toOsg(motion.linear), Misc::Convert::toOsg(motion.angular)));
        }
        return result;
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const BoneMatrices& bones, const osg::Vec3f& velocity,
        const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler, const std::optional<osg::Vec3f>& contact,
        const BoneMotion& motion, const ExtremityVertices& surfaceVertices,
        const UnsupportedExtremityBones& unsupportedBones, KickProfile kickProfile)
        : mTaskScheduler(scheduler), mInitialRootVelocity(velocity), mInheritedVelocity(velocity),
          mUnsupportedExtremityBones(unsupportedBones)
    {
        for (const auto& name : getRequiredBones())
        {
            const auto found = bones.find(name);
            if (found == bones.end())
                throw std::runtime_error("Ragdoll requires standard NPC bone: " + name);
            for (int i = 0; i < 16; ++i)
                if (!std::isfinite(found->second.ptr()[i]))
                    throw std::runtime_error("Ragdoll bone matrix is not finite");
        }
        mActorOffset = actor.getRefData().getPosition().asVec3() - bones.at("Bip01 Pelvis").getTrans();
        const auto position = [&](std::string_view bone) { return bones.find(bone)->second.getTrans(); };
        osg::Vec3f animatedMomentum;
        float totalAnimatedMass = 0.f;

        for (const PartDef& def : sPartDefs)
        {
            auto part = std::make_unique<Part>(actor);
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
            if (def.mBone == "Bip01 L Forearm" || def.mBone == "Bip01 R Forearm"
                || def.mBone == "Bip01 L Calf" || def.mBone == "Bip01 R Calf")
                if (const auto found = surfaceVertices.find(def.mEnd); found != surfaceVertices.end())
                    part->mEnhancedShape = makeExtremityShape(*static_cast<btCapsuleShape*>(part->mShape.get()),
                        bodyWorld, found->second, part->mExtremityVertices);
            const osg::Matrixf bodyMatrix=osg::Matrixf::rotate(Misc::Convert::toOsg(bodyWorld.getRotation()))
                *osg::Matrixf::translate(Misc::Convert::toOsg(bodyWorld.getOrigin()));
            part->mRenderFromBody=bones.find(def.mBone)->second*osg::Matrixf::inverse(bodyMatrix);

            part->mRelativeDensity = def.mDensity;
            makeBody(*part, bodyWorld, def.mMass, def.mRadius);

            // Falling as it was moving, the upper body thrown back by the killing blow.
            btVector3 partVelocity = Misc::Convert::toBullet(velocity);
            if (const auto it = motion.find(part->mBone); it != motion.end())
            {
                partVelocity += Misc::Convert::toBullet(it->second.first);
                part->mBody->setAngularVelocity(Misc::Convert::toBullet(it->second.second));
                animatedMomentum += it->second.first * def.mMass;
            }
            totalAnimatedMass += def.mMass;
            if (!contact && (def.mBone == "Bip01 Spine1" || def.mBone == "Bip01 Head"))
                partVelocity += Misc::Convert::toBullet(kick);
            else if (!contact && def.mBone == "Bip01 Pelvis")
                partVelocity += Misc::Convert::toBullet(kick) * 0.6f;
            part->mBody->setLinearVelocity(partVelocity);

            mParts.push_back(std::move(part));
        }
        mInheritedVelocity += animatedMomentum / totalAnimatedMass;
        if (contact)
        {
            // Apply one physical kick at contact without changing the initial
            // pose or actor reference. Its total linear impulse is mass * kick.
            float totalMass = 0.f;
            for (const auto& part : mParts) totalMass += 1.f / part->mBody->getInvMass();
            mImpulse = kick * totalMass;
            mContact = *contact;
            // The whole impulse on the light chest alone (about 5x the kick and 60 rad/s in one
            // step) separated the neck and elbows by up to 11 units before the joints caught
            // up. Share it down the chain instead: the chest leads, neighbours follow with less,
            // so adjacent parts never differ enough to tear a joint. Momentum is unchanged.
            // A low trip instead shares translation uniformly. The low contact's
            // bounded angular field makes the legs lead and the torso lag while
            // every joint starts with compatible rigid-body motion.
            const auto kickShare = [&](std::size_t i) {
                return kickProfile == KickProfile::LowTrip ? 1.f : sChestKickShare[i];
            };
            btScalar weighted = 0;
            for (std::size_t i = 0; i < mParts.size(); ++i)
                weighted += kickShare(i) / mParts[i]->mBody->getInvMass();
            const btVector3 lead = Misc::Convert::toBullet(mImpulse) / weighted;
            // The off-centre contact tips the body over as a whole, at a tumble's pace.
            btVector3 com(0, 0, 0);
            for (const auto& part : mParts)
                com += part->mBody->getCenterOfMassPosition() / part->mBody->getInvMass();
            com /= totalMass;
            btMatrix3x3 inertia(0, 0, 0, 0, 0, 0, 0, 0, 0);
            for (const auto& part : mParts)
            {
                const btRigidBody* body = part->mBody;
                const btVector3 local = body->getInvInertiaDiagLocal();
                const btMatrix3x3 diag(1 / local.x(), 0, 0, 0, 1 / local.y(), 0, 0, 0, 1 / local.z());
                const btMatrix3x3& basis = body->getWorldTransform().getBasis();
                const btVector3 d = body->getCenterOfMassPosition() - com;
                const btScalar m = 1 / body->getInvMass(), dd = d.dot(d);
                inertia = inertia + basis * diag * basis.transpose()
                    + btMatrix3x3(dd - d.x() * d.x(), -d.x() * d.y(), -d.x() * d.z(), -d.y() * d.x(),
                          dd - d.y() * d.y(), -d.y() * d.z(), -d.z() * d.x(), -d.z() * d.y(), dd - d.z() * d.z())
                        * m;
            }
            btVector3 spin = inertia.inverse() * (Misc::Convert::toBullet(*contact) - com).cross(Misc::Convert::toBullet(mImpulse));
            btVector3 trip = lead;
            if (kickProfile == KickProfile::LowTrip)
            {
                // A sliding kick at the shins: the whole body is thrown forward,
                // faster than the decelerating slide, so it stays ahead of the low
                // camera and lands in front of the player. The feet are swept a
                // little faster than the torso, tipping the airborne body onto its
                // back; a small side roll varies the landing.
                constexpr btScalar comGain = 1.3f, sweep = .35f, maxTripSpin = 3.5f; // rad/s
                const btVector3 kickVelocity = Misc::Convert::toBullet(kick);
                btVector3 along(kickVelocity.x(), kickVelocity.y(), 0);
                const btScalar speed = along.length();
                spin.setZero();
                if (speed > 1.f)
                {
                    along /= speed;
                    const btVector3 axis = (Misc::Convert::toBullet(*contact) - com).cross(along);
                    const btScalar arm = axis.length(); // contact height below the centre of mass
                    if (arm > 5.f)
                    {
                        const btScalar pitch = std::min(maxTripSpin, speed * sweep / arm);
                        const btVector3 side = btVector3(0, 0, 1).cross(along);
                        btScalar offset = (Misc::Convert::toBullet(*contact) - com).dot(side);
                        static std::atomic<unsigned> alternate{ 0 };
                        if (std::abs(offset) < 1.f)
                            offset = (alternate++ & 1) ? 1.f : -1.f;
                        spin = axis / arm * pitch + along * (offset > 0 ? .4f : -.4f) * pitch;
                    }
                    trip = along * (speed * comGain) + btVector3(0, 0, kickVelocity.z());
                }
                mImpulse = Misc::Convert::toOsg(trip) * totalMass;
            }
            const btScalar maxSpin = kickProfile == KickProfile::LowTrip ? 4.f : 4.f; // rad/s
            if (spin.length() > maxSpin)
                spin *= maxSpin / spin.length();
            for (std::size_t i = 0; i < mParts.size(); ++i)
            {
                btRigidBody* body = mParts[i]->mBody;
                body->setLinearVelocity(body->getLinearVelocity() + trip * kickShare(i)
                    + spin.cross(body->getCenterOfMassPosition() - com));
                body->setAngularVelocity(body->getAngularVelocity() + spin);
            }

        }
        // The profile is anatomical; retain each visual bone's fixed local offset
        // from its physical capsule so an arbitrary death pose is not the target.
        for (std::size_t i = 0; i < mParts.size(); ++i)
        {
            osg::Vec3f bonePosition, scale; osg::Quat rotation, stretch;
            bones.at(mParts[i]->mBone).decompose(bonePosition, rotation, scale, stretch);
            mBoneInBody[i] = mParts[i]->mBody->getWorldTransform().getRotation().inverse()
                * Misc::Convert::toBullet(rotation);
            mBoneInBody[i].normalize();
            if (!RagdollEnhancement::validRotation(mBoneInBody[i]))
                throw std::runtime_error("Invalid ragdoll anatomical bone frame");
        }
        mJointFriction = std::make_unique<JointFriction>();
        static std::atomic<std::uint64_t> generation{ 0 };
        static_cast<JointFriction*>(mJointFriction.get())->mGeneration = ++generation;
        // Joints where each part meets its parent, twisting around the part's own bone.
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            const PartDef& def = sPartDefs[i];
            if (def.mParent >= 0)
                addJoint(def.mParent, i, position(def.mBone), def.mSwing1, def.mSwing2, def.mTwist);
        }
        addJointFriction();
        addToWorld();
        try { configureEnhancement(); }
        catch (...) { mTaskScheduler->removeRagdollBodies(mBodies, mConstraints, mJointFriction.get()); mRegistered = false; throw; }
        if (Settings::game().mRagdollEnhancement && kick.length2() > 0)
        {
            // Compatibility start/ledge paths already own their velocity change.
            // Inform the controller once, adding only its bounded limb response.
            const osg::Vec3f point = contact.value_or(Misc::Convert::toOsg(mParts[1]->mBody->getCenterOfMassPosition()));
            const std::size_t part = kickProfile == KickProfile::LowTrip
                ? std::numeric_limits<std::size_t>::max() : 1;
            applyImpact(part, point, contact ? mImpulse : kick * 27.2f, 0, true);
        }
        recordInitialPose(bones);
    }

    void Ragdoll::recordInitialPose(const BoneMatrices& bones)
    {
        getBonePoses(1.f);
        mInitialCOMVelocity=getDynamics()[1]; // measured from all Bullet masses, before their first step
        mInitialMaxAngularSpeed = mMaxAngularSpeed;
        for (const auto& [bone, pose] : mLastPoses)
        {
            const auto& source=bones.at(bone);
            mInitialPoseError = std::max(mInitialPoseError, float((pose.getTrans()-source.getTrans()).length()));
            osg::Vec3f pa,pb,sa,sb;osg::Quat a,b,soa,sob;
            pose.decompose(pa,a,sa,soa);source.decompose(pb,b,sb,sob);
            for (int i=0;i<16;++i) mInitialMatrixError=std::max(mInitialMatrixError,std::abs(pose.ptr()[i]-source.ptr()[i]));
            const double dot=std::abs(a.x()*b.x()+a.y()*b.y()+a.z()*b.z()+a.w()*b.w())/(a.length()*b.length());
            mInitialRotationError=std::max(mInitialRotationError,float(2.*std::acos(std::clamp(dot,0.,1.))*57.295779513));
        }
    }

    Ragdoll::Ragdoll(const MWWorld::Ptr& actor, const CreatureRig& rig, const BoneMatrices& bones,
        const osg::Vec3f& velocity, const osg::Vec3f& kick, PhysicsTaskScheduler* scheduler,
        const std::optional<osg::Vec3f>& contact, KickProfile kickProfile)
        : mTaskScheduler(scheduler), mInitialRootVelocity(velocity), mInheritedVelocity(velocity), mCreature(true)
    {
        for (const auto& def : rig.mParts)
        {
            const auto found = bones.find(def.mBone);
            if (found == bones.end())
                throw std::runtime_error("Creature ragdoll requires bone: " + def.mBone);
            for (int i = 0; i < 16; ++i)
                if (!std::isfinite(found->second.ptr()[i]))
                    throw std::runtime_error("Ragdoll bone matrix is not finite");
        }
        const auto boneAt = [&](const CreatureRigPart& def) -> const osg::Matrixf& { return bones.find(def.mBone)->second; };
        mActorOffset = actor.getRefData().getPosition().asVec3() - boneAt(rig.mParts.front()).getTrans();
        std::vector<int> depth;
        float totalMass = 0.f;
        // What a dead body of this kind does in water: metal and bare bone go down,
        // a gasbag or a fish comes up, flesh hangs near the surface.
        const auto* record = actor.get<ESM::Creature>()->mBase;
        const int flags = record->mFlags;
        // The record's blood type is its material: 1 is dust and bone, 2 is metal sparks.
        const float density = record->mBloodType == 2 ? 2.5f
            : record->mBloodType == 1 ? 1.3f
            : (flags & ESM::Creature::Flies) ? .7f
            : ((flags & ESM::Creature::Swims) && !(flags & ESM::Creature::Walks)) ? .9f
            : (flags & ESM::Creature::Bipedal) ? .95f : 1.05f;
        for (const auto& def : rig.mParts)
        {
            const osg::Matrixf& bone = boneAt(def);
            // The bone already carries the reference's scale and the mesh's own.
            const osg::Vec3f scale = bone.getScale();
            const float size = (scale.x() + scale.y() + scale.z()) / 3.f;
            if (!std::isfinite(size) || size <= 1e-3f)
                throw std::runtime_error("Invalid creature bone scale: " + def.mBone);
            const osg::Vec3f center = def.mCenter * bone;
            osg::Vec3f axisWorld = osg::Matrixf::transform3x3(def.mAxis, bone);
            if (axisWorld.normalize() < 1e-6f)
                axisWorld = osg::Vec3f(0, 0, 1);
            auto part = std::make_unique<Part>(actor);
            part->mBone = def.mBone;
            const btTransform bodyWorld(shortestArcQuat(btVector3(0, 1, 0), Misc::Convert::toBullet(axisWorld)),
                Misc::Convert::toBullet(center));
            const float radius = def.mRadius * size;
            part->mShape = std::make_unique<btCapsuleShape>(radius, std::max(2.f * def.mHalfLength * size, .1f));
            const osg::Matrixf bodyMatrix = osg::Matrixf::rotate(Misc::Convert::toOsg(bodyWorld.getRotation()))
                * osg::Matrixf::translate(Misc::Convert::toOsg(bodyWorld.getOrigin()));
            part->mRenderFromBody = bone * osg::Matrixf::inverse(bodyMatrix);
            const float mass = def.mMass * std::pow(size, 2.25f);
            part->mRelativeDensity = density;
            makeBody(*part, bodyWorld, mass, radius);
            if (def.mHalfLength > def.mRadius * 6.f)
            {
                // A long tentacle is one stiff piece: if its tip grips the floor, a netch's
                // corpse stands on its tentacles as on stilts. Let the tips slide out instead.
                part->mBody->setFriction(.03f);
                part->mBody->setRollingFriction(0.f);
                part->mBody->setSpinningFriction(0.f);
            }
            part->mBody->setLinearVelocity(Misc::Convert::toBullet(velocity));
            totalMass += mass;
            depth.push_back(def.mParent < 0 ? 0 : depth[def.mParent] + 1);
            mParts.push_back(std::move(part));
        }
        mKickScale = std::clamp(std::sqrt(64.f / std::max(totalMass, 1.f)), .25f, 1.f);
        // The chain answers a kick as the NPC body does: the core leads, the ends of the limbs lag.
        const auto kickShare = [&](std::size_t i) {
            if (kickProfile == KickProfile::LowTrip) return 1.f;
            return depth[i] <= 1 ? 1.f : depth[i] == 2 ? .75f : .55f;
        };
        if (kick.length2() > 0.f)
        {
            btScalar weighted = 0;
            btVector3 com(0, 0, 0);
            for (std::size_t i = 0; i < mParts.size(); ++i)
            {
                const btScalar mass = 1 / mParts[i]->mBody->getInvMass();
                weighted += kickShare(i) * mass;
                com += mParts[i]->mBody->getCenterOfMassPosition() * mass;
            }
            com /= totalMass;
            btVector3 lead = Misc::Convert::toBullet(kick) * totalMass / weighted;
            btVector3 spin(0, 0, 0);
            if (contact)
            {
                mImpulse = kick * totalMass;
                mContact = *contact;
                btVector3 along(kick.x(), kick.y(), 0);
                const btScalar speed = along.length();
                const btVector3 lever = Misc::Convert::toBullet(*contact) - com;
                if (speed > 1.f)
                {
                    along /= speed;
                    if (kickProfile == KickProfile::LowTrip)
                        lead = along * (speed * btScalar(1.3)) + btVector3(0, 0, kick.z());
                    const btVector3 axis = lever.cross(along);
                    // A tumble's pace, whatever the size of the body.
                    if (axis.length() > 1.f)
                        spin = axis.normalized() * btMin(btScalar(3.5), speed * btScalar(.35) / btMax(lever.length(), btScalar(5)));
                }
            }
            for (std::size_t i = 0; i < mParts.size(); ++i)
            {
                btRigidBody* body = mParts[i]->mBody;
                body->setLinearVelocity(body->getLinearVelocity() + lead * kickShare(i)
                    + spin.cross(body->getCenterOfMassPosition() - com));
                body->setAngularVelocity(body->getAngularVelocity() + spin);
            }
        }
        mJointFriction = std::make_unique<JointFriction>();
        static std::atomic<std::uint64_t> generation{ 0 };
        static_cast<JointFriction*>(mJointFriction.get())->mGeneration = (std::uint64_t(1) << 62) | ++generation;
        for (std::size_t i = 0; i < mParts.size(); ++i)
        {
            const auto& def = rig.mParts[i];
            if (def.mParent < 0)
                continue;
            btRigidBody& parent = *mParts[def.mParent]->mBody;
            btRigidBody& child = *mParts[i]->mBody;
            const btTransform& childWorld = child.getWorldTransform();
            const btVector3 childAxis = childWorld.getBasis() * btVector3(0, 1, 0);
            const btTransform jointWorld(shortestArcQuat(btVector3(1, 0, 0), childAxis),
                Misc::Convert::toBullet(boneAt(def).getTrans()));
            auto joint = std::make_unique<RagdollEnhancement::ConeTwist>(
                parent, child, parent.getWorldTransform().inverse() * jointWorld, childWorld.inverse() * jointWorld);
            joint->setLimit(btRadians(def.mSwing1), btRadians(def.mSwing2), btRadians(def.mTwist), .9f, .6f, 1.f);
            joint->setOverrideNumSolverIterations(120);
            for (int axis = 0; axis < 3; ++axis)
            {
                joint->setParam(BT_CONSTRAINT_ERP, .9f, axis);
                joint->setParam(BT_CONSTRAINT_CFM, 0.f, axis);
            }
            mJoints.push_back(std::move(joint));
        }
        addJointFriction();
        addToWorld();
        try { configureEnhancement(); }
        catch (...) { mTaskScheduler->removeRagdollBodies(mBodies, mConstraints, mJointFriction.get()); mRegistered = false; throw; }
        recordInitialPose(bones);
    }

    void Ragdoll::makeBody(Part& part, const btTransform& bodyWorld, float mass, float thickness)
    {
        // Choose the activation shape before computing any off-centre impulse.
        auto* shape = Settings::game().mRagdollEnhancement && part.mEnhancedShape
            ? part.mEnhancedShape.get() : part.mShape.get();
        btVector3 inertia(0, 0, 0);
        shape->calculateLocalInertia(mass, inertia);
        btRigidBody::btRigidBodyConstructionInfo info(mass, nullptr, shape, inertia);
        info.m_friction = 0.8f;
        info.m_restitution = 0.05f;
        info.m_linearDamping = 0.15f;
        info.m_angularDamping = 0.7f;
        // In game units (~70 per meter), like the other simulated objects.
        info.m_linearSleepingThreshold = 4.f;
        info.m_angularSleepingThreshold = 1.f;
        auto body = std::make_unique<btRigidBody>(info);
        body->setWorldTransform(bodyWorld);
        body->setInterpolationWorldTransform(bodyWorld);
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
        // The parts don't collide with each other: they overlap at the joints.
        for (size_t i = 0; i < mParts.size(); ++i)
            for (size_t j = i + 1; j < mParts.size(); ++j)
            {
                mParts[i]->mBody->setIgnoreCollisionCheck(mParts[j]->mBody, true);
                mParts[j]->mBody->setIgnoreCollisionCheck(mParts[i]->mBody, true);
            }
        for (const auto& part : mParts)
            mBodies.push_back(part->mBody);
        for (const auto& joint : mJoints)
            mConstraints.push_back(joint.get());
        mTaskScheduler->addRagdollBodies(mBodies, mConstraints, mJointFriction.get());
        mRegistered = true;
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
        joint->setLimit(btRadians(swing1), btRadians(swing2), btRadians(twist), 0.9f, 0.6f, 1.f);
        // A hard landing hits a few parts first; the world's 20 iterations left the others
        // pulled up to 9 units from their joints for a few steps. Solve these to convergence.
        joint->setOverrideNumSolverIterations(100);
        // A kick on the chest must travel through the whole chain within the step that
        // delivers it: the world's 20 iterations left the neck and shoulders pulled apart
        // by up to 9 units while limbs caught up. Solve the joints to convergence and pull
        // a separated anchor back at once, without softness.
        joint->setOverrideNumSolverIterations(120);
        for (int axis = 0; axis < 3; ++axis)
        {
            joint->setParam(BT_CONSTRAINT_ERP, 0.9f, axis);
            joint->setParam(BT_CONSTRAINT_CFM, 0.f, axis);
        }
        RagdollJointControl control;
        control.joint = joint.get(); control.parent = parentIndex; control.child = childIndex;
        control.baselineA = joint->getAFrame(); control.baselineB = joint->getBFrame();
        control.baselineSpans = btVector3(btRadians(swing1), btRadians(swing2), btRadians(twist));
        control.enhancedSpans = control.baselineSpans;
        if (childIndex == 3 || childIndex == 5) control.enhancedSpans.setY(btRadians(55));
        const RagdollNeutralProfile pose;
        const btQuaternion neutralParent = pose.neutral[parentIndex] * mBoneInBody[parentIndex].inverse();
        control.enhancedA = btTransform(neutralParent.inverse() * pose.limitCenter[childIndex],
            control.baselineA.getOrigin());
        control.enhancedB = btTransform(mBoneInBody[childIndex], control.baselineB.getOrigin());
        control.neutral = pose.limitCenter[childIndex].inverse() * pose.neutral[childIndex];
        const btVector3 axis = child.getWorldTransform().getBasis().getColumn(2);
        const btScalar inertia = RagdollEnhancement::effectiveInertia(parent, child, axis);
        const btScalar mass = 1 / child.getInvMass();
        const btScalar lever = btMax(btScalar(8), (child.getCenterOfMassPosition()
            - Misc::Convert::toBullet(position)).length());
        const btScalar gravityTorque = mass * btScalar(627.2) * lever;
        // Near-critical boundary damping, still smoothly gated by penetration
        // and independently capped. A 12*I setting stored too much rebound.
        // Bounds for the SUM of cone/twist potential world gradients and guard.
        // Update these with profile spans, soft width or guard changes; the fixed
        // normalization preserves a conservative potential and caps |spring| at .5*mgL.
        constexpr btScalar gradientBounds[] = { 0, 13, 12, 13, 20, 13, 20, 14, 20, 14, 20 };
        control.soft = { btRadians(12), gravityTorque * btScalar(.5) / gradientBounds[childIndex], inertia * 60,
            gravityTorque, false };
        const btScalar frequency = btScalar(.55) * SIMD_2_PI;
        control.tone = { inertia * frequency * frequency, 4 * inertia * frequency,
            gravityTorque * btScalar(.06), gravityTorque * btScalar(.30) };
        static_cast<JointFriction*>(mJointFriction.get())->mLinks.push_back(control);
        mJoints.push_back(std::move(joint));
    }

    void Ragdoll::addJointFriction()
    {
        auto* friction = static_cast<JointFriction*>(mJointFriction.get());
        friction->mBodies = mBodies;
        // mBodies is populated on registration; parts are already available here.
        for (const auto& part : mParts)
        {
            friction->mBodies.push_back(part->mBody);
            friction->mWaterBodies.emplace_back(part->mBody, part->mRelativeDensity);
        }
        for (std::size_t i = 0; i < mParts.size(); ++i)
            if (const auto& part = mParts[i]; part->mEnhancedShape)
            {
                RagdollShapeControl shape;
                shape.body = part->mBody;
                shape.part = i;
                shape.kind = i == 8 || i == 10 ? RagdollShapeControl::Kind::Foot : RagdollShapeControl::Kind::Hand;
                shape.baseline = part->mShape.get();
                shape.enhanced = part->mEnhancedShape.get();
                shape.mass = 1 / part->mBody->getInvMass();
                shape.baseline->calculateLocalInertia(shape.mass, shape.baselineInertia);
                shape.enhanced->calculateLocalInertia(shape.mass, shape.enhancedInertia);
                shape.surfaceVertices = part->mExtremityVertices;
                shape.hullVertices = static_cast<btConvexHullShape*>(shape.enhanced)->getNumPoints();
                friction->mShapes.push_back(std::move(shape));
            }
        for (const auto& joint : mJoints)
            friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
    }

    Ragdoll::~Ragdoll()
    {
        if (mRegistered)
            mTaskScheduler->removeRagdollBodies(mBodies, mConstraints, mJointFriction.get());
    }

    bool Ragdoll::detachHead(const osg::Vec3f& cutVelocity, const osg::Vec3f& headAngularImpulse)
    {
        if (mTorsoDetached || mSeveredLimbsMask) return false;
        if (mHeadDetached)
            return true;
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(cutVelocity[axis]) || !std::isfinite(headAngularImpulse[axis]))
                return false;
        if (!mRegistered || mParts.size() <= sHeadPart)
            return false;
        btRigidBody* head = mParts[sHeadPart]->mBody;
        const auto found = std::find_if(mJoints.begin(), mJoints.end(), [head](const auto& joint) {
            return &joint->getRigidBodyA() == head || &joint->getRigidBodyB() == head;
        });
        if (found == mJoints.end())
            return false;
        btTypedConstraint* removed = found->get();
        auto friction = std::make_unique<JointFriction>();
        for (const auto& joint : mJoints)
            if (joint.get() != removed)
                friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
        friction->mDetachedHead = head;
        for (btRigidBody* body : mBodies)
            if (body != head)
                friction->mHeadCollisionPending.push_back(body);

        mTaskScheduler->cancelSuperKick(mSuperKickToken);
        const auto snapshot = mTaskScheduler->detachRagdollConstraint(
            removed, mJointFriction.get(), friction.get(), head, mBodies,
            Misc::Convert::toBullet(cutVelocity), Misc::Convert::toBullet(headAngularImpulse));
        mConstraints.erase(std::remove(mConstraints.begin(), mConstraints.end(), removed), mConstraints.end());
        mJoints.erase(found);
        mJointFriction = std::move(friction);
        mHeadDetached = true;
        configureEnhancement();

        const auto renderMatrix = [&](const btTransform& body) {
            return mParts[sHeadPart]->mRenderFromBody
                * osg::Matrixf::rotate(Misc::Convert::toOsg(body.getRotation()))
                * osg::Matrixf::translate(Misc::Convert::toOsg(body.getOrigin()));
        };
        const osg::Matrixf before = renderMatrix(snapshot.before);
        const osg::Matrixf after = renderMatrix(snapshot.after);
        for (int i = 0; i < 16; ++i)
            mHeadDetachTransformError = std::max(mHeadDetachTransformError, std::abs(before.ptr()[i] - after.ptr()[i]));
        mHeadDetachLinearVelocityError = (snapshot.linearAfter
            - (snapshot.launched ? snapshot.requestedVelocity + btVector3(0, 0, Misc::BrutalHeadSeparationSpeed) : snapshot.linearBefore)).length();
        mHeadDetachAngularVelocityError = (snapshot.angularAfter - snapshot.headAngularExpected).length();
        mHeadDetachLinearVelocity = Misc::Convert::toOsg(snapshot.linearBefore);
        mHeadDetachAngularVelocity = Misc::Convert::toOsg(snapshot.angularBefore);
        const auto& retainedBefore = snapshot.retainedBefore;
        const auto& retainedAfter = snapshot.retainedAfter;
        const auto comVelocity = [](const auto& state) {
            return state.mass > 0 ? osg::Vec3f(Misc::Convert::toOsg(state.linear / state.mass)) : osg::Vec3f();
        };
        mHeadDetachRetainedState = { comVelocity(retainedBefore), comVelocity(retainedAfter),
            osg::Vec3f(snapshot.retainedTransformError, snapshot.retainedLinearVelocityError,
                snapshot.retainedAngularVelocityError),
            Misc::Convert::toOsg(retainedBefore.linear), Misc::Convert::toOsg(retainedAfter.linear),
            Misc::Convert::toOsg(retainedBefore.angular), Misc::Convert::toOsg(retainedAfter.angular),
            osg::Vec3f(retainedBefore.mass, (retainedAfter.linear - retainedBefore.linear).length(),
                (retainedAfter.angular - retainedBefore.angular).length()),
            Misc::Convert::toOsg(retainedBefore.center), Misc::Convert::toOsg(retainedAfter.center) };
        mHeadDetachAppliedState = { cutVelocity, Misc::Convert::toOsg(snapshot.linearAfter),
            Misc::Convert::toOsg(snapshot.angularAfter), headAngularImpulse,
            Misc::Convert::toOsg(snapshot.angularAfter - snapshot.angularBefore),
            osg::Vec3f(snapshot.targetLinearVelocityError, mHeadDetachAngularVelocityError,
                snapshot.retainedAngularVelocityError),
            Misc::Convert::toOsg(snapshot.appliedHeadImpulse),
            osg::Vec3f(snapshot.launched, 62.831853071795864, snapshot.unclampedHeadAngularSpeed) };
        for (const auto& velocity : snapshot.partLinearAfter)
            mHeadDetachPartVelocities.emplace_back(Misc::Convert::toOsg(velocity));
        return true;
    }

    std::string Ragdoll::getBisectionPoseData() const
    {
        if (!mTorsoDetached || !mRegistered || mBodies.size() != 11) return {};
        const auto actorFrame = bisectionActorFrame(getPtr());
        if (!actorFrame) return {};
        // Both anchors belong to the same completed physics snapshot, even when
        // the physics worker advances independently of the save operation.
        const auto anchors = mTaskScheduler->readRigidBodyTransforms({ mBodies[0], mBodies[1] });
        std::string bytes("BSP1");
        bytes.reserve(BisectionPoseBytes);
        for (const auto& world : anchors)
        {
            const auto relative = actorFrame->inverse() * world;
            const auto position = relative.getOrigin();
            auto rotation = relative.getRotation();
            if (!validBisectionAnchor(position, rotation)) return {};
            rotation.normalize();
            for (btScalar value : { position.x(), position.y(), position.z(),
                     rotation.x(), rotation.y(), rotation.z(), rotation.w() })
                appendBisectionFloat(bytes, static_cast<float>(value));
        }
        return bytes;
    }

    bool Ragdoll::detachTorso(const osg::Vec3f& cutVelocity, const osg::Vec3f& upperSeparation,
        const std::string& savedPose)
    {
        if (mTorsoDetached) return true;
        if (mHeadDetached || mSeveredLimbsMask || !mRegistered || mParts.size() != 11) return false;
        auto* pelvis = mBodies[0];
        auto* torso = mBodies[1];
        const auto found = std::find_if(mJoints.begin(), mJoints.end(), [&](const auto& joint) {
            return (&joint->getRigidBodyA() == pelvis && &joint->getRigidBodyB() == torso)
                || (&joint->getRigidBodyB() == pelvis && &joint->getRigidBodyA() == torso);
        });
        if (found == mJoints.end()) return false;
        std::optional<std::array<btTransform, 2>> desiredAnchors;
        if (!savedPose.empty())
            if (const auto actorFrame = bisectionActorFrame(getPtr()))
                desiredAnchors = readBisectionPose(savedPose, *actorFrame);
        mTorsoPosePayloadRejected = !savedPose.empty() && !desiredAnchors;
        auto* removed = found->get();
        auto friction = std::make_unique<JointFriction>();
        friction->mTorsoDetached = true;
        for (const auto& joint : mJoints)
            if (joint.get() != removed)
                friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
        // A nonempty payload denotes restoration, including a rejected payload
        // from an older/corrupt save. It must never replay a fresh cut launch.
        mTaskScheduler->cancelSuperKick(mSuperKickToken);
        const auto snapshot = mTaskScheduler->detachRagdollTorso(removed, mJointFriction.get(),
            friction.get(), mBodies, savedPose.empty() ? Misc::Convert::toBullet(cutVelocity) : btVector3(0, 0, 0),
            savedPose.empty() ? Misc::Convert::toBullet(upperSeparation) : btVector3(0, 0, 0), desiredAnchors);
        mTorsoDetachErrors = snapshot.errors;
        mTorsoPoseRestored = snapshot.poseRestored;
        mTorsoPosePayloadRejected |= desiredAnchors.has_value() && !snapshot.poseRestored;
        mTorsoPoseRestoreError = snapshot.poseRestoreError;
        mTorsoPoseRestoreDisplacement = snapshot.poseRestoreDisplacement;
        mTorsoPoseRestoreRotation = snapshot.poseRestoreRotation;
        mTorsoPoseAnchorDistance = snapshot.poseAnchorDistance;
        mConstraints.erase(std::remove(mConstraints.begin(), mConstraints.end(), removed), mConstraints.end());
        mJoints.erase(found);
        mJointFriction = std::move(friction);
        mTorsoDetached = true;
        return true;
    }

    bool Ragdoll::detachLimbs(unsigned mask, const osg::Vec3f& cutVelocity, const std::string& savedPose)
    {
        if (!LimbSevering::validMask(mask)) return false;
        if (mSeveredLimbsMask) return mSeveredLimbsMask == mask;
        if (mHeadDetached || mTorsoDetached || !mRegistered || mBodies.size() != LimbSevering::PartCount)
            return false;
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(cutVelocity[axis])) return false;
        std::vector<btTypedConstraint*> removed;
        for (std::size_t bit = 0; bit < LimbSevering::Roots.size(); ++bit)
        {
            if (!(mask & (1u << bit))) continue;
            auto* child = mBodies[LimbSevering::Roots[bit]];
            auto* parent = mBodies[sPartDefs[LimbSevering::Roots[bit]].mParent];
            const auto found = std::find_if(mJoints.begin(), mJoints.end(), [&](const auto& joint) {
                return (&joint->getRigidBodyA() == parent && &joint->getRigidBodyB() == child)
                    || (&joint->getRigidBodyB() == parent && &joint->getRigidBodyA() == child);
            });
            if (found == mJoints.end()) return false;
            removed.push_back(found->get());
        }
        auto friction = std::make_unique<JointFriction>();
        friction->mSeveredLimbsMask = mask;
        friction->mRemovedConstraints = removed;
        for (const auto& joint : mJoints)
            if (std::find(removed.begin(), removed.end(), joint.get()) == removed.end())
                friction->mPairs.emplace_back(&joint->getRigidBodyA(), &joint->getRigidBodyB());
        std::optional<std::array<btTransform, LimbSevering::PartCount>> desiredPose;
        if (!savedPose.empty())
            if (const auto frame = bisectionActorFrame(getPtr()))
                desiredPose = LimbSevering::decodePose(savedPose, mask, *frame);
        mLimbPosePayloadRejected = !savedPose.empty() && !desiredPose;
        mTaskScheduler->cancelSuperKick(mSuperKickToken);
        const auto snapshot = mTaskScheduler->detachRagdollLimbs(removed, mJointFriction.get(), friction.get(),
            mBodies, mask, savedPose.empty() ? Misc::Convert::toBullet(cutVelocity) : btVector3(0, 0, 0), desiredPose);
        mLimbDetachErrors = snapshot.errors;
        mLimbPoseRestored = snapshot.poseRestored;
        mLimbPosePayloadRejected |= desiredPose.has_value() && !snapshot.poseRestored;
        mLimbPoseRestoreError = snapshot.poseRestoreError;
        std::erase_if(mConstraints, [&](auto* constraint) {
            return std::find(removed.begin(), removed.end(), constraint) != removed.end();
        });
        std::erase_if(mJoints, [&](const auto& joint) {
            return std::find(removed.begin(), removed.end(), joint.get()) != removed.end();
        });
        mJointFriction = std::move(friction);
        mSeveredLimbsMask = mask;
        return true;
    }

    std::string Ragdoll::getLimbSeveringPoseData() const
    {
        if (!mSeveredLimbsMask || !mRegistered || mBodies.size() != LimbSevering::PartCount) return {};
        const auto frame = bisectionActorFrame(getPtr());
        if (!frame) return {};
        // Body and every fragment are read under one completed-step lock. The
        // stable actor frame avoids turning a saved floor gap vertical on reload.
        const auto transforms = mTaskScheduler->readRigidBodyTransforms(mBodies);
        std::array<btTransform, LimbSevering::PartCount> pose;
        std::copy(transforms.begin(), transforms.end(), pose.begin());
        return LimbSevering::encodePose(mSeveredLimbsMask, pose, *frame);
    }

    std::vector<osg::Vec3f> Ragdoll::getHeadState() const
    {
        osg::Vec3f position, linear, angular;
        if (mLastTransforms.size() > sHeadPart)
        {
            const auto& body = mLastTransforms[sHeadPart];
            position = (mParts[sHeadPart]->mRenderFromBody
                * osg::Matrixf::rotate(Misc::Convert::toOsg(body.getRotation()))
                * osg::Matrixf::translate(Misc::Convert::toOsg(body.getOrigin()))).getTrans();
        }
        if (mLinearVelocities.size() > sHeadPart)
            linear = Misc::Convert::toOsg(mLinearVelocities[sHeadPart]);
        if (mAngularVelocities.size() > sHeadPart)
            angular = Misc::Convert::toOsg(mAngularVelocities[sHeadPart]);
        const auto* friction = static_cast<const JointFriction*>(mJointFriction.get());
        std::vector<osg::Vec3f> state{ osg::Vec3f(mHeadDetached, mJoints.size(),
                     friction ? friction->mHeadCollisionPairsEnabled.load(std::memory_order_relaxed) : 0),
            osg::Vec3f(mHeadDetachTransformError, mHeadDetachLinearVelocityError, mHeadDetachAngularVelocityError),
            position, linear, angular, mHeadDetachLinearVelocity, mHeadDetachAngularVelocity };
        state.insert(state.end(), mHeadDetachRetainedState.begin(), mHeadDetachRetainedState.end());
        state.insert(state.end(), mHeadDetachAppliedState.begin(), mHeadDetachAppliedState.end());
        state.insert(state.end(), mHeadDetachPartVelocities.begin(), mHeadDetachPartVelocities.end());
        return state;
    }

    std::vector<std::pair<std::string, osg::Matrixf>> Ragdoll::getBonePoses(std::optional<float> interpolation) const
    {
        // One lock spans the entire articulated body, so joints cannot mix physics steps.
        std::vector<btTransform> rendered;
        mLastTransforms = mTaskScheduler->readRigidBodyTransforms(
            mBodies, &mLinearVelocities, &mAngularVelocities, &rendered, interpolation);
        mMaxSpeed = mMaxAngularSpeed = 0.f;
        for (size_t i=0; i<mBodies.size(); ++i)
        {
            mMaxSpeed = std::max(mMaxSpeed, float(mLinearVelocities[i].length()));
            mMaxAngularSpeed = std::max(mMaxAngularSpeed, float(mAngularVelocities[i].length()));
        }
        mLastPoses.clear();
        mLastPoses.reserve(mParts.size());
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            const auto& part = mParts[i];
            const auto& body=rendered[i];
            mLastPoses.emplace_back(part->mBone,part->mRenderFromBody
                *osg::Matrixf::rotate(Misc::Convert::toOsg(body.getRotation()))
                *osg::Matrixf::translate(Misc::Convert::toOsg(body.getOrigin())));
        }
        return mLastPoses;
    }

    float Ragdoll::bottom() const
    {
        float z = std::numeric_limits<float>::max();
        for (size_t i=0; i<mParts.size(); ++i)
        {
            btVector3 lo, hi; mParts[i]->mBody->getCollisionShape()->getAabb(mLastTransforms[i], lo, hi);
            z = std::min(z, float(lo.z()));
        }
        return z;
    }

    float Ragdoll::retainedBottom() const
    {
        float z = std::numeric_limits<float>::max();
        for (size_t i = 0; i < mParts.size(); ++i)
        {
            // A detached head can fall to a different floor. It must not drag
            // the remaining corpse's persisted scene reference down with it.
            if (mHeadDetached && mParts[i]->mBone == "Bip01 Head")
                continue;
            if (mTorsoDetached && i >= 1 && i <= 6)
                continue; // The lower half owns the persisted actor root/floor.
            if (LimbSevering::fragment(i, mSeveredLimbsMask))
                continue; // A detached limb on a lower floor cannot move the torso's scene reference.
            btVector3 lo, hi;
            mParts[i]->mBody->getCollisionShape()->getAabb(mLastTransforms[i], lo, hi);
            z = std::min(z, float(lo.z()));
        }
        return z;
    }

    float Ragdoll::findKickPart(const osg::Vec3f& origin, const osg::Vec3f& forward, float reach,
        float minimumDot, std::size_t& part, osg::Vec3f& point) const
    {
        float best = -1.f;
        // Render interpolation intentionally trails physics. Target the latest
        // completed physical pose, then measure reach to its collision surface.
        const auto transforms = mTaskScheduler->readRigidBodyTransforms(mBodies);
        const btVector3 from = Misc::Convert::toBullet(origin);
        for (std::size_t i = 0; i < mParts.size() && i < transforms.size(); ++i)
        {
            const osg::Vec3f offset = Misc::Convert::toOsg(transforms[i].getOrigin()) - origin;
            osg::Vec3f flat(offset.x(), offset.y(), 0.f);
            if (flat.normalize() < 1e-3f || flat * forward < minimumDot) continue;
            btTransform source = btTransform::getIdentity();
            source.setOrigin(from);
            auto contact = mTaskScheduler->getHitPoint(source, mParts[i]->mBody);
            if (!contact && mCreature)
            {
                // The player can walk into a corpse. A boot starting inside a big creature's
                // capsule finds no surface to cross, yet it plainly touches that part.
                const auto* capsule = static_cast<const btCapsuleShape*>(mParts[i]->mShape.get());
                const btVector3 local = transforms[i].invXform(from);
                const btScalar along = btClamped(local.y(), -capsule->getHalfHeight(), capsule->getHalfHeight());
                if ((local - btVector3(0, along, 0)).length() <= capsule->getRadius())
                    contact = from;
            }
            if (!contact) continue;
            const float distance = (*contact - from).length();
            if (distance > reach || (best >= 0.f && distance >= best)) continue;
            // Ragdolls are dynamic bodies; only solid world geometry blocks the
            // boot-to-surface ray. A reachable corpse behind a wall is not a hit.
            btCollisionWorld::ClosestRayResultCallback obstruction(from, *contact);
            obstruction.m_collisionFilterGroup = CollisionType_Actor;
            obstruction.m_collisionFilterMask = CollisionType_World | CollisionType_Door | CollisionType_HeightMap;
            mTaskScheduler->rayTest(from, *contact, obstruction);
            if (obstruction.hasHit() && obstruction.m_closestHitFraction < btScalar(.999)) continue;
            best = distance;
            part = i;
            point = Misc::Convert::toOsg(*contact);
        }
        return best;
    }

    bool Ragdoll::isDetachedHeadPart(std::size_t part) const
    {
        return mHeadDetached && part == sHeadPart;
    }

    bool Ragdoll::applyKick(std::size_t part, const osg::Vec3f& velocity, const osg::Vec3f& point)
    {
        if (!mRegistered || part >= mParts.size()) return false;
        auto* control = static_cast<JointFriction*>(mJointFriction.get());
        if (!mTaskScheduler->applyRagdollKick(mJointFriction.get(), part, Misc::Convert::toBullet(point),
                Misc::Convert::toBullet(velocity * mKickScale), (std::uint64_t(1) << 63) | ++control->mSerial)) return false;
        ++mKickCount;
        return true;
    }

    bool Ragdoll::startSuperKick(std::uint64_t token, std::uint64_t kickId, const osg::Vec3f& velocity,
        const btCollisionObject* attacker)
    {
        if (!mRegistered || mHeadDetached || mTorsoDetached || mSeveredLimbsMask
            || (!mCreature && mBodies.size() != 11)) return false;
        mTaskScheduler->startSuperKick(token, kickId, mBodies, velocity * mKickScale, attacker);
        mSuperKickToken = token;
        getBonePoses(1.f);
        mInitialCOMVelocity = getDynamics()[1];
        return true;
    }

    std::map<std::string, double> Ragdoll::getSuperKickState() const
    {
        return mSuperKickToken ? mTaskScheduler->readSuperKick(mSuperKickToken) : std::map<std::string, double>{};
    }

    void Ragdoll::guardSuperKickSpawn()
    {
        mTaskScheduler->guardSuperKickSpawn(mBodies);
    }

    void Ragdoll::configureEnhancement()
    {
        const auto& game = Settings::game();
        RagdollControlConfig config;
        // The anatomical profile describes the human body; a creature keeps plain joints.
        config.enabled = game.mRagdollEnhancement && !mCreature;
        config.softStops = game.mRagdollSoftStops; config.neutralPose = game.mRagdollNeutralPose;
        config.limbResponse = game.mRagdollLimbResponse; config.selfCollision = game.mRagdollSelfCollision;
        config.softStrength = game.mRagdollSoftStrength; config.toneStrength = game.mRagdollToneStrength;
        config.impactStrength = game.mRagdollImpactStrength;
        config.waterEnabled = mWaterEnabled;
        config.waterHeight = mWaterHeight;
        if (!std::isfinite(config.softStrength) || !std::isfinite(config.toneStrength)
            || !std::isfinite(config.impactStrength))
            throw std::runtime_error("Non-finite ragdoll profile strength");
        mTaskScheduler->configureRagdoll(mJointFriction.get(), config);
    }

    std::map<std::string, double> Ragdoll::getEnhancementState() const
    {
        auto state = mTaskScheduler->readRagdollControl(mJointFriction.get());
        state["limbsMask"] = mSeveredLimbsMask;
        state["limbConstraintCount"] = mJoints.size();
        state["limbDetachTransformError"] = mLimbDetachErrors[0];
        state["limbDetachVelocityError"] = mLimbDetachErrors[1];
        state["limbDetachAngularError"] = mLimbDetachErrors[2];
        state["limbPoseRestored"] = mLimbPoseRestored;
        state["limbPosePayloadRejected"] = mLimbPosePayloadRejected;
        state["limbPoseRestoreError"] = mLimbPoseRestoreError;
        state["torsoDetached"] = mTorsoDetached;
        state["torsoConstraintCount"] = mJoints.size();
        state["torsoDetachTransformError"] = mTorsoDetachErrors[0];
        state["torsoDetachVelocityError"] = mTorsoDetachErrors[1];
        state["torsoDetachAngularError"] = mTorsoDetachErrors[2];
        state["torsoPoseRestored"] = mTorsoPoseRestored;
        state["torsoPosePayloadRejected"] = mTorsoPosePayloadRejected;
        state["torsoPoseRestoreError"] = mTorsoPoseRestoreError;
        state["torsoPoseRestoreDisplacement"] = mTorsoPoseRestoreDisplacement;
        state["torsoPoseRestoreRotation"] = mTorsoPoseRestoreRotation;
        state["torsoPoseAnchorDistance"] = mTorsoPoseAnchorDistance;
        for (const auto& [bone, prefix] : { std::pair{ "Bip01 L Hand", "hand4." },
                 std::pair{ "Bip01 R Hand", "hand6." }, std::pair{ "Bip01 L Foot", "foot8." },
                 std::pair{ "Bip01 R Foot", "foot10." } })
        {
            const auto found = mUnsupportedExtremityBones.find(bone);
            state[std::string(prefix) + "unsupportedBoneCount"]
                = found == mUnsupportedExtremityBones.end() ? 0 : found->second;
        }
        state["initialMaxAngularSpeed"] = mInitialMaxAngularSpeed;
        for (int axis = 0; axis < 3; ++axis)
        {
            const std::string suffix(1, "XYZ"[axis]);
            state["initialRootVelocity" + suffix] = mInitialRootVelocity[axis];
            state["initialAnimationVelocity" + suffix] = mInheritedVelocity[axis] - mInitialRootVelocity[axis];
        }
        return state;
    }

    bool Ragdoll::applyImpact(std::size_t part, const osg::Vec3f& point, const osg::Vec3f& impulse,
        std::uint64_t id, bool alreadyApplied)
    {
        if (part == std::numeric_limits<std::size_t>::max())
        {
            const auto transforms = mTaskScheduler->readRigidBodyTransforms(mBodies);
            btScalar distance = BT_LARGE_FLOAT;
            for (std::size_t i = 0; i < transforms.size(); ++i)
            {
                const auto delta = transforms[i].getOrigin() - Misc::Convert::toBullet(point);
                if (delta.length2() < distance) { distance = delta.length2(); part = i; }
            }
        }
        if (!id)
            id = (std::uint64_t(1) << 62) | ++static_cast<JointFriction*>(mJointFriction.get())->mSerial;
        if (mCreature)
        {
            // No anatomical controller: share the momentum across the connected body.
            if (alreadyApplied || part >= mParts.size()) return false;
            btScalar mass = 0;
            for (const auto& each : mParts) mass += 1 / each->mBody->getInvMass();
            const bool applied = mTaskScheduler->applyRagdollKick(mJointFriction.get(), part,
                Misc::Convert::toBullet(point), Misc::Convert::toBullet(impulse) / mass, id);
            if (applied && !mImpulse.length2())
            {
                mImpulse = impulse;
                mContact = point;
            }
            return applied;
        }
        const bool applied = mTaskScheduler->applyRagdollImpact(mJointFriction.get(),
            { id, part, Misc::Convert::toBullet(point), Misc::Convert::toBullet(impulse), alreadyApplied });
        if (applied && !alreadyApplied && getEnhancementState().at("simulatedAge") == 0)
        {
            // Native lethal hits create the pose with zero kick, then deliver J.
            // Record the resulting initial state before the first physical step;
            // compatibility notifications and later corpse hits retain their history.
            mLastTransforms = mTaskScheduler->readRigidBodyTransforms(mBodies, &mLinearVelocities, &mAngularVelocities);
            mInitialCOMVelocity = getDynamics()[1];
            mImpulse = impulse;
            mContact = point;
        }
        return applied;
    }

    bool Ragdoll::getEquipmentFrame(bool left, osg::Matrixf& frame, osg::Vec3f& linear, osg::Vec3f& angular) const
    {
        const std::size_t index = left ? 4 : 6; // Actual hand geometry is rigidly owned by its forearm.
        if (mBodies.size() <= index)
            return false;
        std::vector<btVector3> velocities, spins;
        const auto transforms = mTaskScheduler->readRigidBodyTransforms(mBodies, &velocities, &spins);
        const auto& transform = transforms[index];
        frame = osg::Matrixf::rotate(Misc::Convert::toOsg(transform.getRotation()))
            * osg::Matrixf::translate(Misc::Convert::toOsg(transform.getOrigin()));
        linear = Misc::Convert::toOsg(velocities[index]);
        angular = Misc::Convert::toOsg(spins[index]);
        return true;
    }

    std::vector<btRigidBody*> Ragdoll::getEquipmentGripBodies() const
    {
        if (mBodies.size() <= 6)
            return {};
        return { mBodies[4], mBodies[6] };
    }

    std::vector<osg::Vec3f> Ragdoll::getDynamics() const
    {
        osg::Vec3f center, velocity; float mass = 0.f;
        for (size_t i=0; i<mParts.size(); ++i)
        {
            const float m = 1.f/mParts[i]->mBody->getInvMass(); mass += m;
            center += Misc::Convert::toOsg(mLastTransforms[i].getOrigin())*m;
            velocity += Misc::Convert::toOsg(mLinearVelocities[i])*m;
        }
        return {center/mass, velocity/mass, mInheritedVelocity, mInitialCOMVelocity,
            mImpulse, mContact, osg::Vec3f(mInitialPoseError, mMaxSpeed, mMaxAngularSpeed)};
    }

    float Ragdoll::maxJointError() const
    {
        if (mLastTransforms.size() != mParts.size())
            return 0.f;
        float result = 0.f;
        for (const auto& joint : mJoints)
        {
            const auto* cone = static_cast<const btConeTwistConstraint*>(joint.get());
            const auto a = std::find(mBodies.begin(), mBodies.end(), &joint->getRigidBodyA());
            const auto b = std::find(mBodies.begin(), mBodies.end(), &joint->getRigidBodyB());
            const btVector3 anchorA = mLastTransforms[a - mBodies.begin()] * cone->getAFrame().getOrigin();
            const btVector3 anchorB = mLastTransforms[b - mBodies.begin()] * cone->getBFrame().getOrigin();
            result = std::max(result, static_cast<float>((anchorA - anchorB).length()));
        }
        return result;
    }

    std::optional<osg::Vec3f> Ragdoll::getActorPosition(const osg::Vec3f& actorPosition)
    {
        if (mLastTransforms.empty())
            return std::nullopt;
        const auto& body = mLastTransforms.front();
        return (mParts.front()->mRenderFromBody
            * osg::Matrixf::rotate(Misc::Convert::toOsg(body.getRotation()))
            * osg::Matrixf::translate(Misc::Convert::toOsg(body.getOrigin()))).getTrans() + mActorOffset;
    }

    MWWorld::Ptr Ragdoll::getPtr() const
    {
        return mParts.front()->getPtr();
    }

    void Ragdoll::updatePtr(const MWWorld::Ptr& updated)
    {
        for (const auto& part : mParts)
            part->updatePtr(updated);
    }
}
