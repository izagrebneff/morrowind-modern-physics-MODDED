#include "npcanimation.hpp"
#include "decapitation.hpp"
#include "brutalcutmaterial.hpp"
#include "limbsevering.hpp"
#include <components/misc/strings/algorithm.hpp>
#include <osg/Material>
#include <osg/Image>
#include <osg/Texture2D>
#include <cmath>

#include <osg/Depth>
#include <osg/MatrixTransform>

#include <osgUtil/CullVisitor>
#include <osgUtil/RenderBin>

#include <components/debug/debuglog.hpp>

#include <components/misc/rng.hpp>

#include <components/misc/resourcehelpers.hpp>

#include <components/esm3/loadbody.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadrace.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/keyframe.hpp>
#include <components/sceneutil/lightcommon.hpp>
#include <components/sceneutil/textkeymap.hpp>
#include <components/sceneutil/userdata.hpp>
#include <components/sceneutil/visitor.hpp>
#include <components/settings/values.hpp>

#include <components/vfs/manager.hpp>

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/npcstats.hpp"
#include "../mwmechanics/weapontype.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/world.hpp"

#include "actorutil.hpp"
#include "postprocessor.hpp"
#include "renderbin.hpp"
#include "renderingmanager.hpp"
#include "rotatecontroller.hpp"
#include "vismask.hpp"

namespace
{
    VFS::Path::Normalized getVampireHead(ESM::RefId race, bool female)
    {
        static std::map<std::pair<ESM::RefId, bool>, const ESM::BodyPart*> sVampireMapping;

        const std::pair<ESM::RefId, bool> key(race, female);
        const ESM::BodyPart* vampireHead = nullptr;

        if (const auto it = sVampireMapping.find(key); it == sVampireMapping.end())
        {
            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

            for (const ESM::BodyPart& bodyPart : store.get<ESM::BodyPart>())
            {
                if (!bodyPart.mData.mVampire)
                    continue;
                if (bodyPart.mData.mType != ESM::BodyPart::MT_Skin)
                    continue;
                if (bodyPart.mData.mPart != ESM::BodyPart::MP_Head)
                    continue;
                if (female != (bodyPart.mData.mFlags & ESM::BodyPart::BPF_Female))
                    continue;
                if (!(bodyPart.mRace == race))
                    continue;

                vampireHead = &bodyPart;
            }

            sVampireMapping.emplace_hint(it, key, vampireHead);
        }
        else
        {
            vampireHead = it->second;
        }

        if (vampireHead == nullptr)
            return {};

        return Misc::ResourceHelpers::correctMeshPath(vampireHead->mModel.getNormalized());
    }

}

namespace MWRender
{
    class HeadAnimationTime : public SceneUtil::ControllerSource
    {
    private:
        MWWorld::Ptr mReference;
        float mTalkStart;
        float mTalkStop;
        float mBlinkStart;
        float mBlinkStop;

        float mBlinkTimer;

        bool mEnabled;

        float mValue;

    private:
        void resetBlinkTimer();

    public:
        HeadAnimationTime(const MWWorld::Ptr& reference);

        void updatePtr(const MWWorld::Ptr& updated);

        void update(float dt);

        void setEnabled(bool enabled);

        void setTalkStart(float value);
        void setTalkStop(float value);
        void setBlinkStart(float value);
        void setBlinkStop(float value);

        float getValue(osg::NodeVisitor* nv) override;
    };

    // --------------------------------------------------------------------------------------------------------------

    HeadAnimationTime::HeadAnimationTime(const MWWorld::Ptr& reference)
        : mReference(reference)
        , mTalkStart(0)
        , mTalkStop(0)
        , mBlinkStart(0)
        , mBlinkStop(0)
        , mEnabled(true)
        , mValue(0)
    {
        resetBlinkTimer();
    }

    void HeadAnimationTime::updatePtr(const MWWorld::Ptr& updated)
    {
        mReference = updated;
    }

    void HeadAnimationTime::setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }

    void HeadAnimationTime::resetBlinkTimer()
    {
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        mBlinkTimer = -(2.0f + Misc::Rng::rollDice(6, prng));
    }

    void HeadAnimationTime::update(float dt)
    {
        if (!mEnabled)
            return;

        if (dt == 0.f)
            return;

        if (!MWBase::Environment::get().getSoundManager()->sayActive(mReference))
        {
            mBlinkTimer += dt;

            float duration = mBlinkStop - mBlinkStart;

            if (mBlinkTimer >= 0 && mBlinkTimer <= duration)
            {
                mValue = mBlinkStart + mBlinkTimer;
            }
            else
                mValue = mBlinkStop;

            if (mBlinkTimer > duration)
                resetBlinkTimer();
        }
        else
        {
            // FIXME: would be nice to hold on to the SoundPtr so we don't have to retrieve it every frame
            mValue = mTalkStart
                + (mTalkStop - mTalkStart)
                    * std::min(1.f,
                        MWBase::Environment::get().getSoundManager()->getSaySoundLoudness(mReference)
                            * 2); // Rescale a bit (most voices are not very loud)
        }
    }

    float HeadAnimationTime::getValue(osg::NodeVisitor*)
    {
        return mValue;
    }

    void HeadAnimationTime::setTalkStart(float value)
    {
        mTalkStart = value;
    }

    void HeadAnimationTime::setTalkStop(float value)
    {
        mTalkStop = value;
    }

    void HeadAnimationTime::setBlinkStart(float value)
    {
        mBlinkStart = value;
    }

    void HeadAnimationTime::setBlinkStop(float value)
    {
        mBlinkStop = value;
    }

    // ----------------------------------------------------

    NpcAnimation::NpcType NpcAnimation::getNpcType() const
    {
        const MWWorld::Class& cls = mPtr.getClass();
        // Dead vampires should typically stay vampires.
        if (mNpcType == Type_Vampire && cls.getNpcStats(mPtr).isDead() && !cls.getNpcStats(mPtr).isWerewolf())
            return mNpcType;
        return getNpcType(mPtr);
    }

    NpcAnimation::NpcType NpcAnimation::getNpcType(const MWWorld::Ptr& ptr)
    {
        const MWWorld::Class& cls = ptr.getClass();
        NpcAnimation::NpcType curType = Type_Normal;
        if (cls.getCreatureStats(ptr).getMagicEffects().getOrDefault(ESM::MagicEffect::Vampirism).getMagnitude() > 0)
            curType = Type_Vampire;
        if (cls.getNpcStats(ptr).isWerewolf())
            curType = Type_Werewolf;

        return curType;
    }

    static const inline NpcAnimation::PartBoneMap createPartListMap()
    {
        return { { ESM::PRT_Head, "Head" },
            { ESM::PRT_Hair, "Head" }, // note it uses "Head" as attach bone, but "Hair" as filter
            { ESM::PRT_Neck, "Neck" }, { ESM::PRT_Cuirass, "Chest" }, { ESM::PRT_Groin, "Groin" },
            { ESM::PRT_Skirt, "Groin" }, { ESM::PRT_RHand, "Right Hand" }, { ESM::PRT_LHand, "Left Hand" },
            { ESM::PRT_RWrist, "Right Wrist" }, { ESM::PRT_LWrist, "Left Wrist" }, { ESM::PRT_Shield, "Shield Bone" },
            { ESM::PRT_RForearm, "Right Forearm" }, { ESM::PRT_LForearm, "Left Forearm" },
            { ESM::PRT_RUpperarm, "Right Upper Arm" }, { ESM::PRT_LUpperarm, "Left Upper Arm" },
            { ESM::PRT_RFoot, "Right Foot" }, { ESM::PRT_LFoot, "Left Foot" }, { ESM::PRT_RAnkle, "Right Ankle" },
            { ESM::PRT_LAnkle, "Left Ankle" }, { ESM::PRT_RKnee, "Right Knee" }, { ESM::PRT_LKnee, "Left Knee" },
            { ESM::PRT_RLeg, "Right Upper Leg" }, { ESM::PRT_LLeg, "Left Upper Leg" },
            { ESM::PRT_RPauldron, "Right Clavicle" }, { ESM::PRT_LPauldron, "Left Clavicle" },
            { ESM::PRT_Weapon, "Weapon Bone" }, // Fallback. The real node name depends on the current weapon type.
            { ESM::PRT_Tail, "Tail" } };
    }
    const NpcAnimation::PartBoneMap NpcAnimation::sPartList = createPartListMap();

    NpcAnimation::~NpcAnimation()
    {
        mBrutalHeadCut.reset();
        mBrutalLimbCut.reset();
        mAmmunition.reset();
    }

    bool NpcAnimation::setBrutalHeadSevered(bool severed)
    {
        if (!severed)
        {
            mBrutalHeadCut.reset();
            return true;
        }
        if (mBrutalHeadCut)
            return true;
        if (mBrutalLimbCut)
            return false;
        if (mViewMode != VM_Normal || mNpcType == Type_Werewolf
            || mPtr == MWMechanics::getPlayer())
            return false;

        const auto neck = getBoneWorldMatrix("Bip01 Neck");
        const auto actualHead = getBoneWorldMatrix("Bip01 Head");
        auto head = actualHead;
        if (neck && actualHead && mBrutalRebuildHeadInNeck)
            head = *mBrutalRebuildHeadInNeck * *neck;
        const auto& nodes = getNodeMap();
        const auto neckNode = nodes.find("Bip01 Neck");
        const auto headNode = nodes.find("Bip01 Head");
        if (!neck || !head || neckNode == nodes.end() || headNode == nodes.end()
            || !mObjectParts[ESM::PRT_Neck] || !mObjectParts[ESM::PRT_Head])
            return false;
        osg::Vec3f axis = head->getTrans() - neck->getTrans();
        const float distance = axis.normalize();
        if (!std::isfinite(distance) || distance <= 0.1f)
            return false;

        try
        {
            const osg::Matrixf correction = osg::Matrixf::inverse(*actualHead) * *head;
            const std::function<std::optional<osg::Matrixf>(std::string_view)> pose =
                [&](std::string_view name) -> std::optional<osg::Matrixf> {
                    auto world = getBoneWorldMatrix(name);
                    if (!world) return {};
                    const auto found = nodes.find(std::string(name));
                    if (found != nodes.end())
                    {
                        const auto paths = found->second->getParentalNodePaths();
                        if (!paths.empty())
                            for (const auto* node : paths.front())
                                if (Misc::StringUtils::ciEqual(node->getName(), "Bip01 Head"))
                                {
                                    *world *= correction;
                                    break;
                                }
                    }
                    return world;
                };
            std::vector<DecapitationGeometry::Snapshot> snapshots;
            for (const auto part : { ESM::PRT_Head, ESM::PRT_Hair, ESM::PRT_Neck })
                if (mObjectParts[part])
                {
                    const bool whole = part == ESM::PRT_Hair
                        || mPartslots[part] == MWWorld::InventoryStore::Slot_Helmet;
                    DecapitationGeometry::SnapshotVisitor visitor(snapshots, pose,
                        correction, whole,
                        mPartslots[part] == MWWorld::InventoryStore::Slot_Helmet);
                    visitor.snapshot(mObjectParts[part]->getNode());
                    if (visitor.failed)
                        return false;
                }
            if (snapshots.empty())
                return false;

            auto capState = prepareBrutalCutMaterial(mResourceSystem, false);
            auto fragmentState = prepareBrutalCutMaterial(mResourceSystem, true);
            auto cut = std::make_unique<DecapitationGeometry>();
            // As in original Brutal Morrowind, retain body garment skinning
            // but pin any head-influenced vertices to the neck frame.
            DecapitationGeometry::RetainedRigVisitor retained;
            retained.setNodeMaskOverride(~0u);
            for (int part = 0; part < ESM::PRT_Count; ++part)
                if (part != ESM::PRT_Head && part != ESM::PRT_Hair && part != ESM::PRT_Neck
                    && mObjectParts[part])
                    retained.snapshot(mObjectParts[part]->getNode());
            const auto neckInverse = osg::Matrixf::inverse(*neck);
            for (const auto& [rig, path] : retained.rigs)
            {
                SceneUtil::RigGeometry::BonePoseOverrides overrides;
                std::set<unsigned short> affected;
                for (const auto& name : rig->getInfluencingBoneNames())
                {
                    const auto bone = nodes.find(name);
                    if (bone == nodes.end()) continue;
                    const auto paths = bone->second->getParentalNodePaths();
                    if (paths.empty()) continue;
                    const bool followsHead = std::any_of(paths.front().begin(), paths.front().end(),
                        [](const osg::Node* node) {
                            return Misc::StringUtils::ciEqual(node->getName(), "Bip01 Head");
                        });
                    if (!followsHead) continue;
                    const auto vertices = rig->getVerticesInfluencedBy(name);
                    if (vertices.empty()) continue;
                    const auto world = pose(name);
                    if (!world) return false;
                    overrides.emplace(name, SceneUtil::RigGeometry::BonePoseOverride{
                        "Bip01 Neck", *world * neckInverse, nullptr });
                    affected.insert(vertices.begin(), vertices.end());
                }
                if (overrides.empty()) continue;
                const auto snapshot = rig->snapshotWorldGeometry(path, pose);
                if (!snapshot) return false;
                const auto* vertices = dynamic_cast<const osg::Vec3Array*>(snapshot->getVertexArray());
                if (!vertices) return false;
                for (auto index : affected)
                {
                    if (index >= vertices->size()) return false;
                    cut->retainedVertexMaximum = std::max(cut->retainedVertexMaximum,
                        ((*vertices)[index] - neck->getTrans()) * axis);
                }
                cut->retainedRigs.push_back({rig, std::move(overrides)});
            }
            const float cutDistance = std::max(distance * .5f, cut->retainedVertexMaximum + .15f);
            if (cutDistance >= distance * .95f) return false;
            if (!cut->build(snapshots, *neck, *head, capState, cutDistance, axis, fragmentState))
            {
                Log(Debug::Warning) << "Brutal head cut declined: no valid closed geometry (stage "
                    << cut->failure << ")";
                return false;
            }

            // Shader/material compilation is necessary for procedural meshes
            // in OpenMW 0.52, especially when the original skin uses custom
            // shaders. Do it before altering visibility or touching Bullet.
            mResourceSystem->getSceneManager()->recreateShaders(cut->lower);
            mResourceSystem->getSceneManager()->recreateShaders(cut->upper);

            // All geometry allocation and validation precedes any mutation.
            if (!neckNode->second->addChild(cut->lower))
                return false;
            if (!headNode->second->addChild(cut->upper))
            {
                neckNode->second->removeChild(cut->lower);
                return false;
            }
            for (const auto part : { ESM::PRT_Head, ESM::PRT_Hair, ESM::PRT_Neck })
                if (mObjectParts[part])
                {
                    auto node = mObjectParts[part]->getNode();
                    cut->originals.emplace_back(node, node->getNodeMask());
                    node->setNodeMask(0);
                }
            cut->committed = true;
            for (auto& retainedRig : cut->retainedRigs)
                retainedRig.geometry->setBonePoseOverrides(std::move(retainedRig.poses));
            mBrutalHeadCut = std::move(cut);
            return true;
        }
        catch (const std::exception& error)
        {
            Log(Debug::Warning) << "Brutal head cut rejected without physics detach: " << error.what();
            return false;
        }
    }


    bool NpcAnimation::setBrutalLimbsSevered(unsigned mask)
    {
        if (mask == 0)
        {
            mBrutalLimbCut.reset();
            return true;
        }
        if (mask != 1u && mask != 2u && mask != 4u && mask != 8u)
            return false;
        if (mBrutalLimbCut)
            return mBrutalLimbCut->mask == mask;
        if (mBrutalHeadCut || mViewMode != VM_Normal || mNpcType == Type_Werewolf
            || mPtr == MWMechanics::getPlayer())
            return false;

        // Full-length robes and skirts can bridge both legs. Until the full
        // garment reconstruction path is ported, keep these NPCs intact.
        if (mask & 3u)
        {
            const auto& inventory = mPtr.getClass().getInventoryStore(mPtr);
            if (inventory.getSlot(MWWorld::InventoryStore::Slot_Skirt) != inventory.end()
                || inventory.getSlot(MWWorld::InventoryStore::Slot_Robe) != inventory.end())
                return false;
        }

        const auto& nodes = getNodeMap();
        struct LimbRequest
        {
            unsigned bit;
            const char* name;
            const char* parent;
            const char* child;
            const char* next;
        };
        constexpr LimbRequest requests[] = {
            { 1u, "leftLeg", "Bip01 L Thigh", "Bip01 L Calf", "Bip01 L Foot" },
            { 2u, "rightLeg", "Bip01 R Thigh", "Bip01 R Calf", "Bip01 R Foot" },
            { 4u, "leftArm", "Bip01 L Clavicle", "Bip01 L UpperArm", "Bip01 L Forearm" },
            { 8u, "rightArm", "Bip01 R Clavicle", "Bip01 R UpperArm", "Bip01 R Forearm" }
        };
        const LimbRequest* request = nullptr;
        for (const auto& candidate : requests)
            if (candidate.bit == mask)
                request = &candidate;
        if (!request)
            return false;

        const LimbSeveringGeometry::BoneWorld liveBones =
            [this](std::string_view name) { return getBoneWorldMatrix(name); };
        const auto descendant = [&](std::string_view name, std::string_view ancestor) {
            if (Misc::StringUtils::ciEqual(name, ancestor))
                return true;
            const auto found = nodes.find(std::string(name));
            if (found == nodes.end())
                return false;
            const auto paths = found->second->getParentalNodePaths();
            if (paths.empty())
                return false;
            return std::any_of(paths.front().begin(), paths.front().end(),
                [&](const osg::Node* node) {
                    return Misc::StringUtils::ciEqual(node->getName(), ancestor);
                });
        };
        const auto parent = liveBones(request->parent);
        const auto child = liveBones(request->child);
        const auto next = liveBones(request->next);
        if (!parent || !child || !next || nodes.find(request->child) == nodes.end()
            || nodes.find(request->parent) == nodes.end()
            || nodes.find("Bip01 Pelvis") == nodes.end())
            return false;

        LimbSeveringGeometry::Cut cut;
        cut.bit = request->bit;
        cut.name = request->name;
        cut.parent = request->parent;
        cut.child = request->child;
        cut.parentWorld = *parent;
        cut.childWorld = *child;
        cut.normal = next->getTrans() - child->getTrans();
        cut.length = cut.normal.normalize();
        if (!std::isfinite(cut.length) || cut.length < 0.1f)
            return false;
        cut.distance = std::clamp(cut.length * 0.075f, 0.75f, 2.5f);
        cut.center = child->getTrans() + cut.normal * cut.distance;

        const LimbSeveringGeometry::BoneRegion region =
            [&](std::string_view name) -> unsigned {
                return descendant(name, request->child) ? 1u : 0u;
            };
        const LimbSeveringGeometry::BoneTargets targets =
            [&](std::string_view name) -> unsigned {
                // Clavicle-only torso vertices must not be assigned to the arm.
                return descendant(name, request->child) ? mask : 0u;
            };
        const LimbSeveringGeometry::BoneTargets distal =
            [&](std::string_view name) -> unsigned {
                return !Misc::StringUtils::ciEqual(name, request->child)
                    && descendant(name, request->child) ? mask : 0u;
            };

        try
        {
            // Collect only actual skin/armor bound to the affected limb.
            // Rendering and Bullet remain unchanged until validation succeeds.
            std::vector<DecapitationGeometry::Snapshot> snapshots;
            std::array<bool, ESM::PRT_Count> replaced{};
            for (int part = 0; part < ESM::PRT_Count; ++part)
            {
                if (!mObjectParts[part] || part == ESM::PRT_Head || part == ESM::PRT_Hair
                    || part == ESM::PRT_Shield || part == ESM::PRT_Weapon)
                    continue;
                auto* node = mObjectParts[part]->getNode().get();
                if (!node)
                    continue;
                bool relevant = false;
                DecapitationGeometry::RetainedRigVisitor retained;
                retained.setNodeMaskOverride(~0u);
                retained.snapshot(node);
                for (const auto& [rig, path] : retained.rigs)
                    for (const auto& name : rig->getInfluencingBoneNames())
                        if (targets(name) && !rig->getVerticesInfluencedBy(name).empty())
                            relevant = true;
                const auto paths = node->getParentalNodePaths();
                if (!paths.empty())
                    for (const auto* bone : paths.front())
                        if (targets(bone->getName()))
                            relevant = true;
                if (!relevant)
                    continue;
                const size_t before = snapshots.size();
                DecapitationGeometry::SnapshotVisitor visitor(
                    snapshots, liveBones, osg::Matrixf::identity(), false);
                visitor.snapshot(node);
                if (visitor.failed)
                    return false;
                replaced[part] = snapshots.size() > before;
            }
            if (snapshots.empty())
                return false;

            auto cap = prepareBrutalCutMaterial(mResourceSystem, false);
            auto fragmentCap = prepareBrutalCutMaterial(mResourceSystem, true);
            auto prepared = std::make_unique<LimbSeveringGeometry>();
            if (!prepared->build(snapshots, { cut }, liveBones, liveBones,
                    region, targets, distal, cap, fragmentCap))
            {
                Log(Debug::Warning) << "Brutal limb cut declined: mask=" << mask
                    << ", geometry stage=" << prepared->failure;
                return false;
            }
            for (const auto& surface : prepared->regions)
                mResourceSystem->getSceneManager()->recreateShaders(surface);
            for (const auto& limb : prepared->cuts)
                for (const auto& surface : limb.caps)
                    mResourceSystem->getSceneManager()->recreateShaders(surface);
            prepared->prepareDrawBuffers();

            // Stage everything hidden. If any attachment fails, the cut
            // destructor removes every staged node without changing NPC skin.
            for (const auto& anchor : prepared->anchors)
                if (nodes.find(anchor) == nodes.end())
                    return false;
            for (const auto& limb : prepared->cuts)
                if (nodes.find(limb.parent) == nodes.end()
                    || nodes.find(limb.child) == nodes.end())
                    return false;
            for (int part = 0; part < ESM::PRT_Count; ++part)
                if (replaced[part] && mObjectParts[part])
                    prepared->originals.emplace_back(mObjectParts[part]->getNode(), 0);
            for (size_t index = 0; index < prepared->regions.size(); ++index)
            {
                prepared->regions[index]->setNodeMask(0);
                if (!nodes.at(prepared->anchors[index])->addChild(prepared->regions[index]))
                    return false;
            }
            for (const auto& limb : prepared->cuts)
            {
                for (unsigned side = 0; side < 2; ++side)
                {
                    limb.caps[side]->setNodeMask(0);
                    if (!nodes.at(side ? limb.child : limb.parent)->addChild(limb.caps[side]))
                        return false;
                }
            }
            for (auto& [node, originalMask] : prepared->originals)
            {
                originalMask = node->getNodeMask();
                node->setNodeMask(0);
            }
            for (const auto& regionNode : prepared->regions)
                regionNode->setNodeMask(~0u);
            for (const auto& limb : prepared->cuts)
                for (const auto& capNode : limb.caps)
                    capNode->setNodeMask(~0u);
            prepared->committed = true;
            mBrutalLimbCut = std::move(prepared);
            return true;
        }
        catch (const std::exception& error)
        {
            Log(Debug::Warning) << "Brutal limb geometry rejected: " << error.what();
            return false;
        }
    }

    NpcAnimation::NpcAnimation(const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode,
        Resource::ResourceSystem* resourceSystem, bool disableSounds, ViewMode viewMode, float firstPersonFieldOfView)
        : ActorAnimation(ptr, std::move(parentNode), resourceSystem)
        , mViewMode(viewMode)
        , mShowWeapons(false)
        , mShowCarriedLeft(true)
        , mNpcType(getNpcType(ptr))
        , mFirstPersonFieldOfView(firstPersonFieldOfView)
        , mSoundsDisabled(disableSounds)
        , mAccurateAiming(false)
        , mAimingFactor(0.f)
    {
        mNpc = mPtr.get<ESM::NPC>()->mBase;

        mHeadAnimationTime = std::make_shared<HeadAnimationTime>(mPtr);
        mWeaponAnimationTime = std::make_shared<WeaponAnimationTime>(this);

        for (size_t i = 0; i < ESM::PRT_Count; i++)
        {
            mPartslots[i] = -1; // each slot is empty
            mPartPriorities[i] = 0;
        }

        std::fill(mSounds.begin(), mSounds.end(), nullptr);

        updateNpcBase();
    }

    void NpcAnimation::setViewMode(NpcAnimation::ViewMode viewMode)
    {
        assert(viewMode != VM_HeadOnly);
        if (mViewMode == viewMode)
            return;
        // FIXME: sheathing state must be consistent if the third person skeleton doesn't have the necessary node, but
        // third person skeleton is unavailable in first person view. This is a hack to avoid cosmetic issues.
        bool viewChange = mViewMode == VM_FirstPerson || viewMode == VM_FirstPerson;
        mViewMode = viewMode;
        MWBase::Environment::get().getWorld()->scaleObject(
            mPtr, mPtr.getCellRef().getScale(), true); // apply race height after view change

        mAmmunition.reset();
        rebuild();
        setRenderBin();

        if (viewChange && Settings::game().mShieldSheathing)
        {
            ESM::RefId weaponTypeId;
            MWMechanics::getActiveWeapon(mPtr, &weaponTypeId);
            showCarriedLeft(updateCarriedLeftVisible(weaponTypeId));
        }
    }

    /// @brief A RenderBin callback to clear the depth buffer before rendering.
    /// Switches depth attachments to a proxy renderbuffer, reattaches original depth then redraws first person root.
    /// This gives a complete depth buffer which can be used for postprocessing, buffer resolves as if depth was never
    /// cleared.
    class DepthClearCallback : public osgUtil::RenderBin::DrawCallback
    {
    public:
        DepthClearCallback()
        {
            mDepth = new SceneUtil::AutoDepth;
            mDepth->setWriteMask(true);

            mStateSet = new osg::StateSet;
            mStateSet->setAttributeAndModes(new osg::ColorMask(false, false, false, false), osg::StateAttribute::ON);
        }

        void drawImplementation(
            osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous) override
        {
            osg::State* state = renderInfo.getState();

            PostProcessor* postProcessor = static_cast<PostProcessor*>(renderInfo.getCurrentCamera()->getUserData());

            state->applyAttribute(mDepth);

            unsigned int frameId = state->getFrameStamp()->getFrameNumber() % 2;

            postProcessor->getFbo(PostProcessor::FBO_FirstPerson, frameId)->apply(*state);
            glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
            // color accumulation pass
            bin->drawImplementation(renderInfo, previous);

            auto primaryFBO = postProcessor->getPrimaryFbo(frameId);
            primaryFBO->apply(*state);

            postProcessor->getFbo(PostProcessor::FBO_OpaqueDepth, frameId)->apply(*state);

            // depth accumulation pass
            osg::ref_ptr<osg::StateSet> restore = bin->getStateSet();
            bin->setStateSet(mStateSet);
            bin->drawImplementation(renderInfo, previous);
            bin->setStateSet(restore);

            primaryFBO->apply(*state);

            state->checkGLErrors("after DepthClearCallback::drawImplementation");
        }

        osg::ref_ptr<osg::Depth> mDepth;
        osg::ref_ptr<osg::StateSet> mStateSet;
    };

    /// Overrides Field of View to given value for rendering the subgraph.
    /// Must be added as cull callback.
    class OverrideFieldOfViewCallback : public osg::NodeCallback
    {
    public:
        OverrideFieldOfViewCallback(float fov, RenderingManager* renderingManager)
            : mFov(fov)
            , mRenderingManager(renderingManager)
        {
        }

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            float fov, aspect, zNear, zFar;
            if (cv->getProjectionMatrix()->getPerspective(fov, aspect, zNear, zFar) && std::abs(fov - mFov) > 0.001)
            {
                fov = mFov;
                osg::ref_ptr<osg::RefMatrix> newProjectionMatrix = new osg::RefMatrix();
                newProjectionMatrix->makePerspective(fov, aspect, zNear, zFar);

                osg::Vec2f offset = mRenderingManager->getProjectionOffset();

                double offsetX = (offset.x() / cv->getViewport()->width()) * 2.0;
                double offsetY = (offset.y() / cv->getViewport()->height()) * 2.0;

                const osg::Matrix translation = osg::Matrix::translate(offsetX, offsetY, 0.0);
                newProjectionMatrix->postMult(translation);

                osg::ref_ptr<osg::RefMatrix> invertedOldMatrix = cv->getProjectionMatrix();
                invertedOldMatrix = new osg::RefMatrix(osg::RefMatrix::inverse(*invertedOldMatrix));
                osg::ref_ptr<osg::RefMatrix> viewMatrix = new osg::RefMatrix(*cv->getModelViewMatrix());
                viewMatrix->postMult(*newProjectionMatrix);
                viewMatrix->postMult(*invertedOldMatrix);
                cv->pushModelViewMatrix(viewMatrix, osg::Transform::ReferenceFrame::ABSOLUTE_RF);
                traverse(node, nv);
                cv->popModelViewMatrix();
            }
            else
                traverse(node, nv);
        }

    private:
        float mFov;
        RenderingManager* mRenderingManager;
    };

    void NpcAnimation::setRenderBin()
    {
        if (mViewMode == VM_FirstPerson)
        {
            [[maybe_unused]] static const bool prototypeAdded = [&] {
                osg::ref_ptr<osgUtil::RenderBin> depthClearBin(new osgUtil::RenderBin);
                depthClearBin->setDrawCallback(new DepthClearCallback());
                osgUtil::RenderBin::addRenderBinPrototype("DepthClear", depthClearBin);
                return true;
            }();
            mObjectRoot->getOrCreateStateSet()->setRenderBinDetails(
                RenderBin_FirstPerson, "DepthClear", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
        }
        else if (osg::StateSet* stateset = mObjectRoot->getStateSet())
            stateset->setRenderBinToInherit();
    }

    void NpcAnimation::rebuild()
    {
        mScabbard.reset();
        mHolsteredShield.reset();
        updateNpcBase();

        MWBase::Environment::get().getMechanicsManager()->forceStateUpdate(mPtr);
    }

    int NpcAnimation::getSlot(const osg::NodePath& path) const
    {
        for (int i = 0; i < ESM::PRT_Count; ++i)
        {
            const PartHolder* const part = mObjectParts[i].get();
            if (part == nullptr)
                continue;
            if (std::find(path.begin(), path.end(), part->getNode().get()) != path.end())
            {
                return mPartslots[i];
            }
        }
        return -1;
    }

    void NpcAnimation::updateNpcBase()
    {
        clearAnimSources();
        for (size_t i = 0; i < ESM::PRT_Count; i++)
            removeIndividualPart((ESM::PartReferenceType)i);

        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        const ESM::Race* race = store.get<ESM::Race>().find(mNpc->mRace);
        NpcType curType = getNpcType();
        bool isWerewolf = (curType == Type_Werewolf);
        bool isVampire = (curType == Type_Vampire);
        bool isFemale = !mNpc->isMale();

        mHeadModel.clear();
        mHairModel.clear();

        const ESM::RefId headName = isWerewolf ? ESM::RefId::stringRefId("WerewolfHead") : mNpc->mHead;
        const ESM::RefId hairName = isWerewolf ? ESM::RefId::stringRefId("WerewolfHair") : mNpc->mHair;

        if (!headName.empty())
        {
            const ESM::BodyPart* bp = store.get<ESM::BodyPart>().search(headName);
            if (bp)
                mHeadModel = Misc::ResourceHelpers::correctMeshPath(bp->mModel.getNormalized());
            else
                Log(Debug::Warning) << "Warning: Failed to load body part '" << headName << "'";
        }

        if (!hairName.empty())
        {
            const ESM::BodyPart* bp = store.get<ESM::BodyPart>().search(hairName);
            if (bp)
                mHairModel = Misc::ResourceHelpers::correctMeshPath(bp->mModel.getNormalized());
            else
                Log(Debug::Warning) << "Warning: Failed to load body part '" << hairName << "'";
        }

        if (!isWerewolf && isVampire)
            if (VFS::Path::Normalized vampireHead = getVampireHead(mNpc->mRace, isFemale); !vampireHead.empty())
                mHeadModel = std::move(vampireHead);

        bool is1stPerson = mViewMode == VM_FirstPerson;
        bool isBeast = (race->mData.mFlags & ESM::Race::Beast) != 0;

        std::string_view base;
        if (!isWerewolf)
        {
            if (!is1stPerson)
                base = Settings::models().mXbaseanim.get().value();
            else
                base = Settings::models().mXbaseanim1st.get().value();
        }

        const std::string defaultSkeleton = Misc::ResourceHelpers::correctActorModelPath(
            VFS::Path::toNormalized(getActorSkeleton(is1stPerson, isFemale, isBeast, isWerewolf)),
            mResourceSystem->getVFS());

        std::string smodel = defaultSkeleton;
        bool isCustomModel = false;
        if (!is1stPerson && !isWerewolf && !mNpc->mModel.empty())
        {
            const VFS::Path::Normalized model = Misc::ResourceHelpers::correctMeshPath(mNpc->mModel.getNormalized());
            isCustomModel = !isDefaultActorSkeleton(model);
            smodel = Misc::ResourceHelpers::correctActorModelPath(model, mResourceSystem->getVFS());
        }

        setObjectRoot(smodel, true, true, false);

        updateParts();

        if (!base.empty())
            addAnimSource(base, smodel);

        if (defaultSkeleton != base)
            addAnimSource(defaultSkeleton, smodel);

        if (isCustomModel)
            addAnimSource(smodel, smodel);

        const bool customArgonianSwim = !is1stPerson && !isWerewolf && isBeast && mNpc->mRace.contains("argonian");
        if (customArgonianSwim)
            addAnimSource(Settings::models().mXargonianswimkna.get().value(), smodel);

        if (is1stPerson)
        {
            mObjectRoot->setNodeMask(Mask_FirstPerson);
            mObjectRoot->addCullCallback(new OverrideFieldOfViewCallback(
                mFirstPersonFieldOfView, MWBase::Environment::get().getWorld()->getRenderingManager()));
        }

        mWeaponAnimationTime->updateStartTime();
    }

    std::string NpcAnimation::getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const
    {
        std::string mesh = getShieldMesh(shield, !mNpc->isMale());

        if (mesh.empty())
            return std::string();

        const VFS::Path::Normalized holsteredName(addSuffixBeforeExtension(mesh, "_sh"));
        if (mResourceSystem->getVFS()->exists(holsteredName))
        {
            osg::ref_ptr<osg::Node> shieldTemplate = mResourceSystem->getSceneManager()->getInstance(holsteredName);
            SceneUtil::FindByNameVisitor findVisitor("Bip01 Sheath");
            shieldTemplate->accept(findVisitor);
            osg::ref_ptr<osg::Node> sheathNode = findVisitor.mFoundNode;
            if (!sheathNode)
                return std::string();
        }

        return mesh;
    }

    void NpcAnimation::updateParts()
    {
        if (!mObjectRoot.get())
            return;

        // The cut owns the hidden originals. Restore their masks before
        // replacing clothing/armor nodes; otherwise an old cap can outlive
        // the part it was built from and produce orphaned geometry.
        const bool rebuildHeadCut = static_cast<bool>(mBrutalHeadCut);
        const unsigned rebuildLimbMask = mBrutalLimbCut ? mBrutalLimbCut->mask : 0u;
        if (rebuildHeadCut)
        {
            mBrutalRebuildHeadInNeck = mBrutalHeadCut->headInNeck;
            mBrutalHeadCut.reset();
        }
        if (rebuildLimbMask)
            mBrutalLimbCut.reset();

        NpcType curType = getNpcType();
        if (curType != mNpcType)
        {
            mNpcType = curType;
            mBrutalRebuildHeadInNeck.reset();
            rebuild();
            return;
        }

        static const struct
        {
            int mSlot;
            int mBasePriority;
        } slotlist[] = { // FIXME: Priority is based on the number of reserved slots. There should be a better way.
            { MWWorld::InventoryStore::Slot_Robe, 11 }, { MWWorld::InventoryStore::Slot_Skirt, 3 },
            { MWWorld::InventoryStore::Slot_Helmet, 0 }, { MWWorld::InventoryStore::Slot_Cuirass, 0 },
            { MWWorld::InventoryStore::Slot_Greaves, 0 }, { MWWorld::InventoryStore::Slot_LeftPauldron, 0 },
            { MWWorld::InventoryStore::Slot_RightPauldron, 0 }, { MWWorld::InventoryStore::Slot_Boots, 0 },
            { MWWorld::InventoryStore::Slot_LeftGauntlet, 0 }, { MWWorld::InventoryStore::Slot_RightGauntlet, 0 },
            { MWWorld::InventoryStore::Slot_Shirt, 0 }, { MWWorld::InventoryStore::Slot_Pants, 0 },
            { MWWorld::InventoryStore::Slot_CarriedLeft, 0 }, { MWWorld::InventoryStore::Slot_CarriedRight, 0 }
        };
        static const size_t slotlistsize = sizeof(slotlist) / sizeof(slotlist[0]);

        bool wasArrowAttached = isArrowAttached();
        mAmmunition.reset();

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        for (size_t i = 0; i < slotlistsize && mViewMode != VM_HeadOnly; i++)
        {
            MWWorld::ConstContainerStoreIterator store = inv.getSlot(slotlist[i].mSlot);

            removePartGroup(slotlist[i].mSlot);

            if (store == inv.end())
                continue;

            if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Helmet)
                removeIndividualPart(ESM::PRT_Hair);

            int prio = 1;
            bool enchantedGlow = !store->getClass().getEnchantment(*store).empty();
            osg::Vec4f glowColor = store->getClass().getEnchantmentColor(*store);
            if (store->getType() == ESM::Clothing::sRecordId)
            {
                prio = ((slotlist[i].mBasePriority + 1) << 1) + 0;
                const ESM::Clothing* clothes = store->get<ESM::Clothing>()->mBase;
                addPartGroup(slotlist[i].mSlot, prio, clothes->mParts.mParts, enchantedGlow, &glowColor);
            }
            else if (store->getType() == ESM::Armor::sRecordId)
            {
                prio = ((slotlist[i].mBasePriority + 1) << 1) + 1;
                const ESM::Armor* armor = store->get<ESM::Armor>()->mBase;
                addPartGroup(slotlist[i].mSlot, prio, armor->mParts.mParts, enchantedGlow, &glowColor);
            }

            if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Robe)
            {
                ESM::PartReferenceType parts[] = { ESM::PRT_Groin, ESM::PRT_Skirt, ESM::PRT_RLeg, ESM::PRT_LLeg,
                    ESM::PRT_RUpperarm, ESM::PRT_LUpperarm, ESM::PRT_RKnee, ESM::PRT_LKnee, ESM::PRT_RForearm,
                    ESM::PRT_LForearm, ESM::PRT_Cuirass };
                const size_t partsSize = sizeof(parts) / sizeof(parts[0]);
                for (size_t p = 0; p < partsSize; ++p)
                    reserveIndividualPart(parts[p], slotlist[i].mSlot, prio);
            }
            else if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Skirt)
            {
                reserveIndividualPart(ESM::PRT_Groin, slotlist[i].mSlot, prio);
                reserveIndividualPart(ESM::PRT_RLeg, slotlist[i].mSlot, prio);
                reserveIndividualPart(ESM::PRT_LLeg, slotlist[i].mSlot, prio);
            }
        }

        if (mViewMode != VM_FirstPerson)
        {
            if (mPartPriorities[ESM::PRT_Head] < 1 && !mHeadModel.empty())
                addOrReplaceIndividualPart(ESM::PRT_Head, -1, 1, mHeadModel);
            if (mPartPriorities[ESM::PRT_Hair] < 1 && mPartPriorities[ESM::PRT_Head] <= 1 && !mHairModel.empty())
                addOrReplaceIndividualPart(ESM::PRT_Hair, -1, 1, mHairModel);
        }
        if (mViewMode == VM_HeadOnly)
            return;

        if (mPartPriorities[ESM::PRT_Shield] < 1)
        {
            MWWorld::ConstContainerStoreIterator store = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            MWWorld::ConstPtr part;
            if (store != inv.end() && (part = *store).getType() == ESM::Light::sRecordId)
            {
                const ESM::Light* light = part.get<ESM::Light>()->mBase;
                addOrReplaceIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1,
                    Misc::ResourceHelpers::correctMeshPath(light->mModel.getNormalized()), false, nullptr, true);
                if (mObjectParts[ESM::PRT_Shield])
                    addExtraLight(mObjectParts[ESM::PRT_Shield]->getNode()->asGroup(), SceneUtil::LightCommon(*light));
            }
        }

        showWeapons(mShowWeapons);
        showCarriedLeft(mShowCarriedLeft);

        bool isWerewolf = (getNpcType() == Type_Werewolf);
        ESM::RefId race = (isWerewolf ? ESM::RefId::stringRefId("werewolf") : mNpc->mRace);

        const std::vector<const ESM::BodyPart*>& parts
            = getBodyParts(race, !mNpc->isMale(), mViewMode == VM_FirstPerson, isWerewolf);
        for (int part = ESM::PRT_Neck; part < ESM::PRT_Count; ++part)
        {
            if (mPartPriorities[part] < 1)
            {
                if (const ESM::BodyPart* bodypart = parts[part])
                    addOrReplaceIndividualPart(static_cast<ESM::PartReferenceType>(part), -1, 1,
                        Misc::ResourceHelpers::correctMeshPath(bodypart->mModel.getNormalized()));
            }
        }

        if (wasArrowAttached)
            attachArrow();

        if (rebuildHeadCut && !setBrutalHeadSevered(true))
            Log(Debug::Warning)
                << "Brutal head cut could not be rebuilt after NPC equipment changed";
        mBrutalRebuildHeadInNeck.reset();
        if (rebuildLimbMask && !setBrutalLimbsSevered(rebuildLimbMask))
            Log(Debug::Warning)
                << "Brutal limb cut could not be rebuilt after NPC equipment changed";
    }

    PartHolderPtr NpcAnimation::insertBoundedPart(VFS::Path::NormalizedView model, std::string_view bonename,
        std::string_view bonefilter, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight)
    {
        osg::ref_ptr<osg::Node> attached = attach(model, bonename, bonefilter, isLight);
        if (enchantedGlow)
            mGlowUpdater = SceneUtil::addEnchantedGlow(attached, mResourceSystem, *glowColor);

        return std::make_unique<PartHolder>(std::move(attached), model);
    }

    osg::Vec3f NpcAnimation::runAnimation(float timepassed)
    {
        osg::Vec3f ret = Animation::runAnimation(timepassed);

        mHeadAnimationTime->update(timepassed);

        if (mFirstPersonNeckController)
        {
            if (mAccurateAiming)
                mAimingFactor = 1.f;
            else
                mAimingFactor = std::max(0.f, mAimingFactor - timepassed * 0.5f);

            float rotateFactor = 0.75f + 0.25f * mAimingFactor;

            mFirstPersonNeckController->setRotate(
                osg::Quat(mPtr.getRefData().getPosition().rot[0] * rotateFactor, osg::Vec3f(-1, 0, 0)));
            mFirstPersonNeckController->setOffset(mFirstPersonOffset);
        }

        WeaponAnimation::configureControllers(mPtr.getRefData().getPosition().rot[0] + getBodyPitchRadians());

        return ret;
    }

    void NpcAnimation::removeIndividualPart(ESM::PartReferenceType type)
    {
        mPartPriorities[type] = 0;
        mPartslots[type] = -1;

        mObjectParts[type].reset();
        if (mSounds[type] != nullptr && !mSoundsDisabled)
        {
            MWBase::Environment::get().getSoundManager()->stopSound(mSounds[type]);
            mSounds[type] = nullptr;
        }
    }

    void NpcAnimation::reserveIndividualPart(ESM::PartReferenceType type, int group, int priority)
    {
        if (priority > mPartPriorities[type])
        {
            removeIndividualPart(type);
            mPartPriorities[type] = priority;
            mPartslots[type] = group;
        }
    }

    void NpcAnimation::removePartGroup(int group)
    {
        for (int i = 0; i < ESM::PRT_Count; i++)
        {
            if (mPartslots[i] == group)
                removeIndividualPart((ESM::PartReferenceType)i);
        }
    }

    bool NpcAnimation::isFemalePart(const ESM::BodyPart* bodypart)
    {
        return bodypart->mData.mFlags & ESM::BodyPart::BPF_Female;
    }

    bool NpcAnimation::addOrReplaceIndividualPart(ESM::PartReferenceType type, int group, int priority,
        VFS::Path::NormalizedView mesh, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight)
    {
        if (priority <= mPartPriorities[type])
            return false;

        removeIndividualPart(type);
        mPartslots[type] = group;
        mPartPriorities[type] = priority;
        try
        {
            std::string_view bonename = sPartList.at(type);
            if (type == ESM::PRT_Weapon)
            {
                const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
                MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (weapon != inv.end() && weapon->getType() == ESM::Weapon::sRecordId)
                {
                    const ESM::RefId weaponTypeId = weapon->get<ESM::Weapon>()->mBase->mData.mType;
                    const std::string& weaponBonename = MWMechanics::getWeaponType(weaponTypeId)->mAttachBone;

                    if (weaponBonename != bonename)
                    {
                        const NodeMap& nodeMap = getNodeMap();
                        NodeMap::const_iterator found = nodeMap.find(weaponBonename);
                        if (found != nodeMap.end())
                            bonename = weaponBonename;
                    }
                }
            }

            // PRT_Hair seems to be the only type that breaks consistency and uses a filter that's different from the
            // attachment bone
            const std::string_view bonefilter = (type == ESM::PRT_Hair) ? std::string_view{ "hair" } : bonename;
            mObjectParts[type] = insertBoundedPart(mesh, bonename, bonefilter, enchantedGlow, glowColor, isLight);
        }
        catch (std::exception& e)
        {
            Log(Debug::Error) << "Error adding NPC part: " << e.what();
            return false;
        }

        if (!mSoundsDisabled && group == MWWorld::InventoryStore::Slot_CarriedLeft)
        {
            const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
            MWWorld::ConstContainerStoreIterator csi = inv.getSlot(group);
            if (csi != inv.end())
            {
                const auto soundId = csi->getClass().getSound(*csi);
                if (!soundId.empty())
                {
                    mSounds[type] = MWBase::Environment::get().getSoundManager()->playSound3D(
                        mPtr, soundId, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
                }
            }
        }

        osg::Node* node = mObjectParts[type]->getNode();
        if (node->getNumChildrenRequiringUpdateTraversal() > 0)
        {
            std::shared_ptr<SceneUtil::ControllerSource> src;
            if (type == ESM::PRT_Head)
            {
                src = mHeadAnimationTime;

                if (const SceneUtil::TextKeyMap* keys = SceneUtil::findUserData<SceneUtil::TextKeyMap>(*node))
                {
                    for (const auto& key : *keys)
                    {
                        if (Misc::StringUtils::ciEqual(key.second, "talk: start"))
                            mHeadAnimationTime->setTalkStart(key.first);
                        if (Misc::StringUtils::ciEqual(key.second, "talk: stop"))
                            mHeadAnimationTime->setTalkStop(key.first);
                        if (Misc::StringUtils::ciEqual(key.second, "blink: start"))
                            mHeadAnimationTime->setBlinkStart(key.first);
                        if (Misc::StringUtils::ciEqual(key.second, "blink: stop"))
                            mHeadAnimationTime->setBlinkStop(key.first);
                    }
                }
                SceneUtil::ForceControllerSourcesVisitor assignVisitor(std::move(src));
                node->accept(assignVisitor);
            }
            else
            {
                if (type == ESM::PRT_Weapon)
                    src = mWeaponAnimationTime;
                else
                    src = mAnimationTimePtr[0];
                SceneUtil::AssignControllerSourcesVisitor assignVisitor(std::move(src));
                node->accept(assignVisitor);
            }
        }

        return true;
    }

    void NpcAnimation::addPartGroup(int group, int priority, const std::vector<ESM::PartReference>& parts,
        bool enchantedGlow, osg::Vec4f* glowColor)
    {
        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        const MWWorld::Store<ESM::BodyPart>& partStore = store.get<ESM::BodyPart>();

        const char* ext = (mViewMode == VM_FirstPerson) ? ".1st" : "";
        for (const ESM::PartReference& part : parts)
        {
            const ESM::BodyPart* bodypart = nullptr;
            if (!mNpc->isMale() && !part.mFemale.empty())
            {
                bodypart = partStore.search(ESM::RefId::stringRefId(part.mFemale.getRefIdString() + ext));
                if (!bodypart && mViewMode == VM_FirstPerson)
                {
                    bodypart = partStore.search(part.mFemale);
                    if (bodypart
                        && !(bodypart->mData.mPart == ESM::BodyPart::MP_Hand
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Wrist
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Forearm
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Upperarm))
                        bodypart = nullptr;
                }
                else if (!bodypart)
                    Log(Debug::Warning) << "Warning: Failed to find body part '" << part.mFemale << "'";
            }
            if (!bodypart && !part.mMale.empty())
            {
                bodypart = partStore.search(ESM::RefId::stringRefId(part.mMale.getRefIdString() + ext));
                if (!bodypart && mViewMode == VM_FirstPerson)
                {
                    bodypart = partStore.search(part.mMale);
                    if (bodypart
                        && !(bodypart->mData.mPart == ESM::BodyPart::MP_Hand
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Wrist
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Forearm
                            || bodypart->mData.mPart == ESM::BodyPart::MP_Upperarm))
                        bodypart = nullptr;
                }
                else if (!bodypart)
                    Log(Debug::Warning) << "Warning: Failed to find body part '" << part.mMale << "'";
            }

            if (bodypart)
                addOrReplaceIndividualPart(static_cast<ESM::PartReferenceType>(part.mPart), group, priority,
                    Misc::ResourceHelpers::correctMeshPath(bodypart->mModel.getNormalized()), enchantedGlow, glowColor);
            else
                reserveIndividualPart((ESM::PartReferenceType)part.mPart, group, priority);
        }
    }

    void NpcAnimation::addControllers()
    {
        Animation::addControllers();

        mFirstPersonNeckController = nullptr;
        WeaponAnimation::deleteControllers();

        if (mViewMode == VM_FirstPerson)
        {
            // If there is no active animation, then the bip01 neck node will not be updated each frame, and the
            // RotateController will accumulate rotations.
            if (mStates.size() > 0)
            {
                NodeMap::iterator found = mNodeMap.find("bip01 neck");
                if (found != mNodeMap.end())
                {
                    osg::MatrixTransform* node = found->second.get();
                    mFirstPersonNeckController = new RotateController(mObjectRoot.get());
                    node->addUpdateCallback(mFirstPersonNeckController);
                    mActiveControllers.emplace_back(node, mFirstPersonNeckController);
                }
            }
        }
        else if (mViewMode == VM_Normal)
        {
            WeaponAnimation::addControllers(mNodeMap, mActiveControllers, mObjectRoot.get());
        }
    }

    void NpcAnimation::showWeapons(bool showWeapon)
    {
        mShowWeapons = showWeapon;
        mAmmunition.reset();
        if (showWeapon)
        {
            const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
            MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (weapon != inv.end())
            {
                osg::Vec4f glowColor = weapon->getClass().getEnchantmentColor(*weapon);
                const VFS::Path::Normalized mesh = weapon->getClass().getCorrectedModel(*weapon);
                addOrReplaceIndividualPart(ESM::PRT_Weapon, MWWorld::InventoryStore::Slot_CarriedRight, 1, mesh,
                    !weapon->getClass().getEnchantment(*weapon).empty(), &glowColor);

                // Crossbows start out with a bolt attached
                if (weapon->getType() == ESM::Weapon::sRecordId
                    && weapon->get<ESM::Weapon>()->mBase->mData.mType == ESM::WeaponType::MarksmanCrossbow)
                {
                    const ESM::RefId ammotype
                        = MWMechanics::getWeaponType(ESM::WeaponType::MarksmanCrossbow)->mAmmoType;
                    MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
                    if (ammo != inv.end() && ammo->get<ESM::Weapon>()->mBase->mData.mType == ammotype)
                        attachArrow();
                }
            }
            else
                removeIndividualPart(ESM::PRT_Weapon);
        }
        else
        {
            removeIndividualPart(ESM::PRT_Weapon);
            // If we remove/hide weapon from player, we should reset attack animation as well
            if (mPtr == MWMechanics::getPlayer())
                mPtr.getClass().getCreatureStats(mPtr).setAttackingOrSpell(false);
        }

        updateHolsteredWeapon(!mShowWeapons);
        updateQuiver();
    }

    bool NpcAnimation::updateCarriedLeftVisible(ESM::RefId weaptype) const
    {
        if (Settings::game().mShieldSheathing)
        {
            const MWWorld::Class& cls = mPtr.getClass();
            MWMechanics::CreatureStats& stats = cls.getCreatureStats(mPtr);
            if (stats.getDrawState() == MWMechanics::DrawState::Nothing)
            {
                SceneUtil::FindByNameVisitor findVisitor("Bip01 AttachShield");
                mObjectRoot->accept(findVisitor);
                if (findVisitor.mFoundNode || mViewMode == VM_FirstPerson)
                {
                    const MWWorld::InventoryStore& inv = cls.getInventoryStore(mPtr);
                    const MWWorld::ConstContainerStoreIterator shield
                        = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                    if (shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                        && !getSheathedShieldMesh(*shield).empty())
                        return false;
                }
            }
        }

        return !(MWMechanics::getWeaponType(weaptype)->mFlags & ESM::WeaponType::TwoHanded);
    }

    void NpcAnimation::showCarriedLeft(bool show)
    {
        mShowCarriedLeft = show;
        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator iter = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (show && iter != inv.end())
        {
            osg::Vec4f glowColor = iter->getClass().getEnchantmentColor(*iter);
            VFS::Path::Normalized mesh = iter->getClass().getCorrectedModel(*iter);
            // For shields we must try to use the body part model
            if (iter->getType() == ESM::Armor::sRecordId)
            {
                mesh = getShieldMesh(*iter, !mNpc->isMale());
            }
            if (mesh.empty()
                || addOrReplaceIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1, mesh,
                    !iter->getClass().getEnchantment(*iter).empty(), &glowColor,
                    iter->getType() == ESM::Light::sRecordId))
            {
                if (mesh.empty())
                    reserveIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1);
                if (iter->getType() == ESM::Light::sRecordId && mObjectParts[ESM::PRT_Shield])
                    addExtraLight(mObjectParts[ESM::PRT_Shield]->getNode()->asGroup(),
                        SceneUtil::LightCommon(*iter->get<ESM::Light>()->mBase));
            }
        }
        else
            removeIndividualPart(ESM::PRT_Shield);

        updateHolsteredShield(mShowCarriedLeft);
    }

    void NpcAnimation::attachArrow()
    {
        WeaponAnimation::attachArrow(mPtr);

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
        if (ammo != inv.end() && !ammo->getClass().getEnchantment(*ammo).empty())
        {
            osg::Group* bone = getArrowBone();
            if (bone != nullptr && bone->getNumChildren())
                SceneUtil::addEnchantedGlow(
                    bone->getChild(0), mResourceSystem, ammo->getClass().getEnchantmentColor(*ammo));
        }

        updateQuiver();
    }

    void NpcAnimation::detachArrow()
    {
        WeaponAnimation::detachArrow(mPtr);
        updateQuiver();
    }

    void NpcAnimation::releaseArrow(float attackStrength, float attackWindUp)
    {
        WeaponAnimation::releaseArrow(mPtr, attackStrength, attackWindUp);
        updateQuiver();
    }

    osg::Group* NpcAnimation::getArrowBone()
    {
        const PartHolder* const part = mObjectParts[ESM::PRT_Weapon].get();
        if (part == nullptr)
            return nullptr;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return nullptr;

        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        const ESM::RefId ammoType = MWMechanics::getWeaponType(type)->mAmmoType;
        if (ammoType.empty())
            return nullptr;

        // Try to find and attachment bone in actor's skeleton, otherwise fall back to the ArrowBone in weapon's mesh
        osg::Group* bone = getBoneByName(MWMechanics::getWeaponType(ammoType)->mAttachBone);
        if (bone == nullptr)
        {
            SceneUtil::FindByNameVisitor findVisitor("ArrowBone");
            part->getNode()->accept(findVisitor);
            bone = findVisitor.mFoundNode;
        }
        return bone;
    }

    osg::Node* NpcAnimation::getWeaponNode()
    {
        const PartHolder* const part = mObjectParts[ESM::PRT_Weapon].get();
        if (part == nullptr)
            return nullptr;
        return part->getNode();
    }

    Resource::ResourceSystem* NpcAnimation::getResourceSystem()
    {
        return mResourceSystem;
    }

    void NpcAnimation::enableHeadAnimation(bool enable)
    {
        mHeadAnimationTime->setEnabled(enable);
    }

    void NpcAnimation::setWeaponGroup(const std::string& group, bool relativeDuration)
    {
        mWeaponAnimationTime->setGroup(group, relativeDuration);
    }

    void NpcAnimation::equipmentChanged()
    {
        if (Settings::game().mShieldSheathing)
        {
            ESM::RefId weaponTypeId;
            MWMechanics::getActiveWeapon(mPtr, &weaponTypeId);
            showCarriedLeft(updateCarriedLeftVisible(weaponTypeId));
        }

        updateParts();
    }

    void NpcAnimation::setVampire(bool vampire)
    {
        if (mNpcType == Type_Werewolf) // we can't have werewolf vampires, can we
            return;
        if ((mNpcType == Type_Vampire) != vampire)
        {
            if (mPtr == MWMechanics::getPlayer())
                MWBase::Environment::get().getWorld()->reattachPlayerCamera();
            else
                rebuild();
        }
    }

    void NpcAnimation::setFirstPersonOffset(const osg::Vec3f& offset)
    {
        mFirstPersonOffset = offset;
    }

    void NpcAnimation::updatePtr(const MWWorld::Ptr& updated)
    {
        Animation::updatePtr(updated);
        mHeadAnimationTime->updatePtr(updated);
    }

    // Remember body parts so we only have to search through the store once for each race/gender/viewmode combination
    typedef std::map<std::pair<ESM::RefId, int>, std::vector<const ESM::BodyPart*>> RaceMapping;
    static RaceMapping sRaceMapping;

    const std::vector<const ESM::BodyPart*>& NpcAnimation::getBodyParts(
        const ESM::RefId& race, bool female, bool firstPerson, bool werewolf)
    {
        constexpr int flagFirstPerson = 1 << 1;
        constexpr int flagFemale = 1 << 0;

        int flags = (werewolf ? -1 : 0);
        if (female)
            flags |= flagFemale;
        if (firstPerson)
            flags |= flagFirstPerson;

        RaceMapping::iterator found = sRaceMapping.find(std::make_pair(race, flags));
        if (found != sRaceMapping.end())
            return found->second;
        else
        {
            std::vector<const ESM::BodyPart*>& parts = sRaceMapping[std::make_pair(race, flags)];

            typedef std::multimap<ESM::BodyPart::MeshPart, ESM::PartReferenceType> BodyPartMapType;
            static const BodyPartMapType sBodyPartMap = { { ESM::BodyPart::MP_Neck, ESM::PRT_Neck },
                { ESM::BodyPart::MP_Chest, ESM::PRT_Cuirass }, { ESM::BodyPart::MP_Groin, ESM::PRT_Groin },
                { ESM::BodyPart::MP_Hand, ESM::PRT_RHand }, { ESM::BodyPart::MP_Hand, ESM::PRT_LHand },
                { ESM::BodyPart::MP_Wrist, ESM::PRT_RWrist }, { ESM::BodyPart::MP_Wrist, ESM::PRT_LWrist },
                { ESM::BodyPart::MP_Forearm, ESM::PRT_RForearm }, { ESM::BodyPart::MP_Forearm, ESM::PRT_LForearm },
                { ESM::BodyPart::MP_Upperarm, ESM::PRT_RUpperarm }, { ESM::BodyPart::MP_Upperarm, ESM::PRT_LUpperarm },
                { ESM::BodyPart::MP_Foot, ESM::PRT_RFoot }, { ESM::BodyPart::MP_Foot, ESM::PRT_LFoot },
                { ESM::BodyPart::MP_Ankle, ESM::PRT_RAnkle }, { ESM::BodyPart::MP_Ankle, ESM::PRT_LAnkle },
                { ESM::BodyPart::MP_Knee, ESM::PRT_RKnee }, { ESM::BodyPart::MP_Knee, ESM::PRT_LKnee },
                { ESM::BodyPart::MP_Upperleg, ESM::PRT_RLeg }, { ESM::BodyPart::MP_Upperleg, ESM::PRT_LLeg },
                { ESM::BodyPart::MP_Tail, ESM::PRT_Tail } };

            parts.resize(ESM::PRT_Count, nullptr);

            if (werewolf)
                return parts;

            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

            for (const ESM::BodyPart& bodypart : store.get<ESM::BodyPart>())
            {
                if (bodypart.mData.mFlags & ESM::BodyPart::BPF_NotPlayable)
                    continue;
                if (bodypart.mData.mType != ESM::BodyPart::MT_Skin)
                    continue;

                if (!(bodypart.mRace == race))
                    continue;

                const bool partFirstPerson = ESM::isFirstPersonBodyPart(bodypart);

                bool isHand = bodypart.mData.mPart == ESM::BodyPart::MP_Hand
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Wrist
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Forearm
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Upperarm;

                bool isSameGender = isFemalePart(&bodypart) == female;

                /* A fallback for the arms if 1st person is missing:
                 1. Try to use 3d person skin for same gender
                 2. Try to use 1st person skin for male, if female == true
                 3. Try to use 3d person skin for male, if female == true

                 A fallback in another cases: allow to use male bodyparts, if female == true
                */
                if (firstPerson && isHand && !partFirstPerson)
                {
                    // Allow 3rd person skins as a fallback for the arms if 1st person is missing
                    BodyPartMapType::const_iterator bIt
                        = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                    while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                    {
                        // If we have no fallback bodypart now and bodypart is for same gender (1)
                        if (!parts[bIt->second] && isSameGender)
                            parts[bIt->second] = &bodypart;

                        // If we have fallback bodypart for other gender and found fallback for current gender (1)
                        else if (isSameGender && isFemalePart(parts[bIt->second]) != female)
                            parts[bIt->second] = &bodypart;

                        // If we have no fallback bodypart and searching for female bodyparts (3)
                        else if (!parts[bIt->second] && female)
                            parts[bIt->second] = &bodypart;

                        ++bIt;
                    }

                    continue;
                }

                // Don't allow to use podyparts for a different view
                if (partFirstPerson != firstPerson)
                    continue;

                if (female && !isFemalePart(&bodypart))
                {
                    // Allow male parts as fallback for females if female parts are missing
                    BodyPartMapType::const_iterator bIt
                        = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                    while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                    {
                        // If we have no fallback bodypart now
                        if (!parts[bIt->second])
                            parts[bIt->second] = &bodypart;

                        // If we have 3d person fallback bodypart for hand and 1st person fallback found (2)
                        else if (isHand && !ESM::isFirstPersonBodyPart(*parts[bIt->second]) && partFirstPerson)
                            parts[bIt->second] = &bodypart;

                        ++bIt;
                    }

                    continue;
                }

                // Don't allow to use podyparts for another gender
                if (female != isFemalePart(&bodypart))
                    continue;

                // Use properly found bodypart, replacing fallbacks
                BodyPartMapType::const_iterator bIt
                    = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                {
                    parts[bIt->second] = &bodypart;
                    ++bIt;
                }
            }
            return parts;
        }
    }

    void NpcAnimation::setAccurateAiming(bool enabled)
    {
        mAccurateAiming = enabled;
    }

    bool NpcAnimation::isArrowAttached() const
    {
        return mAmmunition != nullptr;
    }

}
