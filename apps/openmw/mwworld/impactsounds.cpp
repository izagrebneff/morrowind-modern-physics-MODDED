#include "impactsounds.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include <components/debug/debuglog.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/settings/values.hpp>
#include <components/vfs/manager.hpp>

#include "../mwbase/environment.hpp"

#include "../mwphysics/physicssystem.hpp"

#include "../mwrender/renderingmanager.hpp"

#include "class.hpp"
#include "ptr.hpp"

namespace MWWorld
{
    namespace
    {
        constexpr std::string_view configPath = "modernphysics/impactsounds.yaml";

        template <class T>
        void read(const YAML::Node& node, const char* key, T& value)
        {
            if (const YAML::Node child = node[key])
                value = child.as<T>();
        }

        Material materialOf(const Ptr& ptr)
        {
            if (ptr.getClass().isActor())
                return Material::Flesh;
            return classifyItem(ptr);
        }

        std::string nameOf(const Ptr& ptr)
        {
            return ptr.isEmpty() ? std::string("terrain") : ptr.getCellRef().getRefId().toDebugString();
        }
    }

    ImpactSounds& ImpactSounds::get()
    {
        static ImpactSounds instance;
        return instance;
    }

    void ImpactSounds::loadConfig()
    {
        const VFS::Manager& vfs = *MWBase::Environment::get().getResourceSystem()->getVFS();
        const VFS::Path::Normalized path(configPath);
        if (!vfs.exists(path))
        {
            if (!mConfigLoaded)
                Log(Debug::Warning) << "Impact sounds: " << configPath << " not found; using defaults";
            mConfigLoaded = true;
            return;
        }
        mConfigTime = vfs.getLastModified(path);
        mConfigLoaded = true;
        try
        {
            std::stringstream text;
            text << vfs.get(path)->rdbuf();
            const YAML::Node root = YAML::Load(text.str());
            Limits limits;
            if (const YAML::Node node = root["limits"])
            {
                read(node, "min speed", limits.mMinSpeed);
                read(node, "loud speed", limits.mLoudSpeed);
                read(node, "object cooldown", limits.mObjectCooldown);
                read(node, "max sounds", limits.mMaxSounds);
                read(node, "max sounds window", limits.mMaxSoundsWindow);
            }
            mLimits = limits;
            Log(Debug::Info) << "Impact sounds: loaded " << configPath;
        }
        catch (const std::exception& e)
        {
            // Keep what was there: a typo while tuning shouldn't silence everything.
            Log(Debug::Error) << "Impact sounds: can't read " << configPath << ": " << e.what();
        }
    }

    void ImpactSounds::update(
        float duration, MWPhysics::PhysicsSystem& physics, MWRender::RenderingManager& rendering)
    {
        mTime += duration;
        const bool debug = Settings::modernPhysics().mImpactSoundDebug;
        if (!mConfigLoaded)
            loadConfig();
        else if (debug && (mConfigCheckTimer -= duration) <= 0)
        {
            // Tuning: pick up the file as soon as it is saved.
            mConfigCheckTimer = 1.f;
            const VFS::Manager& vfs = *MWBase::Environment::get().getResourceSystem()->getVFS();
            const VFS::Path::Normalized path(configPath);
            if (vfs.exists(path) && vfs.getLastModified(path) != mConfigTime)
                loadConfig();
        }

        const bool objects = Settings::modernPhysics().mImpactSounds && Settings::modernPhysics().mObjectImpactSounds;
        physics.setImpactMinSpeed(objects ? std::max(mLimits.mMinSpeed, 1.f) : 0.f);
        for (const MWPhysics::Impact& impact : physics.takeImpacts())
            if (objects)
                handleObjectImpact(impact, rendering);

        // Forget objects whose cooldown is long over.
        if (mLastSound.size() > 256)
            std::erase_if(mLastSound, [&](const auto& entry) { return mTime - entry.second > 5.f; });
    }

    void ImpactSounds::handleObjectImpact(const MWPhysics::Impact& impact, MWRender::RenderingManager& rendering)
    {
        const Material bodyMaterial = materialOf(impact.mBody);
        Material otherMaterial;
        if (impact.mOtherSimulated && !impact.mOther.isEmpty())
            otherMaterial = materialOf(impact.mOther);
        else
        {
            // A surface: by what it looks like right there (the texture), else its model. Look past the body.
            SceneUtil::PositionAttitudeTransform* bodyNode = impact.mBody.getRefData().getBaseNode();
            const osg::Node::NodeMask bodyMask = bodyNode != nullptr ? bodyNode->getNodeMask() : 0;
            if (bodyNode != nullptr)
                bodyNode->setNodeMask(0);
            const MWRender::RenderingManager::SurfaceResult surface
                = rendering.castRayForSurface(impact.mPoint + impact.mNormal * 16.f, impact.mPoint - impact.mNormal * 16.f);
            if (bodyNode != nullptr)
                bodyNode->setNodeMask(bodyMask);
            const std::string model
                = impact.mOther.isEmpty() ? std::string() : std::string(impact.mOther.getClass().getCorrectedModel(impact.mOther));
            const bool terrain = impact.mOtherTerrain || surface.mTerrain;
            otherMaterial = classifySurface(surface.mHit ? surface.mTexture : std::string(), model,
                terrain ? Material::Earth : Material::Stone);
        }

        std::string why;
        const bool allowed = allow({ impact.mBody.mRef, impact.mOtherSimulated ? impact.mOther.mRef : nullptr }, why);
        log(std::format("object {} ({}) on {} ({}{}) speed {:.0f} mass {:.1f} at {:.0f} {:.0f} {:.0f}: {}",
            nameOf(impact.mBody), getMaterialName(bodyMaterial), nameOf(impact.mOther), getMaterialName(otherMaterial),
            impact.mOtherSimulated ? ", simulated" : "", impact.mSpeed, impact.mBodyMass, impact.mPoint.x(),
            impact.mPoint.y(), impact.mPoint.z(), allowed ? "ok" : why));
    }

    bool ImpactSounds::allow(std::initializer_list<const void*> keys, std::string& why)
    {
        for (const void* key : keys)
        {
            if (key == nullptr)
                continue;
            const auto found = mLastSound.find(key);
            if (found != mLastSound.end() && mTime - found->second < mLimits.mObjectCooldown)
            {
                why = "cooldown";
                return false;
            }
        }
        while (!mRecentSounds.empty() && mTime - mRecentSounds.front() > mLimits.mMaxSoundsWindow)
            mRecentSounds.pop_front();
        if (static_cast<int>(mRecentSounds.size()) >= mLimits.mMaxSounds)
        {
            why = "too many at once";
            return false;
        }
        mRecentSounds.push_back(mTime);
        for (const void* key : keys)
            if (key != nullptr)
                mLastSound[key] = mTime;
        return true;
    }

    void ImpactSounds::log(std::string&& entry)
    {
        if (Settings::modernPhysics().mImpactSoundDebug)
            Log(Debug::Info) << "[impact] " << entry;
        mLog.push_back(std::move(entry));
        constexpr size_t maxLog = 64;
        while (mLog.size() > maxLog)
            mLog.pop_front();
    }
}
