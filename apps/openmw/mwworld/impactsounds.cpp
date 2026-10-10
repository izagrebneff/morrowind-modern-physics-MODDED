#include "impactsounds.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadsoun.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/rng.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/settings/values.hpp>
#include <components/vfs/manager.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/soundmanager.hpp"

#include "../mwworld/esmstore.hpp"

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

        std::vector<std::string> readStrings(const YAML::Node& node)
        {
            if (!node)
                return {};
            if (node.IsSequence())
                return node.as<std::vector<std::string>>();
            return { node.as<std::string>() };
        }

        void readRange(const YAML::Node& node, float (&range)[2])
        {
            if (!node)
                return;
            if (node.IsSequence() && node.size() == 2)
            {
                range[0] = node[0].as<float>();
                range[1] = node[1].as<float>();
            }
            else
                range[0] = range[1] = node.as<float>();
        }

        ImpactSounds::Choice readChoice(const YAML::Node& node)
        {
            ImpactSounds::Choice choice;
            choice.mSounds = readStrings(node["sound"]);
            choice.mFiles = readStrings(node["file"]);
            readRange(node["volume"], choice.mVolume);
            readRange(node["pitch"], choice.mPitch);
            return choice;
        }

        Material readMaterial(const std::string& name)
        {
            for (Material material : { Material::Wood, Material::Stone, Material::Ceramic, Material::Metal,
                     Material::Earth, Material::Cloth, Material::Flesh })
                if (getMaterialName(material) == name)
                    return material;
            if (name != "default")
                throw std::runtime_error("unknown material '" + name + "'");
            return Material::Unknown;
        }

        ImpactSounds::ByMaterial readByMaterial(const YAML::Node& node)
        {
            ImpactSounds::ByMaterial result;
            if (node)
                for (const auto& entry : node)
                    result[readMaterial(entry.first.as<std::string>())] = readChoice(entry.second);
            return result;
        }

        const ImpactSounds::Choice* find(const ImpactSounds::ByMaterial& choices, Material material)
        {
            auto found = choices.find(material);
            if (found == choices.end())
                found = choices.find(Material::Unknown);
            return found == choices.end() || found->second.empty() ? nullptr : &found->second;
        }

        // Not the game's: picking sounds shouldn't change the dice of the game itself.
        Misc::Rng::Generator& prng()
        {
            static Misc::Rng::Generator generator(Misc::Rng::generateDefaultSeed());
            return generator;
        }

        float between(const float (&range)[2])
        {
            return range[0]
                + (range[1] - range[0]) * Misc::Rng::rollProbability(prng());
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
                read(node, "full volume within", limits.mMinDistance);
                read(node, "heard up to", limits.mMaxDistance);
                read(node, "quietest", limits.mQuietest);
            }
            float projectileLoudSpeed = 2500.f;
            ByMaterial projectileStick;
            ByMaterial projectileGlance;
            if (const YAML::Node node = root["projectiles"])
            {
                read(node, "loud speed", projectileLoudSpeed);
                projectileStick = readByMaterial(node["stick"]);
                projectileGlance = readByMaterial(node["glance"]);
            }
            mLimits = limits;
            mProjectileLoudSpeed = projectileLoudSpeed;
            mProjectileStick = std::move(projectileStick);
            mProjectileGlance = std::move(projectileGlance);
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

    void ImpactSounds::projectileHit(
        const osg::Vec3f& position, Material surface, bool stuck, float speed, std::string_view what)
    {
        if (!Settings::modernPhysics().mImpactSounds || !Settings::modernPhysics().mProjectileImpactSounds)
            return;
        const Choice* choice = find(stuck ? mProjectileStick : mProjectileGlance, surface);
        const std::string result = choice != nullptr
            ? play(*choice, position, std::clamp(speed / mProjectileLoudSpeed, 0.f, 1.f), {})
            : std::string("no sound for it");
        log(std::format("projectile {} {} {} speed {:.0f} at {:.0f} {:.0f} {:.0f}: {}", what,
            stuck ? "sticks in" : "glances off", getMaterialName(surface), speed, position.x(), position.y(),
            position.z(), result));
    }

    std::string ImpactSounds::play(
        const Choice& choice, const osg::Vec3f& position, float strength, std::initializer_list<const void*> keys)
    {
        std::string why;
        if (!allow(keys, why))
            return why;
        const size_t count = choice.mSounds.size() + choice.mFiles.size();
        const size_t pick = std::min(count - 1,
            static_cast<size_t>(Misc::Rng::rollProbability(prng()) * count));
        // Harder is louder, but even a soft knock is clearly heard.
        const float volume = between(choice.mVolume) * (mLimits.mQuietest + (1 - mLimits.mQuietest) * strength);
        const float pitch = between(choice.mPitch);
        MWBase::SoundManager& sounds = *MWBase::Environment::get().getSoundManager();
        std::string name;
        MWSound::Sound* sound;
        if (pick < choice.mSounds.size())
        {
            name = choice.mSounds[pick];
            sound = sounds.playSound3D(position, ESM::RefId::stringRefId(name), volume, pitch, MWSound::Type::Sfx,
                MWSound::PlayMode::Normal, 0, mLimits.mMinDistance, mLimits.mMaxDistance);
        }
        else
        {
            name = choice.mFiles[pick - choice.mSounds.size()];
            const VFS::Path::Normalized file = Misc::ResourceHelpers::correctSoundPath(VFS::Path::Normalized(name));
            sound = sounds.playSound3D(position, VFS::Path::NormalizedView(file), volume, pitch, MWSound::Type::Sfx,
                MWSound::PlayMode::Normal, 0, mLimits.mMinDistance, mLimits.mMaxDistance);
        }
        // Out of earshot is fine; a name that doesn't exist is worth knowing about.
        const bool unknown = sound == nullptr
            && (pick < choice.mSounds.size()
                    ? MWBase::Environment::get().getESMStore()->get<ESM::Sound>().search(ESM::RefId::stringRefId(name))
                        == nullptr
                    : !MWBase::Environment::get().getResourceSystem()->getVFS()->exists(
                        Misc::ResourceHelpers::correctSoundPath(VFS::Path::Normalized(name))));
        return std::format("\"{}\" volume {:.2f} pitch {:.2f}{}", name, volume, pitch,
            unknown ? " (NO SUCH SOUND)" : (sound == nullptr ? " (not started)" : ""));
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
