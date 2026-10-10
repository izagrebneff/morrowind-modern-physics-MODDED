#include "materials.hpp"

#include <optional>
#include <span>
#include <string>
#include <utility>

#include <components/esm3/loadalch.hpp>
#include <components/esm3/loadappa.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadbook.hpp>
#include <components/esm3/loadclot.hpp>
#include <components/esm3/loadingr.hpp>
#include <components/esm3/loadligh.hpp>
#include <components/esm3/loadlock.hpp>
#include <components/esm3/loadmisc.hpp>
#include <components/esm3/loadprob.hpp>
#include <components/esm3/loadrepa.hpp>
#include <components/esm3/loadskil.hpp>
#include <components/esm3/loadweap.hpp>
#include <components/misc/strings/lower.hpp>

#include "class.hpp"
#include "ptr.hpp"

namespace MWWorld
{
    namespace
    {
        using Keywords = std::pair<Material, std::span<const std::string_view>>;

        // World surfaces: the words arrows have always gone by (hard ones first: they win when a name has both).
        constexpr std::string_view surfaceMetal[]
            = { "metal", "iron", "steel", "bronze", "silver", "gold", "dwrv", "dwem" };
        constexpr std::string_view surfaceCeramic[] = { "glass", "crystal" };
        constexpr std::string_view surfaceStone[] = { "stone", "rock", "brick", "cobble", "marble", "granite", "slate",
            "tile", "plaster", "stucco", "chitin", "bone", "shell", "pave", "ice", "lava", "velothi", "daed", "ruin" };
        constexpr std::string_view surfaceWood[] = { "wood", "wd_", "bark", "tree", "log", "plank", "board", "timber",
            "branch", "root", "crate", "barrel", "basket", "wicker", "mushroom", "shroom", "fung" };
        constexpr std::string_view surfaceCloth[] = { "straw", "thatch", "hay", "fabric", "cloth", "banner", "rug",
            "carpet", "tapestry", "rope", "leather", "hide", "fur" };
        constexpr std::string_view surfaceEarth[]
            = { "flora", "moss", "dirt", "mud", "sand", "grass", "soil", "ground", "kelp" };
        constexpr Keywords surfaceKeywords[] = { { Material::Metal, surfaceMetal }, { Material::Ceramic, surfaceCeramic },
            { Material::Stone, surfaceStone }, { Material::Wood, surfaceWood }, { Material::Cloth, surfaceCloth },
            { Material::Earth, surfaceEarth } };

        // Items: more words, for the clutter Morrowind is full of (redware, silverware, bottles...).
        constexpr std::string_view itemMetal[] = { "metal", "iron", "steel", "bronze", "silver", "gold", "dwrv", "dwem",
            "brass", "copper", "tin_", "pewter", "plate", "lantern", "candlestick", "key", "coin", "nail", "chain",
            "daedric", "ebony" };
        constexpr std::string_view itemCeramic[] = { "glass", "crystal", "redware", "bottle", "flask", "vial",
            "ceramic", "clay", "pottery", "porcelain", "urn", "vase", "jug", "pitcher", "ashpot", "ash pot", "pot_",
            "_pot", "potion", "soulgem", "soul_gem", "gem", "pearl", "ruby", "emerald", "diamond", "sapphire" };
        constexpr std::string_view itemStone[] = { "stone", "rock", "skull", "bone", "chitin", "shell", "marble" };
        constexpr std::string_view itemWood[] = { "wood", "wd_", "bark", "log", "plank", "board", "crate", "barrel",
            "basket", "bucket", "wicker", "bow", "staff", "club", "broom", "lute", "drum", "mushroom", "shroom" };
        constexpr std::string_view itemCloth[] = { "cloth", "fabric", "rag", "rug", "pillow", "sack", "bag", "fur",
            "pelt", "hide", "leather", "rope", "paper", "book", "scroll", "note", "letter", "netch", "straw" };
        constexpr std::string_view itemEarth[]
            = { "ingred", "plant", "flower", "herb", "moss", "meat", "bread", "food", "fruit", "egg", "ash_salt", "candle" };
        // Ceramic first: redware plates and cups aren't metal.
        constexpr Keywords itemKeywords[] = { { Material::Ceramic, itemCeramic }, { Material::Metal, itemMetal },
            { Material::Stone, itemStone }, { Material::Wood, itemWood }, { Material::Cloth, itemCloth },
            { Material::Earth, itemEarth } };

        Material byName(std::string_view name, std::span<const Keywords> keywords)
        {
            if (name.empty())
                return Material::Unknown;
            const std::string lower = Misc::StringUtils::lowerCase(name);
            for (const auto& [material, words] : keywords)
                for (const std::string_view word : words)
                    if (lower.find(word) != std::string::npos)
                        return material;
            return Material::Unknown;
        }
    }

    std::string_view getMaterialName(Material material)
    {
        switch (material)
        {
            case Material::Wood:
                return "wood";
            case Material::Stone:
                return "stone";
            case Material::Ceramic:
                return "ceramic";
            case Material::Metal:
                return "metal";
            case Material::Earth:
                return "earth";
            case Material::Cloth:
                return "cloth";
            case Material::Flesh:
                return "flesh";
            case Material::Unknown:
                break;
        }
        return "unknown";
    }

    bool isSoftMaterial(Material material)
    {
        return material == Material::Wood || material == Material::Earth || material == Material::Cloth
            || material == Material::Flesh;
    }

    Material classifySurface(std::string_view texture, std::string_view model, Material fallback)
    {
        if (const Material fromTexture = byName(texture, surfaceKeywords); fromTexture != Material::Unknown)
            return fromTexture;
        if (const Material fromModel = byName(model, surfaceKeywords); fromModel != Material::Unknown)
            return fromModel;
        return fallback;
    }

    Material classifyItem(const ConstPtr& item)
    {
        const unsigned int type = item.getType();
        // What the type says for sure.
        if (type == ESM::Potion::sRecordId)
            return Material::Ceramic;
        if (type == ESM::Book::sRecordId || type == ESM::Clothing::sRecordId)
            return Material::Cloth;
        if (type == ESM::Lockpick::sRecordId || type == ESM::Probe::sRecordId || type == ESM::Repair::sRecordId)
            return Material::Metal;
        if (type == ESM::Weapon::sRecordId)
        {
            // Bows and shafts are wood, whatever their tips or fittings are named after.
            const ESM::RefId& weaponType = item.get<ESM::Weapon>()->mBase->mData.mType;
            if (weaponType == ESM::WeaponType::MarksmanBow || weaponType == ESM::WeaponType::Arrow
                || weaponType == ESM::WeaponType::Bolt)
                return Material::Wood;
        }
        if (type == ESM::Ingredient::sRecordId)
        {
            // Mostly plants and meat; some are minerals or bits of shell (glass, ebony, scales...).
            const Material named = byName(item.getCellRef().getRefId().getRefIdString(), itemKeywords);
            return named == Material::Unknown || named == Material::Wood || named == Material::Cloth ? Material::Earth
                                                                                                    : named;
        }

        // Otherwise the name, then what the type suggests.
        const Material named = byName(item.getCellRef().getRefId().getRefIdString(), itemKeywords);
        if (named != Material::Unknown)
            return named;
        const Material fromModel = byName(item.getClass().getCorrectedModel(item), itemKeywords);
        if (fromModel != Material::Unknown)
            return fromModel;

        if (type == ESM::Weapon::sRecordId)
            return Material::Metal;
        if (type == ESM::Armor::sRecordId)
        {
            const ESM::RefId skill = item.getClass().getEquipmentSkill(item, false);
            return skill == ESM::Skill::LightArmor ? Material::Cloth : Material::Metal;
        }
        if (type == ESM::Light::sRecordId)
            return Material::Metal;
        if (type == ESM::Apparatus::sRecordId)
            return Material::Ceramic;
        return Material::Unknown;
    }
}
