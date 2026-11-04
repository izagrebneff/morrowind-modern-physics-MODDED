#ifndef GAME_MWWORLD_MATERIALS_H
#define GAME_MWWORLD_MATERIALS_H

// Fork ("Morrowind - Modern Physics"): what things are made of, judged by names (texture, model, record id) and, for
// items, by their type. Used for whether arrows stick, and for impact sounds.

#include <string_view>

namespace MWWorld
{
    class ConstPtr;

    enum class Material
    {
        Unknown,
        Wood, // also bark, roots, mushrooms (fungal trees)
        Stone, // stone, brick, plaster, bone, shell, chitin, ice
        Ceramic, // pottery, glass, crystal
        Metal,
        Earth, // dirt, sand, grass, plants
        Cloth, // fabric, leather, fur, paper, rope
        Flesh, // bodies
    };

    std::string_view getMaterialName(Material material);

    /// Arrows stick in it (wood, earth, cloth, flesh) rather than glance off.
    bool isSoftMaterial(Material material);

    /// A world surface, by the texture where it was hit and its model's name (either may be empty); fallback if
    /// neither says. (Its keywords are the ones arrows have always gone by.)
    Material classifySurface(std::string_view texture, std::string_view model, Material fallback);

    /// An item, by its type and, where that doesn't settle it, its record id and model.
    Material classifyItem(const ConstPtr& item);
}

#endif
