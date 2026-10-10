#ifndef OPENMW_COMPONENTS_SETTINGS_CATEGORIES_MODERNPHYSICS_H
#define OPENMW_COMPONENTS_SETTINGS_CATEGORIES_MODERNPHYSICS_H

#include <components/settings/settingvalue.hpp>

namespace Settings
{
    /// Switches for the "Morrowind - Modern Physics" fork's features.
    struct ModernPhysicsCategory : WithIndex
    {
        using WithIndex::WithIndex;

        SettingValue<bool> mRecoverableStuckProjectiles{ mIndex, "Modern Physics",
            "recoverable stuck projectiles" };
    };
}

#endif
