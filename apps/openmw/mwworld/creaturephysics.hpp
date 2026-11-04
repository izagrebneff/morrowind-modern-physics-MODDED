#ifndef OPENMW_MWWORLD_CREATUREPHYSICS_H
#define OPENMW_MWWORLD_CREATUREPHYSICS_H

// Keep ordinary NPCs and Dremora (including variant and modded records) on
// standard OpenMW death animations; enable corpse physics for other CREA actors.
#include <algorithm>
#include <cctype>
#include <string_view>

#include <components/esm3/loadcrea.hpp>

#include "class.hpp"
#include "ptr.hpp"

namespace MWWorld
{
    inline bool containsDremoraToken(std::string_view value)
    {
        constexpr std::string_view token = "dremora";
        return std::search(value.begin(), value.end(), token.begin(), token.end(),
                   [](char a, char b) {
                       return std::tolower(static_cast<unsigned char>(a))
                           == std::tolower(static_cast<unsigned char>(b));
                   })
            != value.end();
    }

    template <class PtrType>
    bool supportsCreatureCorpsePhysics(const PtrType& ptr)
    {
        if (ptr.isEmpty() || ptr.getType() != ESM::Creature::sRecordId)
            return false;
        // The record ID usually catches vanilla and TR / modded Dremora; the
        // model and display name also catch variants using unrelated IDs.
        return !containsDremoraToken(ptr.getCellRef().getRefId().getRefIdString())
            && !containsDremoraToken(ptr.getClass().getModel(ptr).value())
            && !containsDremoraToken(ptr.getClass().getName(ptr));
    }
}

#endif
