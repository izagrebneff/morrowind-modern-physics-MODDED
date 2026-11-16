# NPC surface blood (OpenMW 0.52 RC1 experimental port)

This branch is based on commit `4453cba1bab2ea7a887938e5347bee389f099452` of
`izagrebneff/morrowind-modern-physics-MODDED` and keeps the pre-existing creature
physics, flying-corpse and spell-light modifications.

The implementation isolates the skin-conforming NPC blood-decal technique from
`incident201/brutal-morrowind` v0.2.0 (GPL-3.0), including the barycentric
mesh projection and cull-time pose tracking. No Dark Messiah of Might and Magic
resources are used: blood textures are generated in C++ on demand.

### Scope
- NPCs only (not creatures, not the player).
- Only successful health-damaging physical hits with a world-space hit position.
- Marks stick to the impacted body/armor mesh and follow deformations.
- Maximum of 24 marks per character; existing game damage, magic, controls,
  physics, ragdoll behavior, particles, and animations are unchanged.
- No dismemberment, gore spray, sparks, kick, audio imports, Dark Messiah
  animations, nor launcher changes.
- Stain records have native serialization helpers, but **savegame persistence
  is not wired up** in this initial integration.

### Files
- `apps/openmw/mwrender/npcblood.cpp`: selective native decal implementation.
- `components/sceneutil/{decalprojector.hpp,surfacedecals.*}`:
  mesh projection and the double-buffered animated decals.
- `components/sceneutil/{riggeometry.*,morphgeometry.*}`:
  instance-only attachments without changes to the source geometry.
- `apps/openmw/mwlua/types/actor.cpp` and
  `files/data-mw/scripts/omw/combat/local.lua`: normal OpenMW hit callback
  integration, without any second damage event.
- `files/shaders/compatibility/blooddecal.*`: renderer blending/fog shader.

### Build and installation
Build the branch as an ordinary OpenMW source tree using the existing repository
Windows build workflow. Install the resulting **whole build** (executable,
matching Lua scripts and the new shader files); replacing only openmw.exe
while keeping old data files will not enable the effect.

**Status:** experimental source port; compile and in-game verification are
required before marking this ready for regular gameplay.

### Upstream and license
Derived from [Brutal Morrowind v0.2.0](https://github.com/incident201/brutal-morrowind/releases/tag/v0.2.0),
built on [OpenMW](https://github.com/OpenMW/openmw). Preserve their respective
licenses and attribution when distributing a compiled build.
