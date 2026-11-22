# Standalone gore integration (work in progress)

Base: `feature/dm-animation-replacer-phase1-20261123`, preserving the pre-existing OpenMW 0.52 fork's physics, ragdoll, animation-cache changes, and the staged Dark Messiah animation loader.

## Current functional scope

This initial commit integrates **only renderer primitives** required for surface-attached blood:
- `SceneUtil::DecalProjector`: projects wound geometry onto triangles and records barycentric attachment coordinates.
- `SceneUtil::SurfaceDecals`: stores a bounded number of patches and updates them against the current surface during cull traversal, using double-buffered geometry.
- `components/CMakeLists.txt`: builds `surfacedecals.cpp`.

**Not yet active in gameplay:** no hit listener, no NPC/creature attachment, no blood texture/material wiring, no saves, and no dismemberment. The files are the real rendering foundation, not an enabled gore feature. Compilation and game testing are pending.

## Next integration steps (in order)

1. **Render attachment:** integrate `SurfaceDecals` into `SceneUtil::RigGeometry` and `SceneUtil::MorphGeometry`, preserving animated mesh updates and not modifying shared geometry (critical for multiple actors).
2. **Body wound surface capture:** adapt `npcblood.cpp` for NPC and *creature* animation classes, with bounded triangles/patches and per-actor lifecycle handling; preserve surface splatters on corpses, equipment exclusions, per-creature blood colors. Supply external blood textures as separate data directory.
3. **Hit event:** attach a wound only on real damage, using world-space hit and outward normal when available. On unsupported meshes fail gracefully. No existing combat rules or physics changes.
4. **Persistence:** serialize per-actor decals under a versioned blob; restore safely after model rebuild or save load, with size bounds, missing-model fallback and corpse cleanup.
5. **Dismemberment:** start with optional humanoid decapitation, then arm/leg cuts and torso. Port and adapt `decapitation.hpp`, `bisection.cpp/.hpp`, `limbsevering.cpp/.hpp` to the fork's *existing* ragdoll and scene graph instead of replacing ragdoll with Brutal's. Detached parts require render mesh separation, wound caps, Bullet bodies, physics lifetime, cleanup and save compatibility. Non-humanoid creatures need bone-specific rules and cannot be assumed to support human limb masks.
6. **Testing:** ordinary/beast NPCs, armored models, rigid creatures and skinned creatures, first-person, ragdoll, animated decapitation, repeated hits, equipment changes, cell reload and saved corpses. Build Windows and run smoke tests before enabling anything by default.

## Credits and license

Source of the three imported SceneUtil primitives: [incident201/brutal-morrowind](https://github.com/incident201/brutal-morrowind), GPLv3. Their source remains GPLv3; see upstream's LICENSE and this repository's LICENSE. Adapted as a separate, opt-in codebase rather than importing the Brutal Morrowind runtime. Do not redistribute proprietary Dark Messiah resource files in GitHub.
