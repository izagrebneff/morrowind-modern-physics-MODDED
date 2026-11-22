# Blood decals and dismemberment port — staged WIP

Target: `OPENMW-BUILD-WITH-ANIMATION-OPTIMIZATION` (OpenMW 0.52-derived). Source implementation: [incident201/brutal-morrowind](https://github.com/incident201/brutal-morrowind) (GPL-3.0 project; preserve upstream notices and attribution when distributing). The port MUST NOT overwrite local physics/ragdoll/animation optimization changes.

## Implemented in this branch (foundation only)

- Imported `SceneUtil::SurfaceDecals`, including double-buffered decal geometry and local barycentric anchor evaluation, and enabled compilation through `components/CMakeLists.txt`.
- Imported `SceneUtil::DecalProjector` header for placing wound decals on deformed or rigid mesh triangles.
- Added compatibility blood decal vertex/fragment shaders.

These are **unwired infrastructure**. They are not instantiated by actor rendering and currently do **not** render blood, perform dismemberment, or alter combat. A successful build is not yet proven.

## Required wiring and validation before enabling

1. Integrate per-actor decal instances into `RigGeometry` and `MorphGeometry` for skinned/morphing NPC meshes; do not mutate shared geometry. Implement equivalent safe support for rigid creature meshes.
2. Add actor-owned wound state, wound aging/limits, and projected hit attachment. Hook only confirmed ordinary hit events; avoid double damage/application. Preserve original creature blood effects until replacement validated.
3. Reconcile shader and postprocessing compatibility and shadow/depth state. Make two independent switches: surface blood and dismemberment.
4. Adapt `DecapitationGeometry`, `BisectionGeometry`, and `LimbSeveringGeometry` to existing physics and rigging systems. Limit dismemberment to verified supported skeletal layouts; fall back to normal deaths on unsupported creatures.
5. Package textures locally from the user-supplied Dark Messiah export (see `4/effects-data/textures/brutal_dm/`): `model_blood_slash.png`, `model_blood_small.png`, `model_blood_big.png`, `model_blood_tiny.png`, `cut_head.png`, `cut_body.png`. Do not commit proprietary third-party image assets without permission.
6. Compile Windows build, test regular NPCs, beast races, armored NPCs, corpses, creature rigs, decapitation, limb separation, save/load, cell transitions and shadows.

The base branch stays untouched and this PR must remain **draft** until the visual effects work in-game.
