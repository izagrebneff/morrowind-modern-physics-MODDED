# Brutal Morrowind physics replacement — integration branch (NOT BUILD-READY)

Target base: `OPENMW-BUILD-WITH-ANIMATION-OPTIMIZATION`.
Source of physics: `incident201/brutal-morrowind` main (consult upstream license and preserve copyright notices).

## Ported in this branch
All 30 changed/new files in `apps/openmw/mwphysics/` were copied from the Brutal Morrowind tree, including actor collision, movement, simulation scheduler, object dynamics, ragdoll, creature rigging and dismemberment helper headers. Added `components/misc/brutalconstants.hpp`, and `creaturerig` to `apps/openmw/CMakeLists.txt`.

**These files supersede the local physics implementations IN THIS EXPERIMENTAL BRANCH ONLY.** The base branch is unchanged. This is not a working complete port yet. Do not merge into the user-facing branch or distribute a Windows build as finished until required dependencies and tests are resolved.

## Still required to make physics work
Brutal's physics code depends on extensive concurrent changes outside `mwphysics`: `MWMechanics::CreatureStats` and combat/character control, `Settings::game()` Brutal toggles and their defaults, `MWBase`/world/scene calls, skinned creature rigs and graphical dismemberment, Lua events and scripts, dynamic equipment handling, release/equipment serialization, `components/esm3` structures, extra renderer interfaces and additional generated resource files. These must be merged API-by-API instead of overwriting the user's entire fork, otherwise animation optimization and other local patches could be lost.

## Build/testing gates
1. Windows CI compile and link with all missing APIs reconciled.
2. Confirm native ragdoll from NPC and creature death, knockdowns, creature rigs, kicking and shield impacts.
3. Confirm dynamic objects, corpses and dropped equipment, save/load, reset state, cell transitions, interactions, swimming, slope sliding.
4. Confirm no double-physics update, no duplicate ragdoll and no regression in local animation compatibility cache.
5. Smoke-test visually and in game with Dark Messiah asset pack; test dismemberment separately.

Source replacement is implemented but the feature remains incomplete until these gates pass. Existing physics on the target base branch must remain live in the meantime.
