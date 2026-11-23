# Brutal Morrowind physics integration: interface-first route

## Protected baseline
- Base branch: OPENMW-BUILD-WITH-ANIMATION-OPTIMIZATION (commit 5476249a498b46e788e9fadf9652da417941793f).
- Base Windows 2022 and Ubuntu jobs passed in Actions run 44536025168; the overall run did not pass because macOS jobs were cancelled.
- The earlier full-port branch and PR #20 are reference material, NOT merge-ready. Their wholesale file replacements broke public contracts.

## Rules
1. Preserve existing public MWBase::World and MWWorld::World contracts; put Brutal simulation behind adapters.
2. Do not add no-op C++ stubs or disable existing game features just to make CI pass.
3. Preserve animation compatibility cache, attackWindUp, camera, object carrying, input behavior and other fork-specific optimizations.
4. Each stage must pass the fast preflight, Windows and Ubuntu builds and targeted gameplay checks before dependent physics parts are replaced.
5. Do not merge before final compilation and gameplay testing.

## Port sequence
1. Validate baseline and static API checks.
2. Adapt collision shapes and dynamic objects without breaking caller contracts.
3. Adapt threaded scheduler and rigid-body updates.
4. Integrate NPC and creature ragdolls and render-bone synchronization.
5. Reconcile held objects, projectiles, input and existing optimizations.
6. Port blood and effects only after core physics is stable.

## Usage
Run: python3 CI/brutal_port_preflight.py

This heuristic guard checks missing relative includes and selected method references.
It is not a substitute for C++ compilation, link checks or in-game testing.
