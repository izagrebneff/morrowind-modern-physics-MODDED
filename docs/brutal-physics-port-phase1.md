# Brutal Morrowind physics port — Phase 1: joint stability

Source: incident201/brutal-morrowind, `apps/openmw/mwphysics/ragdoll.cpp` (GPL-3.0, see upstream LICENSE).
Target: OPENMW-BUILD-WITH-ANIMATION-OPTIMIZATION.

## Implemented

* For non-flying ragdolls, use Brutal's per-joint Bullet solver iteration cap (120), ERP (0.9) and CFM (0) to hold anchors together through hard impacts.
* For non-flying ragdolls use Brutal's restitution (0.05) and angular damping (0.7).
* Preserve the fork's existing 2.75x part mass multiplier, buoyancy, self-collision policy, custom creature rigid-piece physics, physics scheduler, dynamic-object code and animation optimization.
* Flying corpses keep the fork's 28-iteration special case and original restitution/damping.

## Limitations

This is a first, **uncompiled / untested** integration of Brutal's passive joint stability only, not its complete ragdoll, creature rig, dropped-equipment system, or interactive combat physics. Heavy per-joint solver iterations can increase CPU load; measure stress scenarios with many corpses before merging. The new parameters are applied unconditionally to non-flying ragdolls and must be checked for joint stability and animation pose transitions.

## Planned next passes

1. Compare and adapt explicit Bullet step synchronization and optional joint soft stops, after isolating existing fork changes.
2. Add procedural creature rig support via the exported creature_rigs.txt if not already handled by current fork.
3. Integrate dynamic dropped equipment safely with inventory persistence and lootability.
4. Verify compiler support (Windows), runtime behavior and save compatibility; then review performance.

## Test scenarios

- Human NPC dies on flat ground, stairs and slopes; limbs should stay connected.
- Impact against wall and another ragdoll; look for joint stretching and tunneling.
- Several dead NPCs at once; compare CPU frametime against original branch.
- Flying creature death, underwater death, dragging and freezing, save/reload.
- Pick up and throw an ordinary object; ensure existing physics unaffected.

No Brutal Morrowind runtime module or its Dark Messiah assets are required for this phase.
