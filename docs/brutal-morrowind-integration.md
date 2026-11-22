# Brutal Morrowind physics integration (work in progress)

Base: `OPENMW-BUILD-WITH-ACTIVE-RAGDOLL-80` (OpenMW 0.52 based).
Upstream: https://github.com/incident201/brutal-morrowind (OpenMW 0.51 based).

## Requested behavior

- Dark Messiah animations and assets remain supported.
- Brutal Morrowind should own death-time ragdoll simulation for NPCs and compatible creatures.
- The fork's existing death ragdoll must not concurrently simulate the same actor.
- Preserve the fork's live-actor reactions, spell lighting, and existing static-object restrictions.
- Fall back to the fork's ragdoll when the Brutal implementation cannot create a compatible creature rig.

## Porting requirements

This is a source-level port, **not** a plugin load-order change. The two projects have
incompatible ragdoll implementations and both own Bullet body creation and constraints.
Before selecting an owner, port the relevant Brutal implementation and its dependencies:
`mwphysics/ragdollcontrol.hpp`, `ragdollstep.hpp`, `ragdollmotion.hpp`,
`ragdollkick.hpp`, `ragdollenhancement.hpp`, `creaturerig.*` plus
the related `PhysicsTaskScheduler`, physics-system, and animation bridge changes.

The current fork already implements multiple ragdoll creation paths in
`mwphysics/physicssystem.cpp` (`createRagdoll`, `createSkinnedRagdoll`,
`createRigidPieceRagdoll`, fallback rigid body). The integration must select
exactly one path *before* adding Bullet bodies, and must ensure proper cleanup on
actor deletion, reanimation, and cell transitions.

## Build and acceptance

This commit is an integration scaffold only; it **does not yet enable Brutal
Morrowind death physics**. The push CI is a baseline Windows/Linux/macOS build.
Do not advertise an artifact from this commit as the finished integration.

Acceptance tests: NPC deaths, creature deaths, death while airborne,
back-to-back impulses, death/resurrection, cell unload and save/load, Dark
Messiah kick/finisher, dragging bodies, no duplicate Bullet collision bodies.
