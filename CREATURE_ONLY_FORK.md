# Creature-only death physics fork

Base: `openmw4-creatures-spelllight-only` (`8bfd543d41be438e5c5583e43fced17a503eafa0`).

## Changes
- World statics (STAT) keep fixed collision. Loose items are visual-only static
  collision objects; there is no dynamic item or equipment simulation.
- NPC records have vanilla animated death and no ragdoll.
- Dremora CREA records are excluded by case-insensitive record ID, model and
  display name; they use animated deaths and no ragdoll.
- All other CREA creature records retain ragdoll (including bipedal creatures).
- Death kicks for creatures are limited to 45–110 units/s, plus a small vertical
  component; spells and scripted kills with no attacker use a 55 units/s fallback.
- Retains articulated flying-creature skin-bone binding and controlled falling
  from the base physics fork. The original fix prevents wing and body stretching
  from skipping intermediate skinned bones.
- Retains moving magic-projectile light colored by each bolt's effect. Adds a
  safe fallback for empty effect sets.

## Smoke-test checklist
1. Verify all world statics and dropped items remain fixed.
2. Kill a normal NPC and a Dremora: regular death animations, no physics.
3. Kill a skeletal humanoid, guar, netch and cliff racer: ragdoll.
4. Verify flying creatures fall without stretched wings.
5. Verify small directional knockback on melee death and fallback on scripted death.
6. Cast fire, frost, shock and mixed projectiles in a dim interior: correct
   hue and light follows bolt.

Build is not validated until Windows Actions completes successfully.
