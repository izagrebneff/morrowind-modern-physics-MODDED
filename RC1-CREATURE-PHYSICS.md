# OpenMW 0.52 RC1: creature ragdoll, spell projectile light, death knockback

Baseline: official OpenMW 0.52.0 RC1, revision `4ff261bf7c0d42a6cc06dd1ebc5d451802cff055`, confirmed embedded in the supplied Windows openmw.exe.

Target functionality:
- Ragdoll / dynamic body simulation only for creatures, including non-human creatures; **no NPC ragdoll**.
- No simulated loose items, equipment, arrows or other world objects.
- Spell projectiles illuminate nearby geometry, colored by the spell's effect(s).
- Creatures receive a directional initial death impulse; NPCs receive a brief visible backwards death movement without being turned into rigid bodies.

Source physics reference: sunspotdark/morrowind-modern-physics v1.1.
Previous working feature branch: openmw4-creatures-spelllight-only.

This branch intentionally starts from RC1 and merges changes from the physics fork using GitHub Actions. Work-in-progress until compiled and inspected.
