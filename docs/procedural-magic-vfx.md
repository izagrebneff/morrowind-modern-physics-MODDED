# Procedural Magic VFX (experimental)

This fork replaces visible Morrowind spell particles in three main rendering paths:
- cast, hit, and persistent effects attached to actors;
- standalone world-space magic effects;
- moving magic projectiles, including mixed-effect bolts.

A view-facing quad uses a shared procedural GLSL program: coloured plasma,
rotating wisps, sparks and a pulsing impact ring. No animated DDS atlases are
required for the new visuals. Bolt colour is taken from the original magic
effects; cast and hit effects use a model/texture-name colour fallback.

Physics and gameplay are unchanged. Projectile collision uses the original
game collision mesh. Hidden NIF controllers still determine effect lifetime.

This is an experimental first renderer, not a claim that every special-case
effect is already covered. Scripted VFX, beams and some enchantment effects
may still require dedicated work.

## Testing

1. Build this branch and launch with a test save.
2. Test fire, frost, shock, poison, restoration and mixed-effect bolts.
3. Test first-person and third-person casting, NPC spells, area hits,
   continuous buffs/debuffs and underwater magic.
4. Compare performance with the previous EXE in dense spell battles.
5. If GLSL compilation fails or an effect is invisible, use the previous EXE.

The changes live on a separate branch to preserve physics and weapon fixes.
