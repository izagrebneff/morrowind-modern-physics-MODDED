# Dark Messiah animation replacer — Phase 1 (experimental)

This branch adds optional separate animation source loading for NPC third-person and player first-person animations. **It does not integrate Brutal Morrowind mechanics**, and does not change physics, ragdolls, or animation compatibility cache.

## Local asset installation

The exported Dark Messiah asset files are **not committed** to this public GitHub repository. Install the separately supplied Phase 1 ZIP, copying the contents of `DATA_ROOT` to a new OpenMW data directory, and add that directory last in `openmw.cfg`.

Under `[Game]` in `settings.cfg` enable `use additional anim sources = true`.

The directory layout must be:
- `Animations/dm_replacer/firstperson/*.kf`
- `Animations/dm_replacer/thirdperson/*.kf`

Loading occurs in `NpcAnimation::updateNpcBase` after the existing animation sources. OpenMW selects the last source containing matching text keys. This is an experimental source override: individual KFs may not match skeletons, some keyframes may have different expected events, and the game has not been compiled or tested for this patch. Existing animation loading remains fallback when no replacement keys exist.

## Next phase

Implement weapon-specific variant selection; validate first-person skeleton compatibility; map special Dark Messiah groups (kick, shield bash, clinch, rush, slide) to player actions and combat events without importing Brutal Morrowind's runtime. Preserve existing physics and animation optimization.

## How to disable

Remove the new data directory from `openmw.cfg`, or disable `use additional anim sources`. Without the exported files the added source loader does not discover any replacement files.
