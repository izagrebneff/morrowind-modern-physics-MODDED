# NPC and creature blood decals, OpenMW 0.52
Based on previous Magic Physics / NPC Blood Decals branch (3518395f72a159710537f376d4c63d137ccdd680).
Native C++ surface projection from Brutal Morrowind; no Dark Messiah resources or scripts.
- NPCs except player; creatures with both CreatureAnimation and CreatureWeaponAnimation.
- Successful melee and ranged health-damage hits only; original particles unchanged. The standard combat interface preserves hit position if Diverse Blood clears it and reads its exposed texture override.
- Diverse Blood 2.4.2: exact normalized model lookup (257 models supplied), live user color overrides when exposed; red fallback. No energy stains. Ghosts, skeletons, bonewalkers, atronachs, frost elementals, Dwemer constructs, golems, fabricants, spriggans and similar models/IDs excluded before choosing any color.
- 12 distinct PNGs per color: red, dark, blue, green, orange, gold, white. 01–04 cuts, 05–08 small, 09–12 large. Selection is randomized within group. Textures in original ZIP at files/data/npc_blood_textures.zip; build installs them in resources/vfs/textures/npc_blood. Each PNG uses alpha as mask; neutral RGB128 has no modulation. Put replacements at textures/npc_blood/model_blood_COLOR_01.png etc. in an OpenMW data directory after other data directories. No ESP required. Restart OpenMW after replacing.
- BMD2 records keep color and variant; BMD1 old red records load and map to four equivalent new textures. NBLO optional subrecord for both actor types. Old saves compatible with this branch; compatibility in reverse not guaranteed.
- Default setting [Game] npc blood decals = true. Max 24 stains/8192 vertices per actor. New creature models with unknown names may need an added exclusion.
- Build and in-game validation required. 
