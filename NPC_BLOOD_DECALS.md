# NPC Blood Decals (OpenMW 0.52)
Standalone surface blood decals, based on the geometric approach in [Brutal Morrowind](https://github.com/incident201/brutal-morrowind/tree/16843789f84d588334c74417ee25a8cdf3d71e80). GPL code adapted only for surface projection / rendering, no imported proprietary resources. Base: adc000d0e9ea529f91ed4ea7808501ece0d93276.
- Native C++ on-hit callback, after standard armor/difficulty calculation; only successful melee/ranged health damage, NPCs other than player, including killing blows.
- Original blood particles and all gameplay/physics remain unchanged.
- Per-surface barycentric anchors, current-pose skinning, rigid and morph support. Max 24 stains/8192 decal vertices per NPC.
- NBLO optional save subrecord; existing saves without it load normally. Saves with NBLO should be used with this branch; compatibility with unmodified OpenMW not guaranteed.
- Enabled by default. In settings.cfg: [Game] npc blood decals = true.
- Four original, procedurally drawn RGBA PNG placeholder textures bundled via resources/vfs/textures/npc_blood. Replace with same names in a higher priority data directory, use neutral RGB128 and alpha as mask (DecalModulate). No Dark Messiah, no imported Lua, animations, sounds, particles, or sparks are required.
- Build and in-game verification required.
