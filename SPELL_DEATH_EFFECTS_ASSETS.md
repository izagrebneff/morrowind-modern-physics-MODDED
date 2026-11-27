# Spell Death Effects: optional assets

Place these files in an active OpenMW data directory:

| Mesh | Optional texture override | Use |
|---|---|---|
| Meshes/SpellDeathEffects/fire.nif | Textures/SpellDeathEffects/fire.dds | Fire |
| Meshes/SpellDeathEffects/ice.nif | Textures/SpellDeathEffects/ice.dds | Persistent ice shard |
| Meshes/SpellDeathEffects/shock.nif | Textures/SpellDeathEffects/shock.dds | Electric arcs |
| Meshes/SpellDeathEffects/smoke.nif | Textures/SpellDeathEffects/smoke.dds | Black smoke |
| Meshes/SpellDeathEffects/steam.nif | Textures/SpellDeathEffects/steam.dds | Poison steam |

All files are optional. Missing fire, smoke and steam meshes use procedural sprites, which can independently use the corresponding DDS. Missing ice mesh uses procedural cones; missing shock mesh uses the original spell hit effect. Ice DDS requires an ice NIF with a replaceable root texture; shock DDS requires the external shock NIF. External NIFs may reference their own textures; the optional DDS overrides the first root texture rather than every material. Use small bone-local meshes centered around the origin; fire/smoke/steam are instanced at four bones, ice at up to 21 offsets. Author looping particle models without collision or embedded lights. Temporary NIF effects shrink toward the end of their timer; the engine's procedural sprites also fade their alpha. Restart the game after replacing cached resources. Ice state persists in saves made with this fork. No proprietary Morrowind assets are redistributed here.
