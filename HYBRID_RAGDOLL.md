# Hybrid ragdoll / Hit Reactions Animated 2.0.7

Based on animation optimization commit 5476249a498b46e788e9fadf9652da417941793f.
Install the separate HRA compatibility patch, which adds hybridDeath = fall and not IS_DREMORA to the options passed to animation.playBlended in brutalStaggers.lua. Older engines ignore the extra field.

For a newly dying non-player biped with Lua animations enabled, CharacterController dispatches its ordinary death group before activating native physics. HRA's synchronous play handler selects an Endorphin clip and marks it. The engine accepts only an actual active full-body, non-looping, held clip. If no clip is marked (including chance declines), normal immediate ragdoll behavior is retained. Dremora models with Lua animations use animation-only death to preserve their dissolve controllers. Other creatures keep the existing path.

The selected clip is allowed to finish; the terminal pose remains held for at least one update before native articulated ragdoll creation. The original death kick is consumed without reapplying it, preserving the existing native bounded actor movement velocity. The 15-second guard only recovers from a frozen/interfered-with clip. Existing physics freeze/drag, collision, save pose, and resurrection code is used. If articulated creation fails, leave the animation pose. In-progress Lua death animations are not newly serialized; unloading or saving before handoff can restart death on reload, as in the base engine. Finished ragdoll poses use existing persistence.

Test in game: melee, magic, HRA chance 0/100, varying clip lengths, Dremora dissolve, quadrupeds/flyers, equipment drop, resurrection and save/reload during and after handoff. Build success does not establish visual correctness.
