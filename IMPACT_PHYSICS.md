# Fatal impact and physical gear releases
Base: OPENMW-BUILD-WITH-COMBAT-JUICE-FIX (f1f0b9c4fc1a3943fc93d0fa619f8b1233db2945). No blood decal branches merged.

## Fatal impacts
- Physical damage: horizontal kick from the attacker's location, 140–280 game units/s input based on actual damage. A shared angular/linear field produces backward tipping and slight yaw instead of isolated torso-only impulses. This means a frontal attack pitches the humanoid onto its back; attacks from other sides fall away from the blow.
- Fatal spells: horizontal kick from 235 to 470 game units/s, previous +70 vertical component removed. All parts share the launch with a limited rotational differential.
- No initial positive vertical velocity for a hit-driven ragdoll, zero restitution, reduced angular damping. Ordinary contacts/gravity remain; no guarantee that collision with uneven ground never lifts a limb.
- Humanoid, skinned, rigid-piece and single-body fallbacks included. Non-hit scripted deaths, player death camera, living actor physics, freeze logic preserved. Single-body fallback for fatal physical hits if no usable rig. Restored poses keep existing settle mechanism.
- Exact flight distance is terrain- and friction-dependent: doubled initial horizontal speed is not a measured doubled distance.
## Gear
- Combat Juice 1.2.4 removes equipment using setEquipment in the actor-local script, then teleports in a global callback. The engine now samples both held mesh world matrices before either slot is removed, retaining the real position and rotation (including normal attachment offsets).
- On transfer from dead actor to same cell, near original position and within two simulation seconds, use the stored transform instead of the approximate random launch placement. Transient per-item data is not saved or cloned. For skinned shield geometry use its attachment bone; if no visible model/pose is available the old placement remains.
- Existing native corpse-drop impulse bridge preserved; held releases reduce horizontal scatter and clamp initial vertical velocity downwards. Other drops keep existing behavior. The Lua mod's chance settings, inventory, ownership, and events remain untouched. No Combat Juice or LuaPhysics script changes needed.
## Verification
Build and in-game tests required: sword and shield on death in both hands; two-handed weapon, magic kill, ranged kill, creatures with/without skeletal rigs, save/reload, and walls/slopes. Models with different ground and worn mesh pivots may not match exactly.
