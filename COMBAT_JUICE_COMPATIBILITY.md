# Combat Juice 1.2.4 compatibility
Base: OPENMW-BUILD-WITH-MAGIC-PHYSICS at adc000d0e9ea529f91ed4ea7808501ece0d93276. No blood decal branches merged.
Combat Juice loose_gear.lua strips gear, calls item:teleport from corpse inventory, then sends LuaPhysics_ApplyImpulse. Native physics creates bodies asleep. The Lua container teleport does not use World::initObjectInCell, which normally wakes newly dropped items.
- Wake native bodies after an inventory-to-world teleport from a dead actor, after the original removal.
- For the precise global impulse event sent while the dead actor's old item count is zero and with no culprit, queue a native launch after teleport. If the transfer created no native dynamic body, dispatch the original LuaPhysics event instead. Other LuaPhysics calls are untouched.
- Launch speed scales inversely with item weight and is capped at 400 game units/s. Native gravity and collision handle the flight.
- Clear the displayed right-hand mesh when the inventory slot is empty, even if the drawn flag is still true. Shield handling already clears empty slots.
- Existing Combat Juice/LuaPhysics installation and mod options remain necessary for its gear shedding; no scripts from the mod were copied or changed. No guarantee for items already floating in an old save. Runtime validation with Combat Juice and LuaPhysics is needed.
