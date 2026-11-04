# First-person sitting visibility — OpenMW 0.52

This fork builds on **feature/creatures-only-no-statics-dremora-fixed-flyers-spelllight**
(commit `56d6132998bf17124a393b3040c684df6315509f`). Creature ragdolls, static objects, NPC/Dremora exclusions,
controlled flying-creature physics, death knockback and colored spell projectile
lights remain unchanged.

## Visual-only player rendering API

The player's entire render scene graph is hidden while seated. This removes the
floating first-person hands, held weapon, shield, clothes and armor; it does **not**
apply game invisibility, manipulate inventory or alter collision.

Both calls are supported from player scripts:

```lua
local camera = require('openmw.camera')
camera.setPlayerModelVisible(false) -- after starting to sit
camera.setPlayerModelVisible(true)  -- after standing up
```

In Lua scripts that already hold a player object (including FirstPersonSeating
v0.5.0), the familiar method is supported as well:

```lua
player:setPlayerModelVisible(false)
player:setPlayerModelVisible(true)
```

Also available: `camera.getPlayerModelVisible()` and
`player:getPlayerModelVisible()`.

**Use the updated OpenMW executable together with the existing chair-sitting
Lua mod.** The executable supplies the drawing control, but does not by itself
detect which chair has been activated; the chair mod must call false on sitting
and true on standing. The renderer also restores visibility when game-specific
state is cleared, preventing a stuck invisible player on loading a save.

## Manual testing

1. Sit in first person, with ReAnimation v3 and pluginless Robert's Bodies:
   hands, weapons and armor must disappear, camera lowering remains smooth.
2. Stand up by reactivating the chair: model reappears, camera restores smoothly.
3. Verify third-person and standard first-person are unchanged when not seated.
4. Save, load, change equipment and test other mods.
