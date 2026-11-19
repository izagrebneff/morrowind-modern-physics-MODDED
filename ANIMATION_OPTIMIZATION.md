# Animation compatibility cache

Base: OPENMW-BUILD-WITH-IMPACT-PHYSICS (b4c597ed1af900efbd8c1c998cc81c8ae3748fd0).

- Cache compatibility by the complete, sorted, ASCII-case-insensitive node-name set of the actual instantiated skeleton, not by the NIF filename or a hash alone.
- For each KF source, retain only the matching controller iterators and normalized names. On cache hits, missing bones are never checked again. Valid controllers are still cloned, bound to each actor's own time source, and assigned the original blend mask. Text keys, animation priority and ordering remain unchanged.
- A weak OSG observer checks the identity/lifetime of the immutable KF holder. Reloaded resources are checked again. Root rebuilds discard the instance's layout entry. No pointers to actor-specific scene nodes are shared.
- Cache index bounded to 128 skeleton layouts; each layout holds at most 1024 source entries. Shared entries used by live animations remain valid after eviction. Mutex protects shared cache and warning deduplication.
- Each missing-bone warning is emitted at most once per model/source/bone per process; identical skeleton layouts may share a result without a second warning. Other warnings are unchanged.
- No animation file, script, physics parameter or gameplay behavior changed.

Validation: compile Windows and other platforms; test ordinary NPCs, beast races, first person, custom rower skeleton, transitions, equipment/rebuild and ragdoll. Compare repeated cell-entry frame times and warning counts against base. This is a loading/rebuild optimization, not a guarantee of higher steady-state FPS.
