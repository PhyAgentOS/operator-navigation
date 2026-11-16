# Collision-consistent Smac backend

`navigation_smac_planner::SmacPlannerHybrid` is a separately named Nav2 plugin,
built from the pinned Apache-2.0 Nav2 1.3.13 commit in `source-lock.json`.
The patch keeps continuous coordinates, cell indices, candidate costs and the
actual returned path consistent. It rechecks subcell poses, validates transition
samples and checks the final smoothed footprint. Invalid smoothing falls back
only to a previously validated raw path. Invalid raw paths fail explicitly.

Build on an x86-64 ROS2 Jazzy host with Nav2 1.3.13 development dependencies:

```sh
./build.sh /path/to/navigation2-1.3.13.tar.gz /new/runtime/prefix
```

The archive and patch hashes are verified before building. Copy the resulting
runtime prefix to `backends/smac/install` on the Node packaging host. The frozen
Node embeds this directory and scopes plugin lookup to its planner subprocess.
The system `/opt/ros` files are never changed. Roll back by selecting the prior
Navigation artifact and its matching Skill configuration.

Source, patches, build files and tests remain development resources; the Node
artifact contains the library, plugin index, lock receipt and upstream license.
The Skill contains no planner implementation. A Nav2 ABI change requires rebuild
and regression, not editing the lock to bypass the check.

`tests/probe.cpp` compares native collision checks, cache reuse, bounds, unknown
space, analytic cost snapshots and actual plugin output with/without smoothing.
It accepts an external JSON fixture (`grid`, `footprint`, `samples`) so private
maps need not be published with the Node. Run in a localhost-only isolated ROS
domain; it configures a frozen costmap without sensor or chassis processes.

Transition validation samples at a maximum half-cell bound on translation plus
footprint-radius times angular change. It is not exact continuous collision
detection or a 3-D coverage guarantee. Execution collision checks remain required.
The original discrete yaw bins and Reeds–Shepp model are retained; differential
in-place motion primitives are a separate change. Fast-free inflation shortcuts
are disabled until their conservatism for subcell poses and map edges is proved.
