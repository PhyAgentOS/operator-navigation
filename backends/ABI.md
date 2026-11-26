# Nav2 build and runtime compatibility

The additive plugins can be built against Jazzy Nav2 1.3.13 or 1.3.14. Each
artifact records the actual build version in `backend.json`. The launcher
requires that exact version from the host runtime; this is not a claim that
binaries built for one version can be loaded by the other.

Source review of official navigation2 tags 1.3.13 and 1.3.14 found identical
public headers in nav2_core (12), nav2_costmap_2d (31) and nav2_smac_planner (15).
Of the 28 MPPI headers, only optimizer.hpp changed (an added fallback counter).
The custom critics inherit GoalCritic and GoalAngleCritic and use CriticData;
those interfaces and their upstream score implementations are unchanged.
MPPI's optimizer is not instantiated or subclassed by this plugin.

SMAC retains the checksum-pinned 1.3.13 source and paos4 patch. The 1.3.14 upstream
collision-checker change moves the radius early return before an inflation
warning; it does not alter the full polygon collision checks used here.
Neither this upgrade nor the build-version selection changes footprints,
clearance, arrival thresholds or command ownership.

A successful source review alone is not a build acceptance. Each release must
compile both plugins against its actual ROS installation, pass the two MPPI
native score/plugin-loading tests and pass the frozen Node runtime check.
The packaged metadata records the library and MPPI dependency checksums.
End-to-end Skill navigation remains a separate acceptance gate.
