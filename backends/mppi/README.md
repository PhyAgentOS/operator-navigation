# Segment goal critics

Two isolated plugins for installed Nav2 MPPI 1.3.13. No installed Nav2 library,
DSH code, original critic, profile or running bundle is replaced or edited.

With enforced Reeds–Shepp inversion, MPPI truncates its transformed path at the
next cusp. This may become a single nearby point while the global goal remains
far away. Native PathFollow ignores a one-point path. Native GoalAngle scores
the path endpoint yaw but gates activation on distance to the global goal;
native Goal also targets the global goal. The current segment can consequently
lose the position and heading objectives needed to satisfy its inversion gate.

`SegmentGoalAngleCritic` inherits installed GoalAngleCritic, using current path
endpoint XY for activation and retaining the native path endpoint yaw score.
`SegmentGoalCritic` inherits installed GoalCritic, using the same endpoint XY
for both activation and position scoring. Each passes a local goal view to its
native parent's score method. They preserve native initialization, thresholds,
weights, powers, yaw wrapping and symmetry. Empty or malformed paths safely
preserve costs. Caller pose/path/goal and bookkeeping remain unchanged.

Replace `GoalCritic` and `GoalAngleCritic` in MPPI's critic list with
`SegmentGoalCritic` and `SegmentGoalAngleCritic`, respectively, and rename their
parameter blocks. Preserve the caller's configured weights, thresholds and powers. No inversion gate, collision cost, footprint,
speed limit or acceptance criterion is changed. The native loader resolves the
`mppi::critics::` names from the isolated prefix; source its `local_setup.bash`
after the ROS environment and before launching the controller.

Build from the Node repository root after sourcing ROS 2 Jazzy:

```sh
bash backends/mppi/build.sh backends/mppi/install
```

The build checks the Nav2 version and source hashes, runs native scoring and
plugin discovery tests, and records the actual build dependencies and library
hash in `backend.json`. `ROS_PREFIX` and `ROS_PYTHON` select a compatible
non-system ROS installation when needed. Packaging includes the installed
prefix; the default backend launch scopes it to the controller child.

Native C++ tests cover the recorded DSH failure at 0.059 m cusp distance with
0.263 rad yaw error, plus a single-point cusp 0.18 m away (outside the unchanged
0.12 m inversion XY gate) while its global goal is distant. The latter restores
the native position cost 0.9 for a stationary candidate and 0 at the endpoint.
Tests also cover final-goal parity against original native implementations,
powers/symmetry, inactive/empty/malformed paths, and independently loading both
custom and both original plugins. The plugin-loading executable deliberately
does not link the custom library. These are scoring and loading regressions;
closed-loop navigation must still be validated by the integration owner.

Upstream ABI and source hashes are recorded in `source-lock.json`; Nav2's Apache-2.0
license is retained in `LICENSE`.
