# Third-party dependencies

This repository's adapter code is licensed under Apache-2.0 (see LICENSE).
The frozen Node includes its Python runtime and Python dependencies. Their
installed license files and metadata are retained by PyInstaller hooks where
available; build environments must preserve dependency license information.

ROS2 Jazzy, its middleware and generated message bindings are external runtime
requirements, supplied by the host's ROS installation, not included in this
archive. Source the ROS setup and message overlay before invoking the Node.
FAST-LIO2, SLAM Toolbox, most Nav2 components and Livox SDK/driver are separately installed
backends, each governed by its upstream license. Except for the Smac and additive MPPI plugins described below, this archive does not contain
those backend sources or executables and does not grant redistribution rights
for proprietary robot SDKs.

Python: PSF license; NumPy: BSD-3-Clause; PyArrow: Apache-2.0;
PyYAML: MIT; Dora: Apache-2.0. Forge packages retain their distributed licenses.

## Modified Nav2 Smac plugin

The Node bundles `navigation_smac_planner`, derived from ROS Navigation2 1.3.13
(commit f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501), Apache-2.0.
Copyright notices in the upstream sources are retained. The plugin includes
changes to coordinate conversion, collision validity and candidate cost handling,
bounds checks, transition validation and post-smoothing validation. This is a
locally maintained derivative, not an upstream Nav2 release. The upstream license
and exact build/source receipt are included under `navigation/smac_overlay`.
Source and reproducible patches are in `backends/smac` of this Node repository.

Smac includes the upstream robin-hood-hashing implementation, MIT licensed,
Copyright (c) 2018-2021 Martin Ankerl; its complete license accompanies
the bundled plugin. OMPL, ROS and other system libraries remain dynamically
linked host dependencies under their respective licenses.

## Additive MPPI critics

`navigation_sim_mppi_critics` provides Apache-2.0 SegmentGoal/SegmentGoalAngle
plugins derived from the Nav2 1.3.13 critic interfaces. It leaves the installed
Nav2 libraries unchanged. The source and native regression tests are in
`backends/mppi`; the runtime library, license and build receipt are embedded in
`navigation/mppi_overlay`. Only the controller child receives this plugin path.
