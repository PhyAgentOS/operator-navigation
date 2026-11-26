[English](README.md) | [简体中文](README_zh.md)

# Map navigation

Owns the Nav2 backend, target execution, velocity authorization and arrival verification. Robot frames, footprint, speed limits and behavior trees remain caller configuration.

## Runtime

- Node `0.4.0`, Forge 2, Dora CLI/Python bindings 1.0.1, Forge Msgs/Common 2.1.0; exact build dependencies are in `uv.lock`.
- Ubuntu 24.04 x86_64, Python 3.12 ABI, external ROS 2 Jazzy and required algorithm/message overlays.
- Source the ROS/overlay setup before launch. `navigation --check-runtime` validates imports and generated wire types without opening hardware.
- The `directory_tar_gz` artifact contains the complete `navigation/` runtime; execute `navigation/navigation`. Do not move its executable away from its libraries.
- Forge 1 artifacts remain immutable. Upgrade the binary, Dora and message peers together; changing only the external CLI does not migrate a frozen Node.

## Behavior

The packaged, separately named SMAC planner uses the pinned Nav2 1.3.13 ABI and collision-consistent paos4 patch. It never replaces system ROS libraries. Optional map fusion keeps a separate navigation obstacle map; missing or occluded returns do not establish free space. The same Node includes the additive SegmentGoal/SegmentGoalAngle MPPI critics, scoped to its controller process; native regressions and discovery tests run during packaging. External coordinators own motion permission and task ordering.

Optional capabilities remain disabled when unconfigured. The Node owns only its own child processes; it does not modify vendor projects or control unrelated applications. Snapshots are observations, not a control API; consumers must verify freshness.

`use_sim_time: true` explicitly selects simulation time; the default remains host ROS time. Snapshot writes use an owned same-directory temporary file and atomic replacement.

## Development and packaging

```bash
uv sync --frozen --group dev --group build
uv run --frozen pytest -q
uv run --frozen ruff check .
source /opt/ros/jazzy/setup.bash
# Source required algorithm/message overlays for this deployment.
# Build the pinned planner and MPPI critics in backends/smac and backends/mppi first.
BUILD_PYTHON=.venv/bin/python bash scripts/build_pyinstaller.sh
```

Interfaces: [docs/interfaces.md](docs/interfaces.md). Configuration examples are in `config/`; deployment-specific paths and topics must be supplied by the caller. See `THIRD_PARTY_NOTICES.md` for sources and redistribution notices.

## Versioned releases

Contribute through a personal fork and a PR to `main`. After merging, create a new annotated `vX.Y.Z` tag matching `pyproject.toml` and dispatch **Build versioned release** with that tag. The workflow validates source identity, the Forge/Dora line, locked dependencies, tests and lint, then builds on Ubuntu 24.04 with ROS 2 Jazzy and checks the real frozen entrypoint. It attaches the runtime archive, SHA256 and source metadata to a Draft Release. Review and publish that draft before separately registering the Node in the resource platform.

Archive names use `<entrypoint>-v<version>-ubuntu24.04-x86_64-ros2-jazzy.tar.gz`. Existing releases/assets are never overwritten; changed bytes require a new version. ROS and algorithm backends remain external. Build-time Livox validation compiles only digest-checked official wire definitions and never starts a sensor driver. CI/import tests and simulation do not establish physical hardware acceptance.

Nav2 build compatibility and exact runtime ABI checks are described in [backends/ABI.md](backends/ABI.md).
