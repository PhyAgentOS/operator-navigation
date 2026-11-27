[English](README.md) | [简体中文](README_zh.md)

# Map navigation

## BREAKING CHANGE: Forge 2.x / Dora 1.x migration

Deploy matching Forge packages, Dora CLI/coordinator/daemon and all connected peer nodes together. Rebuild executable artifacts for that environment; replacing a single node in a Forge 1.x / Dora 0.4.x graph is unsupported.

The locked Python baseline is `forge-msgs==2.1.0`, `forge-common==2.1.0`, `Pillow==13.0.0` and `dora-rs==1.0.1`. Forge package families version independently; the `forge_msgs.v1` message namespace is unchanged.

Version `2.0.0` aligns the Node major with the Forge compatibility line. Earlier published `0.x` Forge 2 artifacts remain immutable; this version alignment does not change their runtime algorithms or message interfaces. The deployment break described here is from Forge 1 / Dora 0.4, not from those earlier Forge 2 artifacts. Retain the previous matching artifact and lock set for rollback. This repository does not currently provide a `stable/1.x` maintenance branch or a Forge 1 release workflow.

Source checks, binary builds and simulation checks are separate acceptance stages. Hardware, sustained-run and rollback acceptance must be established for the intended deployment; a version declaration does not establish them.

Owns the Nav2 backend, target execution, velocity authorization and arrival verification. Robot frames, footprint, speed limits and behavior trees remain caller configuration.

## Node, Endpoint and Tool API

| Item | Implementation |
| --- | --- |
| Production Nodes | 1: `navigation`; source `navigation.main:main`; packaged `navigation/navigation` |
| Auxiliary programs | `--version`, `--check-config` and `--check-runtime` are offline CLI modes, not additional Nodes |
| Tool support | Data-plane only; no ToolEndpoint provider or caller |
| Endpoints / operations | 0 / 0; Dora policy messages are not ToolEndpoint operations |
| Direct HTTP Tool API | 0; an external policy and Gateway own caller-facing Tools |
| Gateway Tools | No fixed Tool IDs in this repository; bindings belong to the consuming Skill |

Message inputs, outputs, units and lifecycle: [interface contract](docs/interfaces.md). Source: [entrypoint](src/navigation/main.py), [handler](src/navigation/node.py). Deployment examples: [dataflow](examples/dataflow.yaml), [configuration](config/). The runtime and release sections specify dependency locks, artifact layout and source provenance.

## Runtime

- Node `2.0.0`, Forge 2, Dora CLI/Python bindings 1.0.1, Forge Msgs/Common 2.1.0; exact build dependencies are in `uv.lock`.

Scan acquisition freshness is limited to 400 ms, with one nanosecond of timestamp
quantization tolerance. A delayed transform cannot renew a previous observation;
future stamps, expired wall leases and unhealthy localization still stop motion.
- Ubuntu 24.04 x86_64, Python 3.12 ABI, external ROS 2 Jazzy and required algorithm/message overlays.
- Source the ROS/overlay setup before launch. `navigation --check-runtime` validates imports and generated wire types without opening hardware.
- The `directory_tar_gz` artifact contains the complete `navigation/` runtime; execute `navigation/navigation`. Do not move its executable away from its libraries.
- Forge 1 artifacts remain immutable. Upgrade the binary, Dora and message peers together; changing only the external CLI does not migrate a frozen Node.

## Behavior

The packaged, separately named SMAC planner uses the pinned Nav2 1.3.13 source and collision-consistent paos4 patch, compiled for the exact runtime ABI recorded in its metadata. It never replaces system ROS libraries. Optional map fusion keeps a separate navigation obstacle map; missing or occluded returns do not establish free space. The same Node includes the additive SegmentGoal/SegmentGoalAngle MPPI critics, scoped to its controller process; native regressions and discovery tests run during packaging. External coordinators own motion permission and task ordering.

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

Contribute through a personal fork and a PR to `main`. After merging, create a new annotated `v2.X.Y` tag matching `pyproject.toml` and dispatch **Build versioned release** with `release_line=2.x` and that tag. This workflow accepts only stable `v2.X.Y` tags; other major versions and prerelease tags are rejected. The workflow validates source identity, the Forge/Dora line, locked dependencies, tests and lint, then builds on Ubuntu 24.04 with ROS 2 Jazzy and checks the real frozen entrypoint. It attaches the runtime archive, SHA256 and source metadata to a Draft Release. Review and publish that draft before separately registering the Node in the resource platform.

Archive names use `<entrypoint>-v<version>-ubuntu24.04-x86_64-ros2-jazzy.tar.gz`. Existing releases/assets are never overwritten; changed bytes require a new version. ROS and algorithm backends remain external. Build-time Livox validation compiles only digest-checked official wire definitions and never starts a sensor driver. CI/import tests and simulation do not establish physical hardware acceptance.

Nav2 build compatibility and exact runtime ABI checks are described in [backends/ABI.md](backends/ABI.md).

Optional path, costmap, fused-map and stop snapshots use one bounded background writer. Values and observation timestamps are captured before enqueueing; delayed or failed disk writes leave old files stale and do not block control handling. The control and velocity leases are rechecked immediately before output.
