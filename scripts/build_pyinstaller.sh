#!/usr/bin/env bash
set -euo pipefail
component_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$component_root"
python_bin="${BUILD_PYTHON:-python3}"
test -f backends/mppi/install/lib/libnavigation_sim_mppi_critics.so || {
  echo "Build the pinned MPPI plugins with backends/mppi/build.sh first" >&2; exit 1;
}
"$python_bin" -c 'import sys; assert sys.version_info[:2] == (3, 12), "Python 3.12 required"'
test -f backends/smac/install/lib/libnavigation_smac_planner.so || {
  echo "Build the pinned Smac plugin with backends/smac/build.sh first" >&2; exit 1;
}
# Freeze only the locked build environment; external ROS PYTHONPATH is for runtime checks.
env -u PYTHONPATH "$python_bin" -m PyInstaller --noconfirm --clean --onedir \
  --name navigation --paths src --collect-all forge_msgs --add-data "src/navigation/backend.launch.py:navigation" \
  --add-data "backends/smac/install:navigation/smac_overlay" \
  --add-data "backends/mppi/install:navigation/mppi_overlay" \
  --runtime-hook scripts/ros_runtime_hook.py \
  --exclude-module rclpy --exclude-module rcl_interfaces --exclude-module builtin_interfaces --exclude-module rosidl_parser --exclude-module rosidl_runtime_py --exclude-module ament_index_python --exclude-module action_msgs --exclude-module geometry_msgs --exclude-module lifecycle_msgs --exclude-module nav2_msgs --exclude-module nav_msgs --exclude-module sensor_msgs --exclude-module std_msgs --exclude-module std_srvs --exclude-module slam_toolbox --exclude-module livox_ros_driver2 --exclude-module tf2_ros --exclude-module tf2_py --exclude-module tf2_msgs --exclude-module unique_identifier_msgs --exclude-module rosgraph_msgs --exclude-module composition_interfaces --exclude-module launch --exclude-module launch_ros \
  scripts/pyinstaller_entry.py
./dist/navigation/navigation --version
if [[ "${SKIP_ROS_RUNTIME_CHECK:-0}" != 1 ]]; then
  ./dist/navigation/navigation --check-runtime
else
  echo "ROS runtime check explicitly skipped: hardware acceptance remains pending" >&2
fi
cp LICENSE THIRD_PARTY_NOTICES.md dist/navigation/
version=$("$python_bin" -c 'import tomllib; print(tomllib.load(open("pyproject.toml", "rb"))["project"]["version"])')
archive="dist/navigation-v${version}-ubuntu24.04-x86_64-ros2-jazzy.tar.gz"
[[ ! -e "$archive" ]] || { echo "refusing to overwrite $archive" >&2; exit 1; }
tar --sort=name --mtime='UTC 1970-01-01' --owner=0 --group=0 --numeric-owner -C dist -czf "$archive" navigation
(cd dist && sha256sum "$(basename "$archive")" > "$(basename "$archive").sha256")
