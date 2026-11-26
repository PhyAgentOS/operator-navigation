#!/usr/bin/env bash
# Build and test the additive MPPI critics against an explicit ROS prefix.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
output="$(realpath -m "${1:?new output prefix required}")"
ros_prefix="${ROS_PREFIX:-/opt/ros/jazzy}"
[[ ! -e "$output" ]] || { echo 'Output exists; use a new prefix' >&2; exit 1; }
python3 - "$here" "$ros_prefix" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
source, prefix = map(Path, sys.argv[1:])
lock = json.loads((source / 'source-lock.json').read_text())
actual = ET.parse(prefix / 'share/nav2_mppi_controller/package.xml').getroot().findtext('version')
assert actual == lock['nav2_version'], f'Unvalidated Nav2 ABI: {actual}'
for name, digest in lock['source_sha256'].items():
    assert hashlib.sha256((source / name).read_bytes()).hexdigest() == digest, name
PY
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
set +u
if [[ -f "$ros_prefix/setup.bash" ]]; then source "$ros_prefix/setup.bash"; fi
set -u
cmake -S "$here" -B "$work" -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$output" -DPython3_EXECUTABLE="${ROS_PYTHON:-/usr/bin/python3}" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$work" -j"${BUILD_JOBS:-2}"
cmake --install "$work"
set +u
source "$output/local_setup.bash"
set -u
ctest --test-dir "$work" --output-on-failure
python3 - "$here" "$output" "$ros_prefix" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
source, output, prefix = map(Path, sys.argv[1:])
lock = json.loads((source / 'source-lock.json').read_text())
files = ['include/nav2_mppi_controller/critic_data.hpp',
         'include/nav2_mppi_controller/critics/goal_angle_critic.hpp',
         'include/nav2_mppi_controller/critics/goal_critic.hpp',
         'lib/libmppi_critics.so', 'lib/libmppi_controller.so']
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
lock['built_dependency_sha256'] = {name: digest(prefix / name) for name in files}
lock['library_sha256'] = digest(output / 'lib/libnavigation_sim_mppi_critics.so')
lock['native_regressions_passed'] = True
(output / 'backend.json').write_text(json.dumps(lock, indent=2) + '\n')
PY
