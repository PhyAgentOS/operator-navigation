#!/usr/bin/env bash
# Build a uniquely named plugin against the host's Nav2 1.3.13 ABI.
# Usage: build.sh navigation2-1.3.13.tar.gz OUTPUT_RUNTIME_PREFIX
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
archive="$(realpath "${1:?upstream archive required}")"
output="$(realpath -m "${2:?output prefix required}")"
[[ ! -e "$output" ]] || { echo 'Output already exists; use a new versioned prefix' >&2; exit 1; }
python3 - "$here" "$archive" <<'PY'
import hashlib,json,pathlib,sys
here=pathlib.Path(sys.argv[1]); lock=json.loads((here/'source-lock.json').read_text())
for path,key in [(pathlib.Path(sys.argv[2]),'archive_sha256'),(here/'collision-consistency.patch','patch_sha256')]:
    assert hashlib.sha256(path.read_bytes()).hexdigest()==lock[key], f'Integrity mismatch: {path}'
assert pathlib.Path('/opt/ros/jazzy/share/nav2_core/package.xml').is_file(), 'ROS2 Jazzy required'
import xml.etree.ElementTree as ET
for package in ('nav2_core','nav2_costmap_2d','nav2_smac_planner'):
    version=ET.parse(f'/opt/ros/jazzy/share/{package}/package.xml').getroot().findtext('version')
    assert version in lock['build_nav2_versions'], f'Unvalidated {package} ABI: {version}'
PY
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
tar -xzf "$archive" -C "$work" navigation2-1.3.13/nav2_smac_planner navigation2-1.3.13/LICENSE
source_dir="$work/navigation2-1.3.13/nav2_smac_planner"
patch --batch -d "$source_dir" -p1 < "$here/collision-consistency.patch"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cmake -S "$source_dir" -B "$work/build" -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX="$work/install"
cmake --build "$work/build" -j"${BUILD_JOBS:-2}"
cmake --install "$work/build"
mkdir -p "$output/lib" "$output/share/ament_index/resource_index" "$output/share/navigation_smac_planner"
cp "$work/install/lib/libnavigation_smac_planner.so" "$output/lib/"
cp -r "$work/install/share/ament_index/resource_index/"* "$output/share/ament_index/resource_index/"
cp "$work/install/share/navigation_smac_planner/"{package.xml,smac_plugin_hybrid.xml} "$output/share/navigation_smac_planner/"
cp "$work/navigation2-1.3.13/LICENSE" "$output/share/navigation_smac_planner/LICENSE"
sed -n '1,32p' "$source_dir/include/nav2_smac_planner/thirdparty/robin_hood.h" > "$output/share/navigation_smac_planner/ROBIN_HOOD_LICENSE"
cp "$here/source-lock.json" "$output/backend.json"
python3 - "$output" <<'PY'
import hashlib,json,pathlib,sys
p=pathlib.Path(sys.argv[1]); f=p/'backend.json'; data=json.loads(f.read_text())
import xml.etree.ElementTree as ET
data['nav2_version']=ET.parse('/opt/ros/jazzy/share/nav2_core/package.xml').getroot().findtext('version')
data['source_nav2_version']='1.3.13'
data['library_sha256']=hashlib.sha256((p/'lib/libnavigation_smac_planner.so').read_bytes()).hexdigest()
f.write_text(json.dumps(data,indent=2)+'\n')
PY
