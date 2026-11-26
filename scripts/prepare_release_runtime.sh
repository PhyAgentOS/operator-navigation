#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p .release-runtime
python3 - <<'PY'
import json, pathlib, urllib.request
lock = json.loads(pathlib.Path('backends/smac/source-lock.json').read_text())
with urllib.request.urlopen(lock['archive_url'], timeout=120) as stream:
    pathlib.Path('.release-runtime/nav2.tar.gz').write_bytes(stream.read())
PY
bash backends/smac/build.sh .release-runtime/nav2.tar.gz backends/smac/install
bash backends/mppi/build.sh backends/mppi/install
