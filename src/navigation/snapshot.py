"""Optional local diagnostic snapshots. No UI, credentials, or robot-specific logic."""

import json
import math
import sys
import tempfile
import threading
from pathlib import Path
from time import monotonic


def write_snapshot(folder, name, value):
    if not folder:
        return
    _write_payload(folder, name, _payload(value))


def _payload(value):
    return json.dumps({**value, "issued_monotonic": monotonic()}, allow_nan=False)


def _write_payload(folder, name, payload):
    path = Path(folder) / (name + ".json")
    path.parent.mkdir(parents=True, exist_ok=True)
    # Serialize before allocating a temporary file: invalid values leave the
    # previous snapshot untouched. Every call owns its temporary file, including
    # concurrent writers in one process or in different node processes.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        # Same-directory rename publishes one complete JSON document atomically.
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class LatestSnapshotWriter:
    """Publish optional observations without blocking the navigation event loop.

    At most one pending value per declared name is retained. Serialization and
    the receive timestamp happen in submit(); the worker performs only file IO.
    A delayed write therefore stays old, and never renews a consumer's lease.
    """

    def __init__(self, folder, names):
        self.folder = folder
        self.names = frozenset(names)
        self.pending = {}
        self.condition = threading.Condition()
        self.closing = False
        self.error = None
        self.thread = None
        if folder:
            self.thread = threading.Thread(
                target=self._run, name="navigation-snapshot-writer", daemon=True
            )
            self.thread.start()

    def submit(self, name, value):
        if name not in self.names:
            raise ValueError("Snapshot name was not declared")
        if not self.folder:
            return False
        payload = _payload(value)
        with self.condition:
            if self.closing or self.error is not None:
                return False
            self.pending[name] = payload
            self.condition.notify()
        return True

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending or self.closing)
                    if not self.pending:
                        return
                    name = next(iter(self.pending))
                    payload = self.pending.pop(name)
                _write_payload(self.folder, name, payload)
        except Exception as exc:
            with self.condition:
                self.error = type(exc).__name__
                self.pending.clear()
            print(f"Diagnostic snapshots stopped: {self.error} ({name})", file=sys.stderr)

    def close(self, timeout=2.0):
        with self.condition:
            self.closing = True
            self.condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=timeout)
            if self.thread.is_alive():
                print("Diagnostic snapshot writer shutdown timed out", file=sys.stderr)


def pose_value(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]


def grid_value(msg):
    m = msg.info
    return {
        "width": m.width,
        "height": m.height,
        "resolution": m.resolution,
        "origin": pose_value(m.origin),
        "frame_id": msg.header.frame_id,
        "data": list(msg.data),
    }
