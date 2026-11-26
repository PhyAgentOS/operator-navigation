import json
import threading

import pytest

from navigation import snapshot


def test_blocked_disk_keeps_submission_nonblocking_and_coalesces_latest(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = snapshot._write_payload
    written = []

    def blocked(folder, name, payload):
        written.append(json.loads(payload))
        if len(written) == 1:
            entered.set()
            assert release.wait(5)
        original(folder, name, payload)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    clock = iter([100., 101., 102.])
    monkeypatch.setattr(snapshot, "monotonic", lambda: next(clock))
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("navigation_map", "path", "costmap", "navigation_stop"))
    try:
        assert writer.submit("navigation_map", {"sample": 1})
        assert entered.wait(2)
        # A single stalled write cannot stall these navigation callbacks. A stale
        # queued observation is replaced, rather than replayed after recovery.
        assert writer.submit("navigation_map", {"sample": 2})
        assert writer.submit("navigation_map", {"sample": 3})
        assert not (tmp_path / "navigation_map.json").exists()
    finally:
        release.set()
        writer.close()
    assert [x["sample"] for x in written] == [1, 3]
    assert json.loads((tmp_path / "navigation_map.json").read_text()) == {
        "sample": 3, "issued_monotonic": 102.0
    }
    assert not list(tmp_path.glob("*.tmp"))


def test_disk_failure_keeps_previous_observation_and_reports_failure(tmp_path, monkeypatch, capsys):
    snapshot.write_snapshot(tmp_path, "path", {"sample": "previous"})
    previous = (tmp_path / "path.json").read_bytes()

    def fail(*args):
        raise OSError("disk unavailable")

    monkeypatch.setattr(snapshot, "_write_payload", fail)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("path",))
    assert writer.submit("path", {"sample": "new"})
    writer.close()
    assert writer.error == "OSError"
    assert not writer.submit("path", {"sample": "later"})
    assert (tmp_path / "path.json").read_bytes() == previous
    assert "Diagnostic snapshots stopped: OSError" in capsys.readouterr().err


def test_all_names_advance_when_pending_updates_are_coalesced(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = snapshot._write_payload
    written = []

    def blocked(folder, name, payload):
        written.append((name, json.loads(payload)["sample"]))
        if len(written) == 1:
            entered.set()
            assert release.wait(5)
        original(folder, name, payload)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("navigation_map", "path", "costmap", "navigation_stop"))
    try:
        writer.submit("navigation_map", {"sample": 0})
        assert entered.wait(2)
        writer.submit("path", {"sample": 1})
        writer.submit("costmap", {"sample": 2})
        writer.submit("navigation_stop", {"sample": 3})
        for i in range(1, 100):
            writer.submit("navigation_map", {"sample": i})
    finally:
        release.set()
        writer.close()
    assert written == [("navigation_map", 0), ("path", 1), ("costmap", 2), ("navigation_stop", 3), ("navigation_map", 99)]


def test_close_is_bounded_with_a_stalled_filesystem(tmp_path, monkeypatch, capsys):
    entered, release = threading.Event(), threading.Event()

    def blocked(*args):
        entered.set()
        assert release.wait(5)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("path",))
    try:
        writer.submit("path", {})
        assert entered.wait(2)
        writer.close(timeout=.01)
        assert writer.thread.is_alive()
        assert "shutdown timed out" in capsys.readouterr().err
        assert not writer.submit("path", {})
    finally:
        release.set()
        writer.close()


def test_invalid_snapshot_preserves_previous_and_names_are_bounded(tmp_path):
    snapshot.write_snapshot(tmp_path, "path", {"valid": True})
    previous = (tmp_path / "path.json").read_bytes()
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("path",))
    try:
        with pytest.raises(ValueError):
            writer.submit("unknown", {})
        with pytest.raises(ValueError):
            writer.submit("path", {"range": float("nan")})
    finally:
        writer.close()
    assert (tmp_path / "path.json").read_bytes() == previous


def test_unconfigured_snapshots_start_no_worker():
    writer = snapshot.LatestSnapshotWriter(None, ("path",))
    assert writer.thread is None
    assert not writer.submit("path", {})
    writer.close()


def test_stalled_snapshot_does_not_delay_cancel_or_in_memory_reason(tmp_path, monkeypatch):
    from navigation import velocity

    entered, release = threading.Event(), threading.Event()
    original = snapshot._write_payload

    def blocked(*args):
        entered.set()
        assert release.wait(5)
        original(*args)

    monkeypatch.setattr(snapshot, "_write_payload", blocked)
    writer = snapshot.LatestSnapshotWriter(tmp_path, ("navigation_map", "navigation_stop"))
    gate = velocity.VelocityGate()
    gate.set_enabled(True)
    gate.receive(.05, 0, 0, now=10.)
    monkeypatch.setattr(velocity.time, "monotonic_ns", lambda: 10_400_000_000)
    reasons = []

    def cancel(reason):
        assert not gate.enabled
        reasons.append(reason)
        writer.submit("navigation_stop", {"reason": reason})

    try:
        writer.submit("navigation_map", {"data": [0]})
        assert entered.wait(2)
        _, command = velocity.sample_authorized_output(
            gate, active=True, execution_ready=True,
            permission_failure=lambda now: "expired", cancel=cancel,
        )
        assert command.vx == 0
        assert reasons == ["expired，已停车"]
        assert not (tmp_path / "navigation_stop.json").exists()
    finally:
        release.set()
        writer.close()
    assert json.loads((tmp_path / "navigation_stop.json").read_text())["reason"] == reasons[0]
