"""A delayed TF must not renew or prematurely revoke a valid scan lease."""

import math
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navigation.fusion import NavigationMapFusion


@pytest.fixture
def delayed_tf(monkeypatch):
    class TransformException(Exception):
        pass

    monkeypatch.setitem(
        sys.modules, "tf2_ros", SimpleNamespace(TransformException=TransformException)
    )
    monkeypatch.setitem(
        sys.modules, "rclpy.time", SimpleNamespace(Time=SimpleNamespace(from_msg=lambda m: m))
    )
    fusion = NavigationMapFusion.__new__(NavigationMapFusion)
    fusion.map = SimpleNamespace(header=SimpleNamespace(frame_id="map"))
    fusion.scan = SimpleNamespace(
        header=SimpleNamespace(frame_id="base_link", stamp=SimpleNamespace(sec=1, nanosec=0))
    )
    fusion.ros = Mock()
    fusion.ros.get_clock.return_value.now.return_value.nanoseconds = 1_100_000_000
    fusion.buffer = Mock()
    fusion.buffer.lookup_transform.side_effect = TransformException("TF not arrived yet")
    fusion.config = {"odom_frame": "odom"}
    fusion.evidence = Mock()
    fusion.processed_stamp = -1
    fusion.last_processed = -math.inf
    fusion.last_publish = 10.0
    fusion.accepted_at = 9.8
    fusion.accepted_stamp_ns = 1_000_000_000
    fusion.ready = True
    return fusion


def tick(fusion, now, healthy=True):
    fusion.last_publish = now  # exercise lease logic independently of ROS publication
    fusion.tick({"healthy": healthy, "issued_monotonic": now}, now)


def test_delayed_transform_keeps_only_previous_fresh_observation(delayed_tf):
    tick(delayed_tf, 10.0)
    assert delayed_tf.ready
    assert delayed_tf.accepted_at == 9.8
    assert delayed_tf.processed_stamp == -1  # retry the unprocessed scan
    delayed_tf.evidence.observe.assert_not_called()
    delayed_tf.evidence.invalidate_free.assert_not_called()
    tick(delayed_tf, 10.19)
    assert delayed_tf.ready
    tick(delayed_tf, 10.21)
    assert not delayed_tf.ready
    delayed_tf.evidence.invalidate_free.assert_called_once()


def test_first_scan_without_transform_never_becomes_ready(delayed_tf):
    delayed_tf.accepted_at = -math.inf
    delayed_tf.accepted_stamp_ns = None
    tick(delayed_tf, 10.0)
    assert not delayed_tf.ready


def test_unhealthy_localization_immediately_revokes_previous_observation(delayed_tf):
    tick(delayed_tf, 10.0, healthy=False)
    assert not delayed_tf.ready
    delayed_tf.evidence.invalidate_free.assert_called_once()


def test_stale_scan_immediately_revokes_previous_observation(delayed_tf):
    delayed_tf.ros.get_clock.return_value.now.return_value.nanoseconds = 1_500_000_000
    tick(delayed_tf, 10.0)
    assert not delayed_tf.ready


def test_future_new_scan_retains_only_the_previous_unexpired_observation(delayed_tf):
    delayed_tf.scan.header.stamp.sec = 2
    tick(delayed_tf, 10.0)
    assert delayed_tf.ready
    assert delayed_tf.accepted_at == 9.8
    assert delayed_tf.accepted_stamp_ns == 1_000_000_000
    assert delayed_tf.diagnostics["latest_scan_sim_age_s"] < 0
    delayed_tf.evidence.observe.assert_not_called()
    tick(delayed_tf, 10.21)
    assert not delayed_tf.ready
    assert delayed_tf.diagnostics["reason"] == "accepted_wall_lease_expired"


def test_recent_acceptance_cannot_extend_acquisition_freshness(delayed_tf):
    delayed_tf.accepted_at = 9.99
    delayed_tf.ros.get_clock.return_value.now.return_value.nanoseconds = 1_500_000_000
    tick(delayed_tf, 10.0)
    assert not delayed_tf.ready
    assert delayed_tf.diagnostics["accepted_wall_age_s"] < 0.4
    assert delayed_tf.diagnostics["accepted_sim_age_s"] > 0.4


def test_clock_before_accepted_scan_revokes_even_a_fresh_wall_lease(delayed_tf):
    delayed_tf.ros.get_clock.return_value.now.return_value.nanoseconds = 900_000_000
    tick(delayed_tf, 10.0)
    assert not delayed_tf.ready
    delayed_tf.evidence.invalidate_free.assert_called_once()


def test_only_successful_observation_renews_both_leases(delayed_tf):
    f = delayed_tf
    f.buffer.lookup_transform.side_effect = None
    f.buffer.lookup_transform.return_value = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    f.scan.header.stamp.nanosec = 100_000_000
    f.scan.ranges = [2.0] * 720
    f.scan.angle_min = -math.pi
    f.scan.angle_increment = math.pi / 360
    f.scan.range_min = 0.5
    f.scan.range_max = 20.0
    f.alignment = None
    f.evidence.observe.return_value = True
    f.ros.get_clock.return_value.now.return_value.nanoseconds = 1_150_000_000
    tick(f, 10.0)
    assert f.ready and f.accepted_at == 10.0
    assert f.accepted_stamp_ns == 1_100_000_000
    f.scan.header.stamp.sec = 2
    f.ros.get_clock.return_value.now.return_value.nanoseconds = 1_250_000_000
    tick(f, 10.2)
    assert f.ready and f.accepted_at == 10.0
    f.ros.get_clock.return_value.now.return_value.nanoseconds = 1_510_000_000
    tick(f, 10.3)
    assert not f.ready  # acquisition age .41s expires before wall age .4s


def test_simulation_wall_jitter_tolerance_does_not_extend_sensor_acquisition_age(delayed_tf):
    f = delayed_tf
    f.config["accepted_wall_timeout_s"] = 1.2
    tick(f, 10.3)  # 0.5s wall age, but sensor acquisition age remains 0.1s
    assert f.ready
    assert f.accepted_at == 9.8  # delayed TF never renews acceptance
    f.ros.get_clock.return_value.now.return_value.nanoseconds = 1_410_000_000
    tick(f, 10.31)
    assert not f.ready
    assert f.diagnostics["reason"] == "accepted_scan_outside_sim_time_lease"


def test_simulation_wall_jitter_tolerance_still_expires_when_clock_is_paused(delayed_tf):
    f = delayed_tf
    f.config["accepted_wall_timeout_s"] = 1.2
    tick(f, 11.01)
    assert not f.ready
    assert f.diagnostics["reason"] == "accepted_wall_lease_expired"
