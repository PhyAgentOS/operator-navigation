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

    monkeypatch.setitem(sys.modules, "tf2_ros", SimpleNamespace(TransformException=TransformException))
    monkeypatch.setitem(sys.modules, "rclpy.time", SimpleNamespace(Time=SimpleNamespace(from_msg=lambda m: m)))
    fusion = NavigationMapFusion.__new__(NavigationMapFusion)
    fusion.map = SimpleNamespace(header=SimpleNamespace(frame_id="map"))
    fusion.scan = SimpleNamespace(header=SimpleNamespace(frame_id="base_link", stamp=SimpleNamespace(sec=1, nanosec=0)))
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
