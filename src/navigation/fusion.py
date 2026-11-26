"""Optional shared navigation map; localization continues to consume the prior."""

import copy
import hashlib
import math

import numpy as np

from .occupancy import OccupancyEvidence


class NavigationMapFusion:
    def __init__(self, ros, buffer, config, snapshot_dir=None):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan

        self.ros, self.buffer, self.config = ros, buffer, config
        self.snapshot_dir = snapshot_dir
        self.map = self.evidence = self.scan = None
        self.fingerprint = None
        self.processed_stamp = -1
        self.last_processed = self.last_publish = self.accepted_at = -math.inf
        self.accepted_stamp_ns = None
        self.diagnostics = {"ready": False, "reason": "no_accepted_observation"}
        self.last_snapshot = -math.inf
        self.alignment = None
        self.ready = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = ros.create_publisher(OccupancyGrid, config["output_topic"], qos)
        self.subscriptions = [
            ros.create_subscription(OccupancyGrid, config["prior_topic"], self.on_map, qos),
            ros.create_subscription(
                LaserScan,
                config["scan_topic"],
                self.on_scan,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            ),
        ]

    def on_scan(self, scan):
        self.scan = scan

    def on_map(self, msg):
        q = msg.info.origin.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(abs(q.w) - 1) > 1e-6:
            raise ValueError("navigation fusion requires an axis-aligned occupancy grid")
        data = np.asarray(msg.data, dtype=np.int16)
        fingerprint = (
            msg.header.frame_id,
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            hashlib.sha256(data.tobytes()).digest(),
        )
        if fingerprint == self.fingerprint:
            return
        self.fingerprint, self.map = fingerprint, copy.deepcopy(msg)
        self.evidence = OccupancyEvidence(
            data.reshape(msg.info.height, msg.info.width),
            msg.info.resolution,
            (msg.info.origin.position.x, msg.info.origin.position.y),
            dynamic_memory_seconds=self.config.get("dynamic_memory_seconds", 0.0),
            **{
                k: self.config[k]
                for k in (
                    "observation_range_m",
                    "clear_confirmations",
                    "clear_min_seconds",
                    "free_memory_seconds",
                    "endpoint_margin_m",
                )
            },
        )
        self.ready = False
        self.accepted_at = self.last_publish = -math.inf
        self.accepted_stamp_ns = None
        self.alignment = None

    @staticmethod
    def pose(transform):
        p, q = transform.transform.translation, transform.transform.rotation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def tick(self, health, now):
        from rclpy.time import Time
        from tf2_ros import TransformException

        healthy = (
            health.get("healthy") is True
            and 0 <= now - health.get("issued_monotonic", -math.inf) < 0.65
        )
        wall_timeout = self.config.get("accepted_wall_timeout_s", 0.4)
        reliable = False
        scan_age = None
        tf_error = None
        clock_ns = self.ros.get_clock().now().nanoseconds
        if self.map is not None and self.scan is not None and healthy:
            scan = self.scan
            stamp = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
            age = (clock_ns - stamp) / 1e9
            scan_age = age
            if 0 <= age <= 0.4:
                if stamp != self.processed_stamp and now - self.last_processed >= 0.095:
                    try:
                        at = Time.from_msg(scan.header.stamp)
                        tf = self.buffer.lookup_transform(
                            self.map.header.frame_id, scan.header.frame_id, at
                        )
                        alignment = self.pose(
                            self.buffer.lookup_transform(
                                self.map.header.frame_id, self.config["odom_frame"], at
                            )
                        )
                        if self.alignment is not None:
                            dx = math.hypot(
                                alignment[0] - self.alignment[0], alignment[1] - self.alignment[1]
                            )
                            dyaw = abs(
                                math.atan2(
                                    math.sin(alignment[2] - self.alignment[2]),
                                    math.cos(alignment[2] - self.alignment[2]),
                                )
                            )
                            if dx > 0.20 or dyaw > 0.15:
                                self.evidence.reset()
                        self.alignment = alignment
                        if self.evidence.observe(
                            scan.ranges,
                            scan.angle_min,
                            scan.angle_increment,
                            scan.range_min,
                            scan.range_max,
                            self.pose(tf),
                            stamp / 1e9,
                            now,
                        ):
                            self.accepted_at = now
                            self.accepted_stamp_ns = stamp
                        self.processed_stamp, self.last_processed = stamp, now
                    except TransformException as error:
                        # A new scan can arrive before its timestamped TF. Retry
                        # it on the next tick; do not discard the previous valid
                        # observation inside its existing 0.4 s freshness bound.
                        # This neither accepts the new scan nor renews the lease.
                        tf_error = str(error)
            # A pending/future NEW scan does not invalidate the previous
            # accepted observation. It also cannot refresh either lease.
            # Check both acquisition age and wall age of that accepted scan.
            wall_age = now - self.accepted_at
            sim_age = None if self.accepted_stamp_ns is None else (clock_ns - self.accepted_stamp_ns) / 1e9
            reliable = (0 <= wall_age < wall_timeout and sim_age is not None and 0 <= sim_age <= 0.4)
        else:
            wall_age = now - self.accepted_at
            sim_age = None if self.accepted_stamp_ns is None else (clock_ns - self.accepted_stamp_ns) / 1e9
        reason = ("ready" if reliable else "localization_unhealthy" if not healthy
                  else "missing_map_or_scan" if self.map is None or self.scan is None
                  else "no_accepted_observation" if self.accepted_stamp_ns is None
                  else "accepted_wall_lease_expired" if not 0 <= wall_age < wall_timeout
                  else "accepted_scan_outside_sim_time_lease")
        self.diagnostics = {"ready": reliable, "reason": reason,
                            "accepted_wall_age_s": wall_age if math.isfinite(wall_age) else None,
                            "accepted_sim_age_s": sim_age, "latest_scan_sim_age_s": scan_age,
                            "latest_tf_error": tf_error, "accepted_wall_timeout_s": wall_timeout,
                            "accepted_sim_timeout_s": 0.4}
        self.ready = reliable
        if self.evidence is None:
            return
        if not reliable:
            self.evidence.invalidate_free()
        if now - self.last_publish < 0.095:
            return
        msg = copy.deepcopy(self.map)
        msg.header.stamp = self.ros.get_clock().now().to_msg()
        data = self.evidence.render(now, reliable=reliable)
        msg.data = data.astype(np.int8).ravel().tolist()
        self.publisher.publish(msg)
        self.last_publish = now
        if self.snapshot_dir and now - self.last_snapshot >= 0.5:
            from .snapshot import write_snapshot

            write_snapshot(
                self.snapshot_dir,
                "navigation_map",
                {
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "resolution": msg.info.resolution,
                    "frame_id": msg.header.frame_id,
                    "origin": [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
                    "data": list(msg.data),
                    "reliable": reliable,
                    "observation_health": self.diagnostics,
                },
            )
            self.last_snapshot = now
