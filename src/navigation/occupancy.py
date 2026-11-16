"""Finite-return evidence over an immutable prior, independent of ROS and robot model."""

import math

import numpy as np


class OccupancyEvidence:
    """Finite-ray clearing plus optional bounded memory for scan-only obstacles.

    Missing/NaN/infinite beams provide no evidence. A finite return can clear
    cells strictly before its endpoint, after repeated observations. This is a
    planar sensor model, not a claim of complete 3D volume visibility. Dynamic
    expiry withdraws an old observation, returning to the prior; it is not free
    evidence and cannot erase a prior obstacle or open unexplored space.
    """

    def __init__(
        self,
        prior,
        resolution,
        origin,
        *,
        observation_range_m=4.0,
        clear_confirmations=3,
        clear_min_seconds=0.3,
        free_memory_seconds=30.0,
        endpoint_margin_m=0.15,
        dynamic_memory_seconds=0.0,
    ):
        self.prior = np.asarray(prior, dtype=np.int16).copy()
        if self.prior.ndim != 2 or not np.isin(self.prior, np.arange(-1, 101)).all():
            raise ValueError("invalid occupancy grid")
        self.resolution = float(resolution)
        self.origin = tuple(origin)
        if self.resolution <= 0 or not all(math.isfinite(v) for v in (*origin, resolution)):
            raise ValueError("invalid grid geometry")
        self.radius = observation_range_m
        self.confirmations = clear_confirmations
        self.minimum_time = clear_min_seconds
        self.memory = free_memory_seconds
        self.margin = max(endpoint_margin_m, resolution * math.sqrt(2))
        if not math.isfinite(dynamic_memory_seconds) or dynamic_memory_seconds < 0:
            raise ValueError("dynamic_memory_seconds must be finite and nonnegative")
        self.dynamic_memory = dynamic_memory_seconds
        self.hits = np.zeros(self.prior.shape, dtype=bool)
        self.hit_until = np.zeros(self.prior.shape)
        self.observation_clock = 0.0
        self.observed_at = None
        self.free_until = np.zeros(self.prior.shape)
        self.count = np.zeros(self.prior.shape, dtype=np.uint16)
        self.first = np.zeros(self.prior.shape)
        self.last = np.full(self.prior.shape, -np.inf)
        self.last_stamp = -np.inf

    def invalidate_free(self):
        self.free_until.fill(0)
        self.count.fill(0)
        self.last.fill(-np.inf)
        self.observed_at = None

    def reset(self):
        self.invalidate_free()
        self.hits.fill(False)
        self.hit_until.fill(0)
        self.observation_clock = 0.0
        self.observed_at = None
        self.last_stamp = -np.inf

    def observe(self, ranges, angle_min, angle_increment, range_min, range_max, pose, stamp, now):
        if stamp <= self.last_stamp:
            return False
        if not all(
            math.isfinite(v)
            for v in (*pose, stamp, now, angle_min, angle_increment, range_min, range_max)
        ):
            return False
        if angle_increment <= 0 or range_max <= range_min or range_min < 0:
            return False
        ranges = np.asarray(ranges, dtype=float)
        if ranges.ndim != 1 or not len(ranges):
            return False
        self.last_stamp = stamp
        x, y, yaw = pose
        h, w = self.prior.shape
        ox, oy = self.origin
        r = self.resolution
        x0, x1 = (
            max(0, int(math.floor((x - self.radius - ox) / r))),
            min(w, int(math.ceil((x + self.radius - ox) / r))),
        )
        y0, y1 = (
            max(0, int(math.floor((y - self.radius - oy) / r))),
            min(h, int(math.ceil((y + self.radius - oy) / r))),
        )
        if x1 <= x0 or y1 <= y0:
            return False
        valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
        if not valid.any():
            # A blank projected scan is not evidence of a working observation
            # stream. It must not age hits or renew the motion-ready lease.
            self.observed_at = None
            return False
        if self.observed_at is not None and 0 <= now - self.observed_at <= 0.4:
            self.observation_clock += now - self.observed_at
        self.observed_at = now
        if self.dynamic_memory:
            self.hits[self.hit_until <= self.observation_clock] = False
        # A hit is applied immediately, including close to the observation horizon.
        angles = angle_min + np.arange(len(ranges)) * angle_increment + yaw
        hit = valid & (ranges <= self.radius)
        ix = np.floor((x + ranges[hit] * np.cos(angles[hit]) - ox) / r).astype(int)
        iy = np.floor((y + ranges[hit] * np.sin(angles[hit]) - oy) / r).astype(int)
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[inside], iy[inside]
        # Cell-center evidence, restricted to the actually observed finite ray.
        xx, yy = np.meshgrid(
            ox + (np.arange(x0, x1) + 0.5) * r - x, oy + (np.arange(y0, y1) + 0.5) * r - y
        )
        distance = np.hypot(xx, yy)
        bearing = np.arctan2(yy, xx) - yaw
        bearing = (bearing - angle_min) % (2 * math.pi) + angle_min
        bins = np.rint((bearing - angle_min) / angle_increment).astype(int)
        in_scan = (bins >= 0) & (bins < len(ranges))
        bins = np.clip(bins, 0, len(ranges) - 1)
        angular_error = bearing - (angle_min + bins * angle_increment)
        free = (
            in_scan
            & valid[bins]
            & (distance >= range_min + r)
            & (distance <= self.radius)
            & (distance < ranges[bins] - self.margin)
            & (np.abs(distance * np.sin(angular_error)) <= r * 0.35)
        )
        sl = np.s_[y0:y1, x0:x1]
        # One nearer endpoint wins over all clearing rays in this update.
        hit_cells = np.zeros(free.shape, dtype=bool)
        local = (ix >= x0) & (ix < x1) & (iy >= y0) & (iy < y1)
        hit_cells[iy[local] - y0, ix[local] - x0] = True
        free &= ~hit_cells
        count, first, last = self.count[sl], self.first[sl], self.last[sl]
        continuing = free & (now - last <= 0.5)
        first[free & ~continuing] = now
        # Missing/occluded bins are neutral, not contrary evidence. Keep a
        # short evidence sequence across sparse scans; actual hits reset below.
        count[now - last > 0.5] = 0
        count[free] = np.where(continuing[free], np.minimum(count[free], 60000) + 1, 1)
        last[free] = now
        confirmed = free & (count >= self.confirmations) & (now - first >= self.minimum_time)
        self.hits[sl][confirmed] = False
        # Never invent free space outside the known prior's exploration boundary.
        self.free_until[sl][confirmed & (self.prior[sl] >= 0)] = now + self.memory
        self.hits[iy, ix] = True
        self.hit_until[iy, ix] = self.observation_clock + self.dynamic_memory
        self.free_until[iy, ix] = 0
        self.count[iy, ix] = 0
        return True

    def render(self, now, *, reliable=True):
        output = self.prior.copy()
        if reliable:
            output[self.free_until > now] = 0
        output[self.hits] = 100
        return output
