"""Pure velocity gate, with a monotonic watchdog and explicit task authority.

The later ROS/Dora bridge must call output() on every timer tick, including
while no ROS command arrives. Receipt of Twist alone never arms movement.
"""

import math

from forge_msgs import LocomotionCommand


class VelocityGate:
    def __init__(self, *, max_vx: float = 0.08, max_wz: float = 0.4, timeout_seconds: float = 0.25):
        if any(not math.isfinite(v) or v <= 0 for v in (max_vx, max_wz, timeout_seconds)):
            raise ValueError("limits and timeout must be positive and finite")
        self.max_vx, self.max_wz, self.timeout = max_vx, max_wz, timeout_seconds
        self.enabled = False
        self._last = None
        self._command = LocomotionCommand(vx=0, vy=0, wz=0)

    def set_enabled(self, enabled: bool) -> None:
        # Re-enabling requires a fresh velocity; never resurrect a previous task.
        self.enabled = bool(enabled)
        self._last = None
        self._command = LocomotionCommand(vx=0, vy=0, wz=0)

    def receive(self, vx: float, vy: float, wz: float, *, now: float) -> None:
        if not all(math.isfinite(v) for v in (vx, vy, wz, now)) or vy != 0:
            self.set_enabled(False)
            raise ValueError("finite differential-drive command required; vy must be zero")
        if not self.enabled:
            return
        if self._last is not None and now < self._last:
            self.set_enabled(False)
            raise ValueError("monotonic clock regressed")
        self._command = LocomotionCommand(
            vx=max(-self.max_vx, min(self.max_vx, vx)),
            vy=0,
            wz=max(-self.max_wz, min(self.max_wz, wz)),
        )
        self._last = now

    def output(self, *, now: float) -> LocomotionCommand:
        if (
            not self.enabled
            or self._last is None
            or not math.isfinite(now)
            or not 0 <= now - self._last < self.timeout
        ):
            return LocomotionCommand(vx=0, vy=0, wz=0)
        return self._command.model_copy()
