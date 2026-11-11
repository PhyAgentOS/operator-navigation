import pytest

from navigation.velocity import VelocityGate


def test_authority_watchdog_and_fresh_command_after_cancel():
    gate = VelocityGate()
    gate.receive(0.1, 0, 0.5, now=1)
    assert gate.output(now=1).vx == 0
    gate.set_enabled(True)
    gate.receive(0.1, 0, 0.5, now=2)
    cmd = gate.output(now=2.1)
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.08, 0, 0.4)
    assert gate.output(now=2.25).vx == 0
    gate.set_enabled(False)
    assert gate.output(now=2.1).vx == 0
    gate.set_enabled(True)
    assert gate.output(now=2.1).vx == 0


@pytest.mark.parametrize("vx,vy,wz", [(float("nan"), 0, 0), (0, 0.01, 0), (0, 0, float("inf"))])
def test_invalid_velocity_disarms_until_reenabled(vx, vy, wz):
    gate = VelocityGate()
    gate.set_enabled(True)
    gate.receive(0.05, 0, 0, now=0)
    with pytest.raises(ValueError):
        gate.receive(vx, vy, wz, now=0.1)
    assert gate.output(now=0.1).vx == 0
    assert not gate.enabled
