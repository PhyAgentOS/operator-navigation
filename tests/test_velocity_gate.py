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


@pytest.mark.parametrize("failure", ["control lease expired", "localization lease expired"])
def test_processing_delay_cannot_retimestamp_expired_authority(monkeypatch, failure):
    from navigation import velocity

    gate = VelocityGate()
    gate.set_enabled(True)
    gate.receive(.05, 0, 0, now=10.39)  # even a new Twist cannot renew authority
    monkeypatch.setattr(velocity.time, "monotonic_ns", lambda: 10_400_000_000)
    cancellations = []
    stamp, command = velocity.sample_authorized_output(
        gate, active=True, execution_ready=True,
        permission_failure=lambda now: failure if now - 10 >= .35 else None,
        cancel=cancellations.append,
    )
    assert stamp == 10_400_000_000
    assert command.vx == 0 and not gate.enabled
    assert cancellations == [failure + "，已停车"]


@pytest.mark.parametrize("received,expected", [(10., 0.), (10.39, .05)])
def test_output_uses_current_time_for_old_and_new_twists(monkeypatch, received, expected):
    from navigation import velocity

    gate = VelocityGate()
    gate.set_enabled(True)
    gate.receive(.05, 0, 0, now=received)
    monkeypatch.setattr(velocity.time, "monotonic_ns", lambda: 10_400_000_000)
    cancellations = []
    stamp, command = velocity.sample_authorized_output(
        gate, active=True, execution_ready=True,
        permission_failure=lambda now: None, cancel=cancellations.append,
    )
    assert command.vx == expected and gate.enabled
    assert stamp == 10_400_000_000 and not cancellations


@pytest.mark.parametrize("active,ready", [(False, True), (True, False)])
def test_completed_or_cancelling_task_never_emits_previous_velocity(monkeypatch, active, ready):
    from navigation import velocity

    gate = VelocityGate()
    gate.set_enabled(True)
    gate.receive(.05, 0, 0, now=10.)
    monkeypatch.setattr(velocity.time, "monotonic_ns", lambda: 10_010_000_000)
    _, command = velocity.sample_authorized_output(
        gate, active=active, execution_ready=ready,
        permission_failure=lambda now: None, cancel=lambda _: None,
    )
    assert command.vx == 0 and not gate.enabled
