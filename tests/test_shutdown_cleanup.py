"""Execute the real finalizer with failing transports/backends, without a robot."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navigation.velocity import VelocityGate


@pytest.mark.parametrize("step", ["send", "cancel", "backend", "destroy", "shutdown"])
def test_snapshot_worker_closes_when_other_cleanup_raises(step):
    # Compile the production finalizer itself: no reimplementation of its order
    # or exception handling, and no ROS graph or native process is needed.
    path = Path(__file__).parents[1] / "src/navigation/node.py"
    tree = ast.parse(path.read_text())
    run = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    finalizer = next(n for n in reversed(run.body) if isinstance(n, ast.Try)).finalbody
    code = compile(ast.Module(body=finalizer, type_ignores=[]), str(path), "exec")
    steps = {name: Mock() for name in ["send", "cancel", "backend", "destroy", "shutdown"]}
    steps[step].side_effect = RuntimeError(step)
    close = Mock()
    gate = VelocityGate()
    gate.set_enabled(True)
    scope = {
        "gate": gate,
        "dora": SimpleNamespace(send_output=steps["send"]),
        "time": SimpleNamespace(monotonic=lambda: 10., monotonic_ns=lambda: 10_000_000_000),
        "request_cancel": steps["cancel"],
        "backend": SimpleNamespace(stop=steps["backend"]),
        "ros": SimpleNamespace(destroy_node=steps["destroy"]),
        "rclpy": SimpleNamespace(shutdown=steps["shutdown"]),
        "snapshot_writer": SimpleNamespace(close=close),
    }
    with pytest.raises(RuntimeError, match=step):
        exec(code, scope)
    assert not gate.enabled
    close.assert_called_once_with()
