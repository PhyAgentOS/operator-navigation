from forge_msgs import PolicyCommand, PolicyCommandStatus

from navigation.policy import PolicyEndpoint


class Sink:
    def __init__(self):
        self.values = []

    def send_output(self, port, value):
        assert port == "status"
        self.values.append(PolicyCommandStatus.from_arrow(value))


def event(command):
    return {"value": command.to_arrow()}


def test_replayed_request_does_not_execute_again_and_replays_terminal_result():
    sink = Sink()
    endpoint = PolicyEndpoint(sink, "navigation")
    command = PolicyCommand.from_inputs(
        policy_id="navigation",
        command="navigate_to_pose",
        request_id="task-1",
        inputs={"pose": [1, 2, 0]},
    )
    first = endpoint.receive(event(command))
    assert first is not None
    endpoint.emit(first, "running")
    endpoint.emit(first, "done", outputs={"result": "succeeded"})
    assert endpoint.receive(event(command)) is None
    assert sink.values[-1].outputs() == {"result": "succeeded"}


def test_reused_id_with_different_target_is_rejected_without_replacing_original_cache():
    sink = Sink()
    endpoint = PolicyEndpoint(sink, "navigation")
    first = PolicyCommand.from_inputs(
        policy_id="navigation",
        command="navigate_to_pose",
        request_id="task-1",
        inputs={"pose": [1, 2, 0]},
    )
    endpoint.emit(first, "done", outputs={"result": "succeeded"})
    changed = PolicyCommand.from_inputs(
        policy_id="navigation",
        command="navigate_to_pose",
        request_id="task-1",
        inputs={"pose": [9, 2, 0]},
    )
    assert endpoint.receive(event(changed)) is None
    assert sink.values[-1].status == "rejected"
    endpoint.receive(event(first))
    assert sink.values[-1].status == "done"
