import sys
from types import SimpleNamespace as NS

import pytest

from navigation.costmap_reset import CostmapRefresh


class Future:
    complete = False
    canceled = False

    def done(self):
        return self.complete

    def result(self):
        return NS()

    def cancel(self):
        self.canceled = True


class Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future


def refresh(monkeypatch):
    monkeypatch.setitem(
        sys.modules, "nav2_msgs.srv", NS(ClearEntireCostmap=NS(Request=lambda: NS()))
    )
    clients = []

    def create(*args):
        c = Client()
        clients.append(c)
        return c

    return CostmapRefresh(NS(create_client=create)), clients


def test_both_clears_and_fresh_observation_wait_are_required(monkeypatch):
    task, clients = refresh(monkeypatch)
    task.start(0)
    assert task.poll(0) is None
    clients[0].calls[-1].complete = True
    assert task.poll(0.1) is None
    clients[1].calls[-1].complete = True
    assert task.poll(0.2) is None
    assert task.poll(0.9) is None
    assert task.poll(1.3) is True
    assert all(len(c.calls) == 1 for c in clients)


def test_lost_clear_response_has_idempotent_retry_and_total_timeout(monkeypatch):
    task, clients = refresh(monkeypatch)
    task.start(0)
    task.poll(0)
    first = [c.calls[0] for c in clients]
    assert task.poll(2.1) is None
    assert all(f.canceled for f in first)
    assert all(len(c.calls) == 2 for c in clients)
    with pytest.raises(TimeoutError):
        task.poll(8.1)
