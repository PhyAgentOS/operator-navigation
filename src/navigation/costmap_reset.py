"""Idempotent native costmap refresh, bounded retries and fresh-observation delay."""


class CostmapRefresh:
    def __init__(self, ros):
        from nav2_msgs.srv import ClearEntireCostmap

        self.request_type = ClearEntireCostmap.Request
        self.clients = [
            ros.create_client(ClearEntireCostmap, name)
            for name in (
                "/local_costmap/clear_entirely_local_costmap",
                "/global_costmap/clear_entirely_global_costmap",
            )
        ]
        self.started = self.settle_at = None
        self.futures = {}
        self.sent_at = {}

    def start(self, now):
        self.started, self.settle_at = now, None
        self.futures, self.sent_at = {}, {}

    def poll(self, now):
        """None means pending; True means refreshed; raises on bounded failure."""
        if self.started is None:
            return None
        if now - self.started > 8:
            raise TimeoutError("代价地图刷新超时，请重试定位")
        for index, client in enumerate(self.clients):
            future = self.futures.get(index)
            if future is not None and not future.done() and now - self.sent_at[index] > 2:
                future.cancel()
                del self.futures[index]
                future = None
            if future is None and client.service_is_ready():
                self.futures[index] = client.call_async(self.request_type())
                self.sent_at[index] = now
        if len(self.futures) != len(self.clients) or not all(
            f.done() for f in self.futures.values()
        ):
            return None
        if any(f.result() is None for f in self.futures.values()):
            raise RuntimeError("代价地图刷新未返回有效结果")
        if self.settle_at is None:
            self.settle_at = now
        return True if now - self.settle_at >= 1.0 else None
