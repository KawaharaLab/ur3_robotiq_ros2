"""Bounded, constant-memory timing summaries; no ROS dependency."""
import math
import threading
import time


class TimingWindow:
    def __init__(self, clock=time.monotonic_ns):
        self.clock = clock
        self.lock = threading.Lock()
        self.start_ns = clock()
        self.counters = {}
        self.metrics = {}

    def add(self, counters=None, metrics=None):
        with self.lock:
            for key, count in (counters or {}).items():
                self.counters[key] = self.counters.get(key, 0) + count
            for key, value in (metrics or {}).items():
                if value is None or not math.isfinite(value):
                    continue
                if key not in self.metrics:
                    self.metrics[key] = [0, 0.0, value, value]
                stats = self.metrics[key]
                stats[0] += 1
                stats[1] += value
                stats[2] = min(stats[2], value)
                stats[3] = max(stats[3], value)

    def snapshot(self):
        with self.lock:
            end_ns = self.clock()
            elapsed = (end_ns - self.start_ns) / 1e9
            result = {
                "window_seconds": elapsed,
                "counters": self.counters,
                "rates_hz": {key: count / elapsed if elapsed > 0 else None
                             for key, count in self.counters.items()},
                "metrics": {key: {"count": stats[0], "mean": stats[1] / stats[0],
                                  "min": stats[2], "max": stats[3]}
                            for key, stats in self.metrics.items()},
            }
            self.counters, self.metrics = {}, {}
            self.start_ns = end_ns
            return result
