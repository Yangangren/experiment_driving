import time
import numpy as np


class TimerStat:
    def __init__(self, window_size=10):
        self._window_size = window_size
        self._samples = []
        self._units_processed = []
        self._start_time = None
        self._total_time = 0.0
        self.count = 0

    def __enter__(self):
        assert self._start_time is None, "concurrent updates not supported"
        self._start_time = time.time()

    def __exit__(self, type, value, tb):
        assert self._start_time is not None
        time_delta = time.time() - self._start_time
        self.push(time_delta)
        self._start_time = None

    def push(self, time_delta):
        self._samples.append(time_delta)
        if len(self._samples) > self._window_size:
            self._samples.pop(0)
        self.count += 1
        self._total_time += time_delta

    def push_units_processed(self, n):
        self._units_processed.append(n)
        if len(self._units_processed) > self._window_size:
            self._units_processed.pop(0)

    def has_units_processed(self):
        return len(self._units_processed) > 0

    @property
    def mean(self):
        if not self._samples:
            return 0.0
        return float(np.mean(self._samples))

    @property
    def mean_units_processed(self):
        if not self._units_processed:
            return 0.0
        return float(np.mean(self._units_processed))

    @property
    def mean_throughput(self):
        time_total = float(sum(self._samples))
        if not time_total:
            return 0.0
        return float(sum(self._units_processed)) / time_total


class ValueSmooth:
    def __init__(self, window_size=10):
        self._window_size = window_size
        self._samples = []

    def push(self, sample):
        self._samples.append(sample)
        if len(self._samples) > self._window_size:
            self._samples.pop(0)

    @property
    def mean(self):
        if not self._samples:
            return 0
        return np.mean(np.array(self._samples), axis=0)
