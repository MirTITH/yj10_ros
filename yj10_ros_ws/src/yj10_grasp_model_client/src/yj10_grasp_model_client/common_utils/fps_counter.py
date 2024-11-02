from collections import deque
import time


class FrameRateCounter:
    def __init__(self, window_size=100):
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)

    def tick(self):
        current_time = time.time()
        self.timestamps.append(current_time)

    def get_fps(self, window_size=None):
        if window_size is None:
            window_size = self.window_size

        num_timestamps = min(len(self.timestamps), window_size)
        if num_timestamps <= 1:
            return 0.0

        return (num_timestamps - 1) / (self.timestamps[-1] - self.timestamps[-num_timestamps])

    def print_info(self, prefix=""):
        fps = self.get_fps(2)
        avg_fps = self.get_fps()
        print(f"{prefix}FPS: {fps:.2f}, Avg FPS: {avg_fps:.2f}")
