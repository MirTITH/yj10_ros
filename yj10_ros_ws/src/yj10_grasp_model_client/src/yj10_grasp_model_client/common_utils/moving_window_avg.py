class MovingWindowAvg:
    def __init__(self, window_size):
        self.window_size = window_size
        self.window = []

    def add_value(self, value):
        self.window.append(value)
        if len(self.window) > self.window_size:
            self.window.pop(0)

    def get_average(self):
        if len(self.window) == 0:
            return None
        return sum(self.window) / len(self.window)
