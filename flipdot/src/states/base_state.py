import numpy as np


class FlipDotState:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.start_time = None

    def on_enter(self):
        """Called once when switching TO this state."""
        pass

    def get_frame(self, frame_count, elapsed_time) -> np.ndarray:
        """Must return a numpy array of (height, width)."""
        raise NotImplementedError