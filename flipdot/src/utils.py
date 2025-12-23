"""Some utils for converting between numpy arrays and FlipDotFrame messages."""

import numpy as np
from flipdotframe_msg.msg import FlipDotFrame


class FlipDotRosConverter:
    @staticmethod
    def to_msg(grid: np.ndarray) -> FlipDotFrame:
        """
        Converts a 2D numpy array (0s and 1s) into a FlipDotFrame message.
        """
        msg = FlipDotFrame()
        msg.height, msg.width = grid.shape

        # Flatten and pack: 8 pixels -> 1 byte
        # bitorder='little' ensures Bit 0 is the first pixel (0,0)
        packed = np.packbits(grid.flatten(), bitorder='little')
        msg.data = packed.tobytes()
        return msg

    @staticmethod
    def from_msg(msg: FlipDotFrame) -> np.ndarray:
        """
        Converts a FlipDotFrame message back into a 2D numpy array.
        """
        # Convert byte string back to uint8 array
        data_uint8 = np.frombuffer(msg.data, dtype=np.uint8)

        # Unpack bits into a 1D array of 0s and 1s
        unpacked = np.unpackbits(data_uint8, bitorder='little')

        # Truncate to the actual number of pixels (in case of padding)
        # and reshape back to the display dimensions
        total_pixels = msg.width * msg.height
        return unpacked[:total_pixels].reshape((msg.height, msg.width))
