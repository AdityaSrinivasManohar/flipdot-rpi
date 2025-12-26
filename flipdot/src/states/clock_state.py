from datetime import datetime

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from python.runfiles import runfiles

from .base_state import FlipDotState


class ClockState(FlipDotState):
    def __init__(self, width, height, logger):
        super().__init__(width, height)
        self.logger = logger

        rf = runfiles.Create()
        font_path = rf.Rlocation("_main/flipdot/fonts/Tiny5-Regular.ttf")
        self.logger.info(f"font path is - {font_path}")
        
        if not font_path:
            raise FileNotFoundError("Could not find Tiny5 font in Bazel runfiles.")
        
        try:
            self.font = ImageFont.truetype(font_path, 8)
        except Exception as e:
            self.logger.warn(f"Error - {e}")
            self.logger.warn("Font file not found, falling back to default.")
            self.font = ImageFont.load_default()
    
    def get_frame(self, frame_count, elapsed_time):
        # 1. Create a black 1-bit image (mode '1')
        image = Image.new('1', (self.width, self.height), color=0)
        draw = ImageDraw.Draw(image)

        # 2. Get the time string with blinking colon
        time_format = "%H:%M" if int(elapsed_time) % 2 == 0 else "%H %M"
        time_str = datetime.now().strftime(time_format)

        # 3. Dynamic Centering Logic
        # textbbox returns (left, top, right, bottom)
        bbox = draw.textbbox((0, 0), time_str, font=self.font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        # Calculate coordinates to center text
        # We subtract bbox[0] and bbox[1] to account for font internal offsets (padding)
        x = (self.width - text_width) // 2 + 1
        y = (self.height - text_height) // 2 - bbox[1]

        # 4. Draw the text
        draw.text((x, y), time_str, font=self.font, fill=1)

        # 5. Convert to NumPy array for Flipdot
        return np.array(image).astype(np.uint8)