from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from python.runfiles import runfiles

from .base_state import FlipDotState


class ClockState(FlipDotState):
    def __init__(self, width, height, logger):
        super().__init__(width, height)
        self.logger = logger

        self.timezone = ZoneInfo("America/Detroit")

        rf = runfiles.Create()
        font_path = rf.Rlocation("_main/flipdot/fonts/Tiny5-Regular.ttf")
        
        if not font_path:
            raise FileNotFoundError("Could not find Tiny5 font in Bazel runfiles.")
        
        try:
            self.font = ImageFont.truetype(font_path, 8)
        except Exception as e:
            self.logger.warn(f"Error - {e}")
            self.logger.warn("Font file not found, falling back to default.")
            self.font = ImageFont.load_default()
    
    def get_frame(self, frame_count, elapsed_time):
        image = Image.new('1', (self.width, self.height), color=0)
        draw = ImageDraw.Draw(image)

        local_now = datetime.now(self.timezone)

        # to make the : show up every sec
        if int(elapsed_time) % 2 == 0:
            time_format = "%H:%M"
        else:
            time_format = "%H %M"
        
        time_str = local_now.strftime(time_format)

        # Centering Logic
        bbox = draw.textbbox((0, 0), time_str, font=self.font, anchor="lt")
        
        # Calculate actual rendered dimensions
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]

        # Calculate coordinates using floor division for integer pixel alignment
        # add a small offset if the font has an internal top-padding (bbox[1])
        x = (self.width - text_w) // 2
        y = (self.height - text_h) // 2 - bbox[1]

        draw.text((int(x), int(y)), time_str, font=self.font, fill=1, anchor="lt")

        return np.array(image).astype(np.uint8)