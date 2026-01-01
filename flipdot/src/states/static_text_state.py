import numpy as np
from PIL import Image, ImageDraw, ImageFont
from python.runfiles import runfiles

from .base_state import FlipDotState


class StaticTextState(FlipDotState):
    def __init__(self, width, height, text, logger):
        super().__init__(width, height)
        self.logger = logger
        self.text = text
        
        # Load font
        rf = runfiles.Create()
        font_path = rf.Rlocation("_main/flipdot/fonts/Tiny5-Regular.ttf")
        # font_path = rf.Rlocation("_main/flipdot/fonts/Times-New-Roman.ttf")
        
        try:
            self.font = ImageFont.truetype(font_path, 8)
        except Exception:
            self.font = ImageFont.load_default()

        # Pre-render the static frame once to save CPU
        self.static_frame = self._prepare_frame()

    def _prepare_frame(self):
        # Create a blank 1-bit image
        image = Image.new('1', (self.width, self.height), color=0)
        draw = ImageDraw.Draw(image)
        
        # Calculate text bounding box for centering
        bbox = draw.textbbox((0, 0), self.text, font=self.font)
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]

        # Centering math
        x = (self.width - text_w) // 2 - bbox[0]
        y = (self.height - text_h) // 2 - bbox[1]

        draw.text((x, y), self.text, font=self.font, fill=1)
        
        # Convert to the numpy format required by the flipdot manager
        return np.array(image).astype(np.uint8)

    def get_frame(self, frame_count, elapsed_time):
        # Just return the pre-rendered frame
        return self.static_frame