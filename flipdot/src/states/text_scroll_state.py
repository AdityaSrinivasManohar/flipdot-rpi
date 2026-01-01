import numpy as np
from PIL import Image, ImageDraw, ImageFont
from python.runfiles import runfiles

from .base_state import FlipDotState


class TextScrollState(FlipDotState):
    def __init__(self, width, height, text, logger, speed=1):
        super().__init__(width, height)
        self.logger = logger
        self.text = text

        # Pixels per frame, since the display refreshes at 10hz
        # speed of 1 would move the text at 10hz as well
        self.speed = speed
        
        # Load font
        rf = runfiles.Create()
        font_path = rf.Rlocation("_main/flipdot/fonts/Tiny5-Regular.ttf")
        
        try:
            self.font = ImageFont.truetype(font_path, 8)
        except Exception:
            self.font = ImageFont.load_default()

        # Pre-render the entire text into a long buffer
        self._prepare_buffer()

    def _prepare_buffer(self):
        # Calculate how wide the image needs to be
        temp_draw = ImageDraw.Draw(Image.new('1', (1, 1)))
        bbox = temp_draw.textbbox((0, 0), self.text, font=self.font)
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]

        # Create a buffer with extra space at the end so it loops cleanly
        self.buffer_width = text_w + self.width
        self.text_image = Image.new('1', (self.buffer_width, self.height), color=0)
        draw = ImageDraw.Draw(self.text_image)
        
        # Center vertically in the 14px height
        y = (self.height - text_h) // 2 - bbox[1]
        draw.text((self.width, y), self.text, font=self.font, fill=1)

    def get_frame(self, frame_count, elapsed_time):
        # Calculate current horizontal offset
        # Wrap around using modulo (%) so it loops forever
        offset = int(frame_count * self.speed) % self.buffer_width
        
        # Create the display frame
        image = Image.new('1', (self.width, self.height), color=0)
        
        # Paste the portion of the long text image onto our 28x14 display
        # We paste the buffer at a negative offset to "slide" it
        image.paste(self.text_image, (-offset, 0))
        
        # If the end of the text is on screen, paste the beginning again to loop
        if offset > self.buffer_width - self.width:
            image.paste(self.text_image, (self.buffer_width - offset, 0))

        return np.array(image).astype(np.uint8)