import numpy as np
import requests
from PIL import Image, ImageDraw, ImageFont
from python.runfiles import runfiles

from .base_state import FlipDotState


class WeatherState(FlipDotState):
    # raw data dictionary
    ICONS_RAW = {
        "SUN": {
            "data": [
                1, 0, 65, 4, 32, 8, 3, 128, 15, 224, 15, 224, 31, 240,
                223, 246, 31, 240, 15, 224, 15, 224, 3, 128, 32, 8, 65,
                4, 1, 0, 0, 0
            ],
            "w": 16,
            "h": 16
        },
        "CLOUD": {
            "data": [
                3, 192, 7, 224, 15, 240, 127, 254, 255, 255, 255, 255,
                255, 255, 127, 254
            ],
            "w": 16,
            "h": 8
        },
        "RAIN": {"data": [0, 0, 33, 8, 66, 16, 132, 32], "w": 16, "h": 4},
        "SNOW": {
            "data": [0, 0, 0, 0, 0, 16, 16, 40, 40, 16, 17, 0, 2, 128, 1, 0],
            "w": 16,
            "h": 8
        },
        "LIGHTNING": {
            "data": [0, 0, 3, 224, 7, 192, 15, 128, 3, 192, 7, 0, 6, 0, 8, 0],
            "w": 16,
            "h": 8
        }
    }

    def __init__(self, width, height, logger):
        super().__init__(width, height)
        self.logger = logger
        self.lat, self.lon = 42.2808, -83.7430
        self.temp = "--"
        self.weather_code = 0
        self.last_update = 0
        
        # Pre-convert all icons to PIL Images
        self.icons = {name: self._convert_to_pil(name) for name in self.ICONS_RAW}

        # Load font
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

        self._update_weather()

    def _convert_to_pil(self, name):
        """Uses your NumPy logic to create a PIL Image."""
        info = self.ICONS_RAW[name]
        # 1. Create NumPy array from bytes
        raw_arr = np.array(info["data"], dtype=np.uint8)
        # 2. Unpack to bits
        bits = np.unpackbits(raw_arr)
        # 3. Reshape and truncate to exact width/height (ignores padding bits)
        # This prevents the 'ValueError' because NumPy handles the size mismatch
        bitmap = bits.reshape(info["h"], -1)[:, :info["w"]]
        
        # 4. Convert 0/1 array to a PIL Image
        # We multiply by 255 so 1 becomes white (255) and 0 stays black
        return Image.fromarray(bitmap * 255).convert('1')

    def _update_weather(self):
        try:
            url = "https://api.open-meteo.com/v1/forecast"
            params = {
                "latitude": self.lat,
                "longitude": self.lon,
                "current": "temperature_2m,weather_code",
                "temperature_unit": "celsius"
            }
            response = requests.get(url, params=params, timeout=5).json()
            current = response.get("current", {})
            self.temp = f"{int(round(current.get('temperature_2m', 0)))}"
            self.weather_code = current.get("weather_code", 0)
        except Exception as e:
            self.logger.error(f"Weather Error: {e}")

    def get_frame(self, frame_count, elapsed_time):
        # 1. Update logic (every 15 mins)
        if elapsed_time - self.last_update > 900:
            self._update_weather()
            self.last_update = elapsed_time

        # 2. Create base 28x14 canvas
        image = Image.new('1', (self.width, self.height), color=0)
        draw = ImageDraw.Draw(image)
        
        # 3. Map WMO code to icon key
        icon_key = "SUN"
        if 1 <= self.weather_code <= 3: 
            icon_key = "CLOUD"
        elif 51 <= self.weather_code <= 67: 
            icon_key = "RAIN"
        elif 71 <= self.weather_code <= 77: 
            icon_key = "SNOW"
        elif self.weather_code >= 95: 
            icon_key = "LIGHTNING"

        # 4. Paste the icon (from our pre-converted NumPy-to-PIL dictionary)
        icon_img = self.icons[icon_key]
        # Center vertically (14px height - icon height) // 2
        # Note: 16px icons like SUN will paste at y=-1, which is fine!
        icon_y = (self.height - icon_img.height) // 2
        image.paste(icon_img, (0, icon_y))

        # 5. Draw Temperature Text
        # We calculate the bounding box to center it in the right half (x: 14 to 28)
        icon_end_x = icon_img.width
        
        # Calculate remaining width available for text
        remaining_width = self.width - icon_end_x
        
        # Get text dimensions
        bbox = draw.textbbox((0, 0), self.temp, font=self.font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]

        # Centering math: Start at the icon's end, then add half of the leftover space
        text_x = icon_end_x + (remaining_width - tw) // 2
        
        # Vertical centering
        text_y = (self.height - th) // 2 - bbox[1]

        # Draw the text
        draw.text((text_x, text_y), self.temp, font=self.font, fill=1)

        # 6. Optional: Blink a single pixel in the corner if data is old 
        # (Useful for debugging if the WiFi drops)
        if (elapsed_time - self.last_update) > 1000:
            draw.point((27, 0), fill=1)

        # 7. Final output as NumPy array for the flipdots
        return np.array(image).astype(np.uint8)