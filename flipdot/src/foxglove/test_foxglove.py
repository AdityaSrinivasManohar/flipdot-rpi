import foxglove
import time
from foxglove.schemas import RawImage, Timestamp
import numpy as np

def main():
    # 1. Start the Foxglove WebSocket server
    # By default, this listens on ws://localhost:8765
    server = foxglove.start_server(host="0.0.0.0", port=8765)
    print("Foxglove server started on ws://localhost:8765")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            # 2. Log a simple dictionary (this becomes a JSON message in Foxglove)
            foxglove.log("/test_topic", {
                "msg": "Hello Flipdot!",
                "count": time.time()
            })
            
            # Sleep to keep the loop from hogging CPU
            time.sleep(1.0)

            # --- CHANNEL 1: Image Data ---
            # Create a dummy 28x28 grayscale image (simulating a Flipdot display)
            # data: bytes, encoding: 'mono8', width, height, step (bytes per row)
            width, height = 28, 28
            pixel_data = np.random.randint(0, 255, (height, width), dtype=np.uint8).tobytes()
            
            foxglove.log(
                "/flipdot/image",
                RawImage(
                    timestamp=Timestamp.now(),
                    frame_id="flipdot_board",
                    width=width,
                    height=height,
                    encoding="mono8",
                    step=width,
                    data=pixel_data
                )
            )
            
    except KeyboardInterrupt:
        print("\nStopping server...")
        server.stop()

if __name__ == "__main__":
    main()