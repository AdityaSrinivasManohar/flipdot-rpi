import rclpy
from rclpy.node import Node

from flipdot.src.states.clock_state import ClockState
from flipdot.src.states.text_scroll_state import TextScrollState
from flipdot.src.states.weather_state import WeatherState
from flipdot.src.utils import FlipDotFrame, FlipDotRosConverter


class FlipDotFSM(Node):
    def __init__(self):
        super().__init__('flipdot_fsm')
        self.publisher = self.create_publisher(FlipDotFrame, '/flipdot/command', 10)
        
        demo = "Welcome to the demo, firstly you can check out the clock"
        demo_next = "Next the current weather"

        # Initialize States
        self.states = {
            "CLOCK": ClockState(28, 14, self.get_logger()),
            "WEATHER": WeatherState(28, 14, self.get_logger()),
            "TEXT_SCROLL_WELCOME": TextScrollState(23, 14, demo, self.get_logger()),
            "TEXT_SCROLL_NEXT": TextScrollState(23, 14, demo_next, self.get_logger()),
            "TEXT_SCROLL_THANK_YOU": TextScrollState(23, 14, "Thank you!", self.get_logger()),
        }
        
        self.current_state_key = "TEXT_SCROLL_WELCOME"
        self.state = self.states[self.current_state_key]
        self.state_start_time = self.get_clock().now()
        
        self.create_timer(0.1, self.run_fsm)
        self.frame_count = 0

    def transition_to(self, next_key):
        self.get_logger().info(f"Transitioning to {next_key}")
        self.current_state_key = next_key
        self.state = self.states[next_key]
        self.state_start_time = self.get_clock().now()
        self.state.on_enter()

    def run_fsm(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds / 1e9

        if self.current_state_key == "TEXT_SCROLL_WELCOME" and elapsed > 21.0:
            self.transition_to("CLOCK")
        elif self.current_state_key == "CLOCK" and elapsed > 10.0:
            self.transition_to("TEXT_SCROLL_NEXT")
        elif self.current_state_key == "TEXT_SCROLL_NEXT" and elapsed > 15.0:
            self.transition_to("WEATHER")
        elif self.current_state_key == "WEATHER" and elapsed > 10.0:
            self.transition_to("TEXT_SCROLL_THANK_YOU")

        # # --- FSM TRANSITION LOGIC ---
        # At some point we change this to some key press or time based logic
        # if self.current_state_key == "CLOCK" and elapsed > 15.0:
        #     self.transition_to("TEST")
        # elif self.current_state_key == "TEST" and elapsed > 10.0:
        #     self.transition_to("CLOCK")

        # --- GET AND PUBLISH FRAME ---
        grid = self.state.get_frame(self.frame_count, elapsed)
        msg = FlipDotRosConverter.to_msg(grid)
        self.publisher.publish(msg)
        self.frame_count += 1

def main(args=None):
    print("Initializing")
    rclpy.init(args=args)
    node = FlipDotFSM()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()