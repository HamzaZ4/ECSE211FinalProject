from utils.brick import TouchSensor, EV3ColorSensor, Motor, wait_ready_sensors
import time

class LineFollower:
    def __init__(self):
        # Initialize sensors and motors
        self.touch_sensor = TouchSensor(1)
        self.color_sensor = EV3ColorSensor(2)
        self.left_motor = Motor("A")
        self.right_motor = Motor("B")
        self.c
        
        wait_ready_sensors()
        self.color_sensor.set_mode("RED")
        
        # Motor control constants
        self.BASE_SPEED = 200  # Base speed for both motors
        self.TARGET_RED = 15   # Target red value
        self.THRESHOLD = 2     # Threshold for line detection
        self.Kp = 5            # Proportional gain for line following
        
        self.is_running = False
    
    def check_button(self):
        if self.touch_sensor.is_pressed():
            self.is_running = not self.is_running
            if self.is_running:
                print("Starting movement")
            else:
                print("Stopping movement")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)
            time.sleep(1)  # Debounce
            return True
        return False

    def rotate_until_red(self):
        """Rotate right until the red line is detected again."""
        print("Lost line, rotating right to find it...")
        self.left_motor.set_dps(self.BASE_SPEED)
        self.right_motor.set_dps(-self.BASE_SPEED)
        while self.is_running:
            if self.check_button():
                return False
            red_value = self.color_sensor.get_red()
            if red_value is None:
                continue
            # Debug print
            print(f"Searching - Red value: {red_value}")
            if abs(red_value - self.TARGET_RED) < self.THRESHOLD:
                print("Red line found!")
                break
            time.sleep(0.01)
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        return True

    def follow_line(self):
        """Follow the red line using proportional control."""
        print("Press touch sensor to start line following")
        while True:
            self.check_button()
            if self.is_running:
                try:
                    while self.is_running:
                        if self.check_button():
                            break
                        red_value = self.color_sensor.get_red()
                        if red_value is None:
                            continue
                        
                        # Print red value for debugging
                        print(f"Red value: {red_value}")
                        
                        error = self.TARGET_RED - red_value
                        
                        # Apply proportional control to adjust speeds
                        correction = self.Kp * error
                        left_speed = self.BASE_SPEED + correction
                        right_speed = self.BASE_SPEED - correction
                        
                        # Clamp motor speeds to valid range
                        max_speed = 300  # Maximum speed for your motors
                        left_speed = max(min(left_speed, max_speed), -max_speed)
                        right_speed = max(min(right_speed, max_speed), -max_speed)
                        
                        # Set motor speeds
                        self.left_motor.set_dps(-left_speed)
                        self.right_motor.set_dps(-right_speed)
                        
                        # If no red is detected, rotate until found
                        if red_value < 1:
                            self.left_motor.set_dps(0)
                            self.right_motor.set_dps(0)
                            if not self.rotate_until_red():
                                break
                        
                        time.sleep(0.01)
                    
                    self.left_motor.set_dps(0)
                    self.right_motor.set_dps(0)
                    print("Line following stopped")
                    
                except KeyboardInterrupt:
                    self.left_motor.set_dps(0)
                    self.right_motor.set_dps(0)
                    print("\nProgram terminated")
                    break

    def main(self):
        self.follow_line()

if __name__ == "__main__":
    follower = LineFollower()
    follower.main()
