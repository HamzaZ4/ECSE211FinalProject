from utils.brick import TouchSensor,EV3UltrasonicSensor, EV3ColorSensor, Motor, configure_ports, wait_ready_sensors
import time


class MapNavigator:
    def __init__(self):
        
        # Initialize sensors and motors
        
        self.touch_sensor = TouchSensor(1)
        self.color_sensor = EV3ColorSensor(2)
        self.ultrasonic_sensor = EV3UltrasonicSensor(3)  # Assuming it's connected to port 3
        self.left_motor = Motor("C")
        self.right_motor = Motor("B")
        

        wait_ready_sensors()
        self.color_sensor.set_mode("RGB")  # Set the color sensor mode to RGB to detect blueish color

        # Motor control constants
        self.BASE_SPEED = -200  # Base speed for both motors
        self.THRESHOLD_BLUE = 40  # Threshold for detecting blueish color
        self.OBSTACLE_DISTANCE = 10  # Minimum distance (in cm) from the wall to avoid collision
        self.SAFE_DISTANCE = 10  # Safe distance to stop when no wall is detected
        
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

    def avoid_wall(self):
        """Avoid walls using the ultrasonic sensor."""
        distance = self.ultrasonic_sensor.get_raw_value()
        print(f"Ultrasonic sensor distance: {distance} cm")
        if distance < self.OBSTACLE_DISTANCE:
            print("Obstacle detected! Turning...")
            # Stop and turn to avoid the wall
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(0)
            time.sleep(0.5)
            # Turn right to avoid the obstacle
            self.left_motor.set_dps(self.BASE_SPEED)
            self.right_motor.set_dps(-self.BASE_SPEED)
            time.sleep(1)
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(0)

    def is_blue_floor(self):
        """Check if the floor is blueish."""
        r, g, b = self.color_sensor.get_rgb()  # Get RGB values
        print(f"RGB values: R={r} G={g} B={b}")
        # Simple heuristic to detect blueish color (you can adjust thresholds based on testing)
        if b > self.THRESHOLD_BLUE and r < self.THRESHOLD_BLUE and g < self.THRESHOLD_BLUE:
            print("Blue floor detected! Stopping.")
            
            return True
        return False

    def move(self):
        """Move the robot while avoiding walls and blue areas."""
        print("Press touch sensor to start movement")
        while True:
            self.check_button()
            if self.is_running:
                try:
                    while self.is_running:
                        # Avoid obstacles using ultrasonic sensor
                        #self.check_button()
                        self.avoid_wall()
                        

                        if self.is_blue_floor():
                            self.left_motor.set_dps(0);
                            self.right_motor.set_dps(0);
                            time.sleep(1)
                            self.left_motor.set_dps(self.BASE_SPEED)
                            self.right_motor.set_dps(-self.BASE_SPEED)
                            time.sleep(0.5)

                        # Continue moving forward
                        self.left_motor.set_dps(self.BASE_SPEED)
                        self.right_motor.set_dps(self.BASE_SPEED)

                        time.sleep(0.1)

                    self.left_motor.set_dps(0)
                    self.right_motor.set_dps(0)
                    print("Movement stopped")

                except KeyboardInterrupt:
                    self.left_motor.set_dps(0)
                    self.right_motor.set_dps(0)
                    print("\nProgram terminated")
                    break

    def main(self):
        self.move()


if __name__ == "__main__":
    navigator = MapNavigator()
    navigator.main()
