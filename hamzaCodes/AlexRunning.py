import threading
import time
import math
import sys
from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, EV3GyroSensor, Motor, reset_brick, wait_ready_sensors

# Constants
SWEEP_SPEED = 220  # Speed in degrees per second
DELAY = 0.01  # Delay between degrees for smooth and accurate readings
SWEEP_START = 0
SWEEP_END = 180
ANGLE_TOLERANCE = 5  # Tolerance for turning accuracy due to gyro inaccuracies
BLUE_THRESHOLD = 0.8  # Threshold percentage for considering path as blocked
SAFETY_MARGIN = 5  # Degrees to exclude from edges of safe ranges
MIN_NON_BLUE_SEQUENCE = 3  # Minimum length of non-blue sequence to be considered a bridge
MIN_BRIDGE_WIDTH = 10  # Minimum width of a bridge in degrees

class Robot:
    def __init__(self):
        print("Initializing robot...")

        # Sensors
        self.gyro = EV3GyroSensor(3)
        self.front_us = EV3UltrasonicSensor(1)
        self.block_cs = EV3ColorSensor(2)
        self.floor_cs = EV3ColorSensor(4)

        # Initialize color sensors to component mode for RGB readings
        self.block_cs.set_mode("component")
        self.floor_cs.set_mode("component")

        # Motors
        self.left_motor = Motor("C")
        self.right_motor = Motor("B")
        self.door_motor = Motor("A")
        self.arm_motor = Motor("D")

        # PID parameters
        self.KP = 1.0
        self.KI = 0.0
        self.KD = 0.0
        self.MAX_CORRECTION = 60
        self.last_error = 0
        self.integral = 0

        # Robot parameters
        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_COUNTS_PER_ROTATION = 360
        self.AXLE_LENGTH = 10.5  # distance between wheels in cm
        self.target_angle = 0
        self.position = [0, 0]  # [x, y] in cm

        # Heading management
        self.heading = 0        # Cumulative heading in degrees

        # Threading and shared data
        self.stop_event = threading.Event()
        self.shared_data = {
            "angle": 0,
            "distance": 0,
            "is_blue": False,
            "is_yellow": False,
            "block_collection": False,
            "block_approach_angle": -999,
            "sweep_data": []  # List to store sweep data (gyro_angle, is_blue)
        }
        self.lock = threading.Lock()

        wait_ready_sensors()
        print("Robot initialized")

    def is_blue(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        blue_dominance_threshold = 50
        # Adjusted to consider red lines as blue when in blue areas
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold

    def is_yellow(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        return b <= 75 and r >= 200 and g >= 170

    def arm_angle_to_heading_angle(self, arm_angle):
        # Map arm motor angle to heading angle
        # Arm angle 0 corresponds to heading angle 90
        # Arm angle 90 corresponds to heading angle 0
        # Arm angle 180 corresponds to heading angle 270
        heading_angle = (90 - arm_angle) % 360
        return heading_angle

    def sweep(self):
        try:
            self.arm_motor.reset_encoder()
            current_position = SWEEP_START
            direction = 1
            time.sleep(DELAY)

            # Clear previous sweep data
            with self.lock:
                self.shared_data["sweep_data"] = []

            while not self.stop_event.is_set():
                current_position += direction
                if current_position > SWEEP_END or current_position < SWEEP_START:
                    direction *= -1
                    current_position += 2 * direction  # Reverse direction

                self.arm_motor.set_position(current_position)
                time.sleep(DELAY)

                rgb = self.floor_cs.get_rgb()
                is_blue = self.is_blue(rgb) if rgb else False
                is_yellow = self.is_yellow(rgb) if rgb else False

                distance = self.front_us.get_value()

                # Map arm angle to heading angle
                heading_angle = self.arm_angle_to_heading_angle(current_position)

                # Since the gyro is reset after each turn, adjust heading angle
                # to be relative to the robot's cumulative heading
                adjusted_heading = (self.heading + heading_angle) % 360

                # Store sweep data
                with self.lock:
                    self.shared_data["angle"] = adjusted_heading
                    self.shared_data["distance"] = distance
                    self.shared_data["is_blue"] = is_blue
                    self.shared_data["is_yellow"] = is_yellow
                    # Append current data to sweep_data
                    self.shared_data["sweep_data"].append((adjusted_heading, is_blue))

                time.sleep(DELAY)

                # If completed a full sweep, break
                if current_position == SWEEP_START and direction == -1:
                    break  # Exit the sweep to allow main loop processing

        except Exception as e:
            print(f"Sweep error: {e}")

    # Movement Methods
    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def normalize_angle(self, angle):
        """Normalize angle to be between 0 and 359 degrees"""
        return angle % 360

    def angle_difference(self, target, current):
        """Compute the smallest difference between two angles."""
        diff = (target - current + 180) % 360 - 180
        return diff

    def get_gyro_angle(self):
        """Get the normalized gyro angle"""
        angle = self.gyro.get_abs_measure()
        if angle is not None:
            return self.normalize_angle(angle)
        return 0  # If gyro reading is None, return 0

    def get_heading_error(self, target_heading):
        """Get the smallest angle difference between current and target heading"""
        current_angle = self.get_gyro_angle()
        error = self.angle_difference(target_heading, current_angle)
        return error

    def drive_straight(self, speed=-200, duration=0.5):
        start_time = time.time()
        self.gyro.reset_measure()  # Reset gyro before driving straight
        time.sleep(0.1)  # Give some time for gyro to reset

        while time.time() - start_time < duration:
            current_error = self.get_gyro_angle()

            # PID correction
            correction = self.KP * current_error

            # Ensure symmetric motor control
            left_speed = speed - correction
            right_speed = speed + correction

            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)

            time.sleep(0.05)  # Small delay to prevent overwhelming the CPU

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

    def turn(self, target_heading):
        """Turn the robot to an absolute heading (in degrees)."""
        target_heading = self.normalize_angle(target_heading)

        # Calculate the relative angle to turn
        self.gyro.reset_measure()  # Reset gyro before turning
        time.sleep(0.1)  # Allow time for gyro to reset

        while True:
            current_angle = self.get_gyro_angle()
            error = self.angle_difference(target_heading, current_angle)

            if abs(error) < ANGLE_TOLERANCE:
                break

            # Adjust speed based on error for smoother turning
            turn_speed = 100  # Fixed speed for turning

            if error > 0:
                # Need to turn left
                left_speed = -turn_speed
                right_speed = turn_speed
            else:
                # Need to turn right
                left_speed = turn_speed
                right_speed = -turn_speed

            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)

            time.sleep(0.01)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        # Update the cumulative heading
        self.heading = target_heading

    # Improved Avoid Blue Functionality
    def avoid_blue(self):
        print("Blue area detected! Analyzing sweep data to determine best path.")
        # Retrieve sweep data
        with self.lock:
            sweep_data = self.shared_data["sweep_data"][:]
            # Clear sweep data after use
            self.shared_data["sweep_data"] = []

        # Normalize sweep data
        normalized_sweep_data = self.normalize_sweep_data(sweep_data)

        # Calculate percentage of blue detection
        total_angles = len(normalized_sweep_data)
        blue_angles = sum(1 for angle, is_blue in normalized_sweep_data if is_blue)
        blue_percentage = blue_angles / total_angles if total_angles > 0 else 1

        if blue_percentage >= BLUE_THRESHOLD:
            print("Path is blocked by blue area. Turning 180 degrees.")
            new_heading = (self.heading + 180) % 360
            self.turn(new_heading)
            # Perform another sweep after turning
            self.sweep()
            # Process the new sweep data
            self.process_sweep_after_turn()
            return

        # Proceed if path is not blocked
        # Group consecutive non-blue angles into ranges
        safe_ranges = []
        current_range = []
        for angle, is_blue in sorted(normalized_sweep_data):
            if not is_blue:
                current_range.append(angle)
            else:
                if current_range:
                    safe_ranges.append(current_range)
                    current_range = []
        if current_range:
            safe_ranges.append(current_range)

        # Filter safe ranges to only include bridges (wide enough)
        bridge_ranges = [r for r in safe_ranges if len(r) >= MIN_BRIDGE_WIDTH]

        if not bridge_ranges:
            print("No bridge found. Turning 180 degrees.")
            new_heading = (self.heading + 180) % 360
            self.turn(new_heading)
            # Perform another sweep after turning
            self.sweep()
            # Process the new sweep data
            self.process_sweep_after_turn()
            return

        # Exclude safety margins from bridge ranges
        adjusted_safe_ranges = []
        for safe_range in bridge_ranges:
            if len(safe_range) >= 2 * SAFETY_MARGIN:
                adjusted_range = safe_range[SAFETY_MARGIN:-SAFETY_MARGIN]
                adjusted_safe_ranges.append(adjusted_range)
            else:
                # Range too small after applying safety margin
                continue

        if not adjusted_safe_ranges:
            print("No safe angles found after applying safety margin. Turning 180 degrees.")
            new_heading = (self.heading + 180) % 360
            self.turn(new_heading)
            # Perform another sweep after turning
            self.sweep()
            # Process the new sweep data
            self.process_sweep_after_turn()
            return

        # Select the largest safe range
        largest_safe_range = max(adjusted_safe_ranges, key=len)

        # Choose the middle angle of the largest safe range
        middle_index = len(largest_safe_range) // 2
        best_angle = largest_safe_range[middle_index]

        # Set the target heading to the best angle
        target_heading = best_angle

        print(f"Turning to {target_heading} degrees to cross the bridge.")
        self.turn(target_heading)
        # Perform another sweep after turning
        self.sweep()
        # Process the new sweep data
        self.process_sweep_after_turn()
    def normalize_sweep_data(self, sweep_data):
        """
        Normalize the sweep data to:
        - Replace small non-blue sequences within blue areas with blue.
        This prevents the robot from considering small gaps (like red lines) as safe paths.
        """
        normalized_data = []
        is_blue_list = [is_blue for angle, is_blue in sweep_data]

        # Identify small non-blue sequences surrounded by blue
        i = 0
        while i < len(is_blue_list):
            if is_blue_list[i]:
                # Blue detected
                normalized_data.append((sweep_data[i][0], True))
                i += 1
            else:
                # Start of non-blue sequence
                start = i
                while i < len(is_blue_list) and not is_blue_list[i]:
                    i += 1
                end = i
                sequence_length = end - start

                # Check if non-blue sequence is smaller than MIN_NON_BLUE_SEQUENCE
                if sequence_length < MIN_NON_BLUE_SEQUENCE:
                    # Treat the sequence as blue
                    for j in range(start, end):
                        normalized_data.append((sweep_data[j][0], True))
                else:
                    # Keep the non-blue sequence
                    for j in range(start, end):
                        normalized_data.append((sweep_data[j][0], False))
        return normalized_data

    def process_sweep_after_turn(self):
        # Process the new sweep data after turning
        with self.lock:
            new_sweep_data = self.shared_data["sweep_data"][:]
            # Clear sweep data after use
            self.shared_data["sweep_data"] = []

        # Normalize sweep data
        normalized_sweep_data = self.normalize_sweep_data(new_sweep_data)

        # Recalculate blue percentage
        total_angles = len(normalized_sweep_data)
        blue_angles = sum(1 for angle, is_blue in normalized_sweep_data if is_blue)
        blue_percentage = blue_angles / total_angles if total_angles > 0 else 1

        if blue_percentage >= BLUE_THRESHOLD:
            print("Path is still blocked after turning.")
            # Decide whether to turn again or stop
            # For simplicity, we'll attempt to turn another 90 degrees
            new_heading = (self.heading + 90) % 360
            print(f"Attempting to turn to {new_heading} degrees.")
            self.turn(new_heading)
            # Perform another sweep after turning
            self.sweep()
            # Recursively process the new sweep data
            self.process_sweep_after_turn()
        else:
            print("Path is clear after turning. Moving forward.")
            self.drive_straight(speed=-200, duration=1)

    def main(self):
        try:
            while not self.stop_event.is_set():
                # Start the sweep thread
                sweep_thread = threading.Thread(target=self.sweep)
                sweep_thread.start()
                sweep_thread.join()  # Wait for sweep to complete

                # Normalize sweep data
                with self.lock:
                    sweep_data = self.shared_data["sweep_data"][:]
                normalized_sweep_data = self.normalize_sweep_data(sweep_data)

                # Check for blue detection in sweep data
                is_blue_detected = any(is_blue for angle, is_blue in normalized_sweep_data)
                blue_ahead = any(
                    is_blue and abs(self.angle_difference(angle, self.heading)) <= 10
                    for angle, is_blue in normalized_sweep_data
                )

                if is_blue_detected:
                    self.avoid_blue()
                else:
                    # Move forward a little
                    self.drive_straight(speed=-200, duration=0.5)

                # Small delay before next sweep
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Stopping robot...")
            self.stop_event.set()
            self.arm_motor.set_power(0)
            self.left_motor.set_power(0)
            self.right_motor.set_power(0)
            reset_brick()
            print("Brick reset. Exiting program.")
        finally:
            reset_brick()

    def test_turn(self):
        try:
            while not self.stop_event.is_set():
                user_input = input("Enter target heading (or 'exit' to quit): ")
                if user_input.lower() == 'exit':
                    break
                try:
                    target_heading = float(user_input)
                except ValueError:
                    print("Invalid input. Please enter a numerical angle.")
                    continue

                target_heading = self.normalize_angle(target_heading)
                print(f"Turning to {target_heading} degrees.")
                self.turn(target_heading)
                print("Turn complete.\n")

        except KeyboardInterrupt:
            print("Stopping robot...")
            self.stop_event.set()
            self.arm_motor.set_power(0)
            self.left_motor.set_power(0)
            self.right_motor.set_power(0)
            reset_brick()
            print("Brick reset. Exiting program.")
        finally:
            reset_brick()

# Instantiate and run the robot
if __name__ == "__main__":
    robot = Robot()
    # Uncomment one of the following lines to run the desired function
    robot.main()
    #robot.test_turn()
