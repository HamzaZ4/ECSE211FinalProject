#from more import move_forward
from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import utils.brick
import time
import math
import random
import traceback
import threading
from MapModule import *

SWEEP_SPEED = 220  # Speed in degrees per second
DELAY = 0.01  # Delay between degrees for smooth and accurate readings
SWEEP_START = 0
SWEEP_END = 200

class IntegratedRobot:
    def __init__(self):
        print("Initializing robot...")
        self.gyro = EV3GyroSensor(3)
        self.front_us = EV3UltrasonicSensor(1)
        self.block_cs = EV3ColorSensor(2)
        self.floor_cs = EV3ColorSensor(4)

        self.left_motor = Motor("C")
        self.right_motor = Motor("B")
        self.door_motor = Motor("A")
        self.arm_motor = Motor("D")

        self.KP = 1.0
        self.KI = 0.0
        self.KD = 0.0

        # Maximum correction to avoid over-compensation
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
        self.heading = 0  # Current orientation in degrees
        self.shared_data = {
            "rgb": [],
            "distance": 0.0,
            "current_angle": 0
        }
        wait_ready_sensors()
        self.arm_motor.set_limits(70, SWEEP_SPEED)
        print("Robot initialized")

    # Function to reset sensors and motors
    def reset(self):
        self.left_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def is_yellow(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        return b <= 75 and r >= 200 and g >= 170

    def normalize_angle(self, angle):
        """Normalize angle to be between 0 and 359 degrees"""
        return angle % 360

    def is_green(self, rgb):
        r, g, b = rgb
        green_dominance_threshold = 50
        return g > r + green_dominance_threshold and g > b + green_dominance_threshold

    def is_red(self, rgb):
        r, g, b = rgb
        red_dominance_threshold = 50
        return r > g + red_dominance_threshold and r > b + red_dominance_threshold

    def is_blue(self, rgb):
        r, g, b = rgb
        blue_dominance_threshold = 50
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold

    def find_exact_blue(self, blue_intervals):
        real_blue_intervals = []
        if not blue_intervals:
            return []
        interval = blue_intervals[0]
        if not interval:
            return []
        if len(interval) == 1:
            return interval
        cur_start, cur_end = interval[0]
        for i in range(len(interval) - 1):
            next_start, next_end = interval[i + 1]

            if abs(cur_end - next_start) < 20:
                cur_end = next_end
            else:
                real_blue_intervals.append((cur_start, cur_end))
                cur_start = next_start
        real_blue_intervals.append((cur_start, cur_end))
        return real_blue_intervals

    def thread_arm_sensors(self):
        while True:
            self.shared_data["rgb"] = self.floor_cs.get_rgb()
            self.shared_data["distance"] = self.front_us.get_value()
            self.shared_data["current_angle"] = self.gyro.get_abs_measure()
            time.sleep(DELAY)

    def is_yellow(self):
        """Check if the block is yellow"""
        r, g, b = self.block_cs.get_rgb()  # Get RGB values
        if (r == None):
            r = 0
        if (g == None):
            g = 0
        if (b == None):
            b = 0

        print(f"RGB values: R={r} G={g} B={b}")
        # Simple heuristic to detect blueish color (you can adjust thresholds based on testing)
        if b <= 75 and r >= 200 and g >= 170:
            print("Poop detected.")
            return True
        return False

    def is_cube(self):
        r, g, b = self.block_cs.get_rgb()  # Get RGB values
        if (r == None):
            r = 0
        if (g == None):
            g = 0
        if (b == None):
            b = 0
        if r+b+g > 33:
            return True
        return False

    def collect_cube(self):
        self.arm_motor.set_position(0)
        self.drive_straight(-)



        while not self.is_cube():
            print("no cube")
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(-25)
            time.sleep(0.1)
            self.left_motor.set_dps(-25)
            self.right_motor.set_dps(0)

        print("Block detected")
        if self.is_yellow():
            self.door_motor.reset_encoder()
            self.door_motor.set_position(-50)
            print("door opened")
            time.sleep(0.5)
            self.left_motor.set_position(-250)
            self.right_motor.set_position(-250)
            print("poop picked up")
            time.sleep(0.5)
            self.door_motor.set_position(50)
            print("door closed")
            time.sleep(0.5)
            self.left_motor.set_position(250)
            self.right_motor.set_position(250)
            print("back to initial position")

    def sweep(self, start=SWEEP_START, stop=SWEEP_END):
        try:
            self.arm_motor.set_power(50)
            self.arm_motor.set_dps(SWEEP_SPEED)
            self.arm_motor.set_position(SWEEP_START)
            current_position = SWEEP_START
            direction = 2
            time.sleep(DELAY)
            blue_intervals = []
            current_blue_intervals = []
            current_min = None
            current_max = None

            while True:
                current_position += direction
                self.arm_motor.set_position(current_position)

                rgb = self.shared_data["rgb"]
                is_blue = self.is_blue(rgb) if rgb else False
                is_red = self.is_red(rgb) if rgb else False

                if is_blue:
                    if current_min is None:  # Start a new interval
                        current_min = current_position
                    current_max = current_position

                elif is_red:
                    if current_min is not None:
                        current_max = current_position
                elif not is_red:
                    if current_min is not None:
                        # End the current interval
                        current_blue_intervals.append((current_min, current_max))
                        current_min = None
                        current_max = None

                time.sleep(DELAY)

                if current_position > stop or current_position < start:
                    if current_blue_intervals != []:
                        blue_intervals.append(current_blue_intervals)
                        current_blue_intervals = []
                    direction *= -1

                    if current_position < start:
                        break

            return blue_intervals
        except Exception as e:
            print(f"Sweep error: {e}")

    def get_gyro_angle(self):
        """Get the normalized gyro angle"""
        angle = self.gyro.get_abs_measure()
        if angle is not None:
            return self.normalize_angle(angle)
        return self.target_angle

    # Function to calculate distance traveled
    def calculate_distance(self, left_counts, right_counts):
        avg_counts = (abs(left_counts) + abs(right_counts)) / 2
        return (avg_counts / self.ENCODER_COUNTS_PER_ROTATION) * self.WHEEL_CIRCUMFERENCE

    # Function to update position
    def update_position(self, distance, angle):
        """
        Update robot's position based on encoder readings
        """
        # Read motor encoders
        left_counts = self.left_motor.get_encoder()
        right_counts = self.right_motor.get_encoder()

        # Determine movement direction and calculate distance
        signed_distance = self.calculate_distance(left_counts, right_counts) * \
                          (1 if left_counts >= 0 and right_counts >= 0 else -1)

        # Calculate x and y displacements
        current_x = calculate_x(signed_distance, self.heading)
        current_y = calculate_y(signed_distance, self.heading)

        # Update total position
        self.position[0] += current_x
        self.position[1] += current_y

    # Function to drive the robot straight
    def drive_straight(self, speed=-200, duration=0.2):
        start_time = time.time()
        self.target_angle = self.get_gyro_angle()  # Lock current angle

        while time.time() - start_time < duration:
            current_error = self.get_heading_error()

            # More conservative PID correction
            correction = self.KP * current_error

            # Ensure symmetric motor control
            left_speed = speed + correction - 20
            right_speed = speed - correction

            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)
            time.sleep(DELAY)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

    def get_heading_error(self):
        """Get the smallest angle difference between current and target angle"""
        current_angle = self.get_gyro_angle()
        error = current_angle - self.target_angle

        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        return error

    def angle_difference(self, target, current):
        """
        Calculate the minimal signed difference between two angles, considering wrap-around.
        Returns a value between -180 and 180 degrees.
        """
        diff = (target - current + 180) % 360 - 180
        return diff

    def get_gyro_angle(self):
        """Get the normalized gyro angle from the gyro sensor."""
        value = self.gyro.get_value()
        if value is not None:
            if isinstance(value, list) or isinstance(value, tuple):
                angle = value[0]  # Adjust index based on your sensor's data structure
            else:
                angle = value
            # No need to invert the angle since the gyro increases when turning right
            return self.normalize_angle(angle)
        return self.heading  # Return the last known heading if reading fails

    def turn(self, angle):
        angle = angle * -1
        initial_heading = self.get_gyro_angle()
        target_heading = self.normalize_angle(initial_heading + angle)
        print(f"Turning from {initial_heading:.2f}° to {target_heading:.2f}° (angle: {angle:.2f}°)")

        # Determine turn direction based on the sign of the angle
        if angle < 0:  # Turn right
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)
            print("robot turning left")
        else:  # Turn left
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)
            print("robot turning right")

        start_time = time.time()
        MAX_TURN_DURATION = 5  # seconds

        while True:
            current_heading = self.get_gyro_angle()
            heading_error = self.angle_difference(target_heading, current_heading)
            # print(f"Current heading: {current_heading:.2f}°, Target heading: {target_heading:.2f}°, Heading error: {heading_error:.2f}°")

            if abs(heading_error) < 2:
                print("Desired heading reached. Stopping motors.")
                break
            if time.time() - start_time > MAX_TURN_DURATION:
                print("Turn timeout reached. Stopping motors.")
                break

            time.sleep(DELAY)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        self.heading = self.get_gyro_angle()  # Update global heading
        print(f"Turn complete. New heading: {self.heading:.2f}°")

    def avoid_blue(self, blue_intervals):
        """
        Adjusts the robot's movement to navigate through a suitable non-blue gap.
        """

        # Process blue intervals
        exact_blue = self.find_exact_blue(blue_intervals)
        if not exact_blue:
            print("No blue detected after processing. Moving forward.")
            self.drive_straight(speed=-100, duration=1)  # Advance forward
            return

        # Initialize non-blue intervals
        non_blue_intervals = []

        # Starting point
        current_position = SWEEP_START

        for interval in exact_blue:
            start_blue, end_blue = interval

            # Ensure start and end are in order
            if start_blue > end_blue:
                start_blue, end_blue = end_blue, start_blue

            # If there's a gap before this blue interval
            if start_blue > current_position:
                non_blue_intervals.append((current_position, start_blue))

            # Update current position
            current_position = end_blue

        # Check for gap after the last blue interval
        if current_position < SWEEP_END:
            non_blue_intervals.append((current_position, SWEEP_END))

        print(f"Non-blue intervals: {non_blue_intervals}")

        # Define minimum acceptable gap width (in degrees)
        MIN_GAP_WIDTH = 60  # Adjust based on your robot's size and safety margin

        # Define forward direction range
        FORWARD_MIN = 80
        FORWARD_MAX = 100

        # Check if there is a clear path directly ahead (between FORWARD_MIN and FORWARD_MAX degrees)
        max_gap_width = 0
        best_interval = -1
        forward_gap = None
        for interval in non_blue_intervals:
            gap_start, gap_end = interval
            gap_width = gap_end - gap_start

            # Check if the gap is wide enough
            if gap_width >= MIN_GAP_WIDTH:
                # Check if the gap includes the forward direction
                if gap_start <= FORWARD_MIN and gap_end >= FORWARD_MAX:
                    forward_gap = interval
                    break
        sweep_min_gap = 10
        if forward_gap and len(blue_intervals) > 1:
            # Safe to move forward
            print("Clear path directly ahead. Moving forward.")
            self.drive_straight(speed=-100, duration=1)
        else:
            # Find the best alternative gap
            best_gap = None
            selected_turn_angle = None

            for interval in non_blue_intervals:
                gap_start, gap_end = interval
                gap_width = gap_end - gap_start

                if gap_width >= MIN_GAP_WIDTH:

                    print(f" gap end {gap_end} gab start {gap_start}")

                    # Determine turn angle based on gap location relative to 90 degrees
                    if gap_end <= 90:
                        # Gap is entirely to the left; aim for the end of the gap
                        target_angle = gap_end + sweep_min_gap
                    elif gap_start >= 90:
                        # Gap is entirely to the right; aim for the start of the gap
                        target_angle = gap_start - sweep_min_gap
                    else:

                        target_angle = (gap_start + gap_end) / 2
                        # Gap spans over 90 degrees; aim for the point closest to 90 degrees
                        # if (90 - gap_start) < (gap_end - 90):
                        # print("in lower")
                        # target_angle = gap_end  - sweep_min_gap
                        # Closer to the start
                        # else:
                        # print(
                        # target_angle = gap_start  + sweep_min_gap# Closer to the end

                    # Calculate turn angle from current heading (assuming 90° is straight ahead)
                    turn_angle = target_angle - 90

                    # Normalize turn_angle to [-180, 180]
                    if turn_angle > 180:
                        turn_angle -= 360
                    elif turn_angle < -180:
                        turn_angle += 360

                    # Select the gap with the minimal absolute turn angle
                    if selected_turn_angle is None or abs(turn_angle) < abs(selected_turn_angle):
                        selected_turn_angle = turn_angle
                        best_gap = interval

            if best_gap and selected_turn_angle is not None:
                # Limit the turn angle to the maximum allowed
                MAX_TURN_ANGLE = 180  # Allows for drastic turns when necessary
                turn_angle = max(min(selected_turn_angle, MAX_TURN_ANGLE), -MAX_TURN_ANGLE)

                print(f"Turning towards gap at angle {turn_angle:.2f}°.")
                self.turn(turn_angle)
                self.drive_straight(speed=-100, duration=0.5)
            else:
                # No suitable gap found
                print("No suitable gap found. Stopping.")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)



    def turn_away_from_wall(self):
        """
        Turns the robot 90 degrees away from the wall using negative dps for forward motion.
        """
        print("Wall detected! Turning away...")
        # Turn in place: one wheel forward, the other backward
        self.left_motor.set_dps(300)  # Positive dps to turn
        self.right_motor.set_dps(-300)  # Negative dps to turn
        time.sleep(1.0)  # Adjust time for a 90-degree turn
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print("Turned away from the wall.")

    def drive_to_target_with_correction(self, target_angle, distance):
        """
        Drives the robot toward the target angle while continuously correcting its heading.
        """
        print(f"Driving toward target angle: {target_angle:.2f}° with distance: {distance:.2f} cm.")

        dist_per_second = -self.WHEEL_DIAMETER * math.pi * (300 / 360)  # Distance per second at 300 dps
        drive_time = distance / dist_per_second  # Calculate drive duration

        start_time = time.time()
        while time.time() - start_time < drive_time:
            # Check current heading
            current_heading = self.get_gyro_angle()
            heading_error = self.angle_difference(target_angle, current_heading)

            # Apply correction if necessary
            correction = self.KP * heading_error
            left_speed = -200 - correction
            right_speed = -200 + correction

            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)

            time.sleep(DELAY)

        # Stop the motors after driving
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print("Reached target.")


class CubeDetector:
    def __init__(self, robot):
        self.robot = robot
        self.distances = []
        self.angles = []

    def perform_sweep(self, start_angle=0, end_angle=180, step=5):
        """
        Perform a sweep to collect distances and angles using the ultrasonic sensor.
        """
        distances = []
        angles = []
        print("Performing sweep...")
        for angle in range(start_angle, end_angle + 1, step):
            self.robot.arm_motor.set_position(angle)
            time.sleep(0.2)  # Allow motor and sensor to stabilize
            distance = self.robot.front_us.get_value()  # Ensure correct unit (cm)
            distances.append(distance)
            angles.append(angle)
            print(f"{angle}deg: {distance}cm")

        self.distances = distances
        self.angles = angles
        print("Sweep complete.")
        return distances, angles

    def detect_block(self):

        """
        Detect blocks based on the sweep data, prioritizing the closest block.
        """
        detection_radius = 10  # cm
        blocks = []
        current_block = []

        for i in range(len(self.distances)):
            if self.distances[i] <= detection_radius:
                current_block.append((self.angles[i], self.distances[i]))
            else:
                if current_block:
                    blocks.append(current_block)
                    current_block = []

        if current_block:  # Add the last block
            blocks.append(current_block)
        if blocks:
            print(f"Detected blocks: {blocks}")
            # Find the closest block
            closest_block = min(blocks, key=lambda block: min(pair[1] for pair in block))
            center_angle = (closest_block[0][0] + closest_block[-1][0]) // 2
            min_distance = min(pair[1] for pair in closest_block)
            print(f"Target block center at {center_angle}° and {min_distance} cm.")
            return center_angle, min_distance
        else:
            print("No blocks detected.")
            return None, None

    def move_towards_cube(self, center_angle, distance):
        """
        Moves the robot toward the cube located at the specified center angle.
        """
        if center_angle is None or distance is None:
            print("No cube detected. Cannot move towards it.")
            return

        print(f"Targeting cube at center angle: {center_angle:.2f}° with distance: {distance:.2f} cm.")

        # Calculate the turn angle needed to align with the center angle
        turn_angle = (center_angle - 90 + 360) % 360  # Normalize to [0, 360)
        if turn_angle > 180:
            turn_angle -= 360  # Normalize to [-180, 180]

        print(f"Calculated turn angle: {turn_angle:.2f}°.")

        # Turn the robot toward the cube
        self.robot.turn(turn_angle)



        # Step 2: Calculate movement time based on distance
        dist_per_second = self.robot.WHEEL_DIAMETER * math.pi * (300 / 360)  # Distance per second at 300 dps
        move_time = distance / dist_per_second  # Calculate time required to cover the distance

        print(f"Driving straight for {move_time:.2f} seconds toward the cube.")

        # Step 3: Drive straight toward the cube with optional heading correction
        self.robot.drive_to_target_with_correction(center_angle, distance)
        print("Reached the cube.")

    def go_towards_cube(self):
        """
        Moves the robot straight towards the cube while keeping it in front.
        Stops if the robot is at least 4 cm away from the cube.
        """
        print("Moving towards cube...")

        while True:
            distance = self.robot.front_us.get_value()
            if distance and distance <= 4:  # Stop if the robot is within 4 cm of the cube
                print(f"Cube reached. Final distance: {distance} cm.")
                self.robot.left_motor.set_dps(0)
                self.robot.right_motor.set_dps(0)
                break

            if distance is not None:
                print(f"Current distance: {distance} cm. Moving forward.")
                self.robot.drive_straight(speed=-200, duration=0.1)
            else:
                print("No valid distance reading. Stopping motors to avoid collision.")
                self.robot.left_motor.set_dps(0)
                self.robot.right_motor.set_dps(0)
                break

            time.sleep(0.1)  # Allow for smooth updates

    def turn_to_cube(self, center_angle):
        """
        Turns the robot towards the detected cube, starting about 10 degrees before the given angle.
        The turn is limited to ±30 degrees for controlled movement.
        """
        MAX_TURN_ANGLE = 30  # Limit the turn to ±30 degrees
        PRE_TURN_OFFSET = 10  # Start turning 10 degrees before the center angle
        print(f"Preparing to turn towards cube at approximate angle {center_angle}°...")

        # Step 1: Rotate the arm to face forward (90°)
        self.robot.arm_motor.set_position(90)
        time.sleep(0.5)  # Allow time for the arm to stabilize

        # Step 2: Calculate the adjusted turn angle
        adjusted_angle = center_angle - PRE_TURN_OFFSET
        turn_angle = adjusted_angle - 90

        # Limit the turn angle to ±30 degrees
        if turn_angle > MAX_TURN_ANGLE:
            turn_angle = MAX_TURN_ANGLE
        elif turn_angle < -MAX_TURN_ANGLE:
            turn_angle = -MAX_TURN_ANGLE

        print(f"Calculated turn angle (with offset): {turn_angle}°.")

        # Step 3: Turn the robot toward the adjusted angle
        self.robot.turn(turn_angle)
        time.sleep(0.5)  # Allow time for the turn to complete

        # Step 4: Fine-tune alignment by analyzing distances
        print("Fine-tuning alignment with cube...")
        fine_tune_step = 2  # Smaller steps for fine-tuning
        while True:
            distance = self.robot.front_us.get_value()
            if distance and distance < 30:  # Detect the cube based on distance drop
                print(f"Cube aligned. Distance: {distance} cm.")
                break

            # Make fine adjustments to center on the cube
            self.robot.turn(fine_tune_step)  # Adjust turning step for fine-tuning
            time.sleep(0.2)  # Increase delay for better sensor readings

        print("Alignment with cube complete.")


if __name__ == "__main__":
    try:
        # Initialize the robot and detector
        robot = IntegratedRobot()
        detector = CubeDetector(robot)

        print("Starting cube detection and navigation...")

        # Step 1: Reset the robot's sensors and motors
        robot.reset()

        # Step 2: Perform a sweep to populate distance and angle arrays
        print("Performing a sweep to detect potential blocks...")
        distances, angles = detector.perform_sweep(start_angle=0, end_angle=180, step=5)

        # Step 3: Detect the closest block
        center_angle, distance = detector.detect_block()

        if center_angle is not None and distance is not None:
            print(f"Cube detected at angle {center_angle}° with distance {distance} cm.")

            # Step 4: Turn to align with the cube using the updated turn_to_cube method
            detector.turn_to_cube(center_angle)

            # Step 5: Move toward the cube
            detector.go_towards_cube()

            # Step 6: Recheck surroundings after reaching the cube
            print("Rechecking surroundings after reaching the cube...")
            distances, angles = detector.perform_sweep(start_angle=0, end_angle=180, step=5)
            center_angle, distance = detector.detect_block()

            if center_angle is not None and distance is not None:
                print(f"Additional cube detected at angle {center_angle}° with distance {distance} cm.")
                detector.turn_to_cube(center_angle)
                detector.go_towards_cube()
                robot.collect_cube()
            else:
                print("No additional cubes detected. Task complete.")
        else:
            print("No cube detected. Exiting...")

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")
        traceback.print_exc()
    finally:
        print("Exiting program.")
        reset_brick()
        exit()
