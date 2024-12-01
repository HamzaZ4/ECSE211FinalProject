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
SWEEP_END = 180
SWEEP_UNIT = 2
MIN_GAP_WIDTH = 60


def calculate_x(distance, heading):
    """
    Calculate x displacement considering direction of movement
    """
    return distance * math.cos(math.radians(heading))

def calculate_y(distance, heading):
    """
    Calculate y displacement considering direction of movement
    """
    return distance * math.sin(math.radians(heading))

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
        self.heading = 0        # Current orientation in degrees
        self.shared_data = {
            "rgb": [],
            "distance": 0.0,
            "current_angle": 0,
            "block_rgb": []
        }
        self.distances = []
        self.angles = []
        self.map = Map()
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
        blue_dominance_threshold = 20
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold

    def find_exact_blue(self, blue_intervals):
        real_blue_intervals = []
        if not blue_intervals:
            return []
        interval = blue_intervals
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
                cur_end = next_end
        real_blue_intervals.append((cur_start, cur_end))
        return real_blue_intervals

    def thread_arm_sensors(self):
        while True:
            self.shared_data["rgb"] = self.floor_cs.get_rgb()
            self.shared_data["distance"] = self.front_us.get_value()
            self.shared_data["current_angle"] = self.gyro.get_abs_measure()
            time.sleep(DELAY)

    def blue_sweep(self, start=SWEEP_START, stop=SWEEP_END):
        try:
            self.arm_motor.set_power(50)
            self.arm_motor.set_dps(SWEEP_SPEED)
            self.arm_motor.set_position(SWEEP_START)
            current_position = SWEEP_START
            direction = SWEEP_UNIT
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
                    if current_blue_intervals != [] and direction == SWEEP_UNIT:
                        print(f"current {current_blue_intervals}")
                        blue_intervals = current_blue_intervals
                    current_blue_intervals = []
                    direction *= -1

                    if current_position < start:
                        break
            print(f"blue intervals: {blue_intervals}")
            return blue_intervals
        except Exception as e:
            print(f"Sweep error: {e}")

    def sweep(self, start=SWEEP_START, end=SWEEP_END, direction=1):

        self.distances = []
        self.angles = []
        self.arm_motor.set_position(start)
        print(f" {start}, {end}")
        sweep_direction = SWEEP_UNIT * direction
        water_detected = False
        block_position = -999
        if direction > 0:

            for i in range(start, end + sweep_direction, sweep_direction):
                self.arm_motor.set_position(i)
                rgb = self.shared_data["rgb"]
                self.distances.append(self.shared_data["distance"])
                self.angles.append(i)
                is_blue = self.is_blue(rgb) if rgb else False

                if is_blue:
                    return True, False, block_position


                if self.is_cube():
                    time.sleep(0.1)
                    if self.is_yellow_orange():

                        block_position = i
                        return False, True, block_position

                    else:
                        return False, False, block_position
                time.sleep(0.03)


        else:
            for i in range(start, end + sweep_direction, sweep_direction):
                self.arm_motor.set_position(i)
                rgb = self.shared_data["rgb"]
                self.distances.append(self.shared_data["distance"])
                self.angles.append(i)
                is_blue = self.is_blue(rgb) if rgb else False

                if is_blue:
                    return True, False, block_position

                if self.is_cube():
                    time.sleep(0.1)
                    block_position = i
                    if self.is_yellow_orange():

                        return False, True, block_position

                    else:
                        return False, False, block_position
                time.sleep(0.03)

        return water_detected, False, block_position


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
        signed_distance = round(self.calculate_distance(left_counts, right_counts), 2)

        # Calculate x and y displacements
        current_x = round(calculate_x(signed_distance, self.heading),2)
        current_y = round(calculate_y(signed_distance, self.heading),2)

        # Update total position
        self.position[0] += current_x
        self.position[1] += current_y

        self.map.update_map(self.position[0],self.position[1])

    # Function to drive the robot straight
    def drive_straight(self, speed=-200, duration=0.2):

#         fwd, possible_heading = self.map.ston(self, 0, (duration * 5.34))
#         print(f"Forward Clear {fwd}, Best direction {possible_heading}")

        start_time = time.time()
        self.target_angle = self.get_gyro_angle()  # Lock current angle

        while time.time() - start_time < duration:
            current_error = self.get_heading_error()

            # More conservative PID correction
            correction = self.KP * current_error

            # Ensure symmetric motor control
            left_speed = speed + correction - 5
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
        print(f"Current heading {self.heading}")
        if angle > 0 :
            print(f"Turn left {angle} degrees")
        else:
            print(f"Turn right {angle * 1} degress")

        angle = angle * -1
        initial_heading = self.get_gyro_angle()
        target_heading = self.normalize_angle(initial_heading + angle)
        print(f"Non normalized target angle :{initial_heading + angle}")
        print(f"Turning from {initial_heading:.2f}° to {target_heading:.2f}° (angle: {angle:.2f}°)")

        # Determine turn direction based on the sign of the angle
        if angle < 0:  # Turn left
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)
            print("robot turning left")
        else:  # Turn right
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)
            print("robot turning right")

        start_time = time.time()
        MAX_TURN_DURATION = 5  # seconds

        while True:
            current_heading = self.get_gyro_angle()
            print(f"gyro angle:{current_heading}")
            heading_error = self.angle_difference(target_heading, current_heading)
            #print(f"Current heading: {current_heading:.2f}°, Target heading: {target_heading:.2f}°, Heading error: {heading_error:.2f}°")

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
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

    def detect_block(self):
        print("Analyzing block detection data...")

        detection_radius = 20  # Detection threshold (cm)
        block_distance_threshold = 20  # Block identification distance

        candidates = []
        current_interval = []

        for i in range(len(self.distances)):
            print(f"Distance: {self.distances[i]:.2f}, Angle: {self.angles[i]:.2f}")
            if self.distances[i] <= detection_radius and self.angles[i]<130 and self.angles[i]>45:
                current_interval.append((self.angles[i], self.distances[i]))
            else:
                if current_interval:
                    candidates.append(current_interval)
                    current_interval = []

        if current_interval:
            candidates.append(current_interval)

        for interval in candidates:
            angles_in_interval = [pair[0] for pair in interval]
            distances_in_interval = [pair[1] for pair in interval]
            angular_span = angles_in_interval[-1] - angles_in_interval[0]

            if angular_span <= 25 and all(d < block_distance_threshold for d in distances_in_interval):
                center_angle = (angles_in_interval[0] + angles_in_interval[-1]) / 2
                min_distance = min(distances_in_interval)
                print(f"Block detected at center angle {center_angle:.2f}, distance {min_distance:.2f}.")
                return center_angle, min_distance

        print("No block detected.")
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
        self.turn(turn_angle)

        # Step 2: Calculate movement time based on distance
        dist_per_second = self.WHEEL_DIAMETER * math.pi * (300 / 360)  # Distance per second at 300 dps
        move_time = distance / dist_per_second  # Calculate time required to cover the distance

        print(f"Driving straight for {move_time:.2f} seconds toward the cube.")

        # Step 3: Drive straight toward the cube with optional heading correction
        self.drive_to_target_with_correction(center_angle, distance)
        print("Reached the cube.")

    def go_towards_cube(self, stop_distance=4):
        print("Approaching cube...")
        self.arm_motor.set_position(90)  # Arm faces forward
        time.sleep(0.5)

        while True:
            distance = self.front_us.get_value()

            if distance is not None and distance <= stop_distance:
                print(f"Reached cube at safe distance {distance:.2f} cm.")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)
                break

            if distance is not None:
                print(f"Current distance: {distance:.2f} cm. Moving forward.")
                self.drive_straight(speed=-150, duration=0.1)
            else:
                print("No valid distance. Stopping.")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)
                break

            time.sleep(0.1)

    def turn_to_cube(self, center_angle):
        """
        Aligns the robot with the cube by turning to 30 degrees before the center angle.
        Dynamically adjusts until it detects the cube again and checks up to 30 degrees on both sides of the detected center.
        """
        PRE_TURN_OFFSET = 20  # Degrees to start turning before the detected angle
        MAX_TURN_OFFSET = 20  # Maximum turn beyond the center angle on either side
        ALIGNMENT_THRESHOLD = 9.9  # Distance threshold (cm) for ultrasonic sensor

        print(f"Preparing to align with cube at center angle {center_angle:.2f}°...")

        # Step 1: Rotate the arm to face forward (90°)
        self.arm_motor.set_position(90)
        time.sleep(0.5)  # Allow arm to stabilize

        # Step 2: Turn to start position (30 degrees before center angle)
        start_angle = center_angle - PRE_TURN_OFFSET
        self.turn(start_angle - 90)
        print(f"Turned to starting angle: {start_angle:.2f}°.")

        # Step 3: Sweep until cube is detected or maximum bounds are reached
        current_angle = start_angle
        while True:
            distance = self.shared_data["distance"]

            if distance is not None and distance <= ALIGNMENT_THRESHOLD or self.is_cube():
                print(f"Cube detected at {distance:.2f} cm. Alignment complete.")
                break

            # Adjust angle to continue sweeping
            current_angle += 2  # Fine-tuning in 2-degree steps
            self.turn(2)
            time.sleep(0.1)

            # Stop if maximum bounds are exceeded
            if current_angle > center_angle + MAX_TURN_OFFSET:
                print(f"Exceeded maximum bounds of {center_angle + MAX_TURN_OFFSET:.2f}°. Reversing direction.")
                break

        # Step 4: Sweep back to the other side if no cube is detected
        if distance is None or distance > ALIGNMENT_THRESHOLD:
            print("No cube detected. Sweeping back in the opposite direction.")
            while True:
                distance = self.front_us.get_value()

                if distance is not None and distance <= ALIGNMENT_THRESHOLD:
                    print(f"Cube detected at {distance:.2f} cm during reverse sweep. Alignment complete.")
                    break

                # Adjust angle to sweep back
                current_angle -= 2  # Fine-tuning in 2-degree steps
                self.turn(-2)
                time.sleep(0.1)

                # Stop if the lower bounds are exceeded
                if current_angle < start_angle:
                    print("Exceeded lower bounds during reverse sweep. Stopping alignment.")
                    break

        print("Cube aligned or maximum bounds reached. Ready to approach.")

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

    def is_yellow_orange(self):
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
        if r >= 190:
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
        if r + b + g > 120:
            return True
        return False


    def collect_cube(self):
        time.sleep(0.5)
        self.arm_motor.set_position(0)
        self.door_motor.set_limits(30)
        dist_per_second = self.WHEEL_DIAMETER * math.pi * (300 / 360)  # Distance per second at 300 dps
        drive_time = 8 / dist_per_second  # Calculate drive duration
        #self.drive_straight(-300, drive_time)
        a = 0
        is_poop = False

        if self.is_yellow_orange():
            self.arm_motor.set_position(0)
            self.drive_straight(300,3)
            print("poop picked up")
            time.sleep(0.5)
            self.door_motor.set_position(50)
            print("door closed")
            time.sleep(0.5)
            self.poop_counter = self.poop_counter + 1
            self.drive_straight(-300, drive_time)
            print("back to initial position")
        else:
            drive_time = drive_time = 10 / dist_per_second
            self.drive_straight(300, drive_time)
            self.turn(random.choice([-90, 90]))


    def cube_detection_process(self):
        # Ensure `distances` and `angles` are populated before detection
        center_angle, distance = self.detect_block()

        if center_angle is not None and distance is not None:
            print(f"Cube detected at angle {center_angle:.2f}° with distance {distance:.2f} cm.")

            # Align with and approach the cube
            self.turn_to_cube(center_angle)
            self.go_towards_cube(stop_distance=4)

            # Collect the cube
            print("Collecting the cube...")
            self.collect_cube()

            print("Cube collection complete. Task complete.")
            return True
        else:
            print("No cube detected. Exiting...")
            return False


    def come_back_to_zero_degrees(self):

        start = self.arm_motor.get_position()
        for i in range(start, SWEEP_START, -2):
            self.arm_motor.set_position(i)
            time.sleep(DELAY)




    def avoid_blue(self, blue_intervals):
        """
        Adjusts the robot's movement to navigate through a suitable non-blue gap.
        """

        # Process blue intervals
        # Find the best alternative gap
        best_gap = 0
        selected_interval = None
        selected_turn_angle = 0
        exact_blue = self.find_exact_blue(blue_intervals)
        print(f"exact blue {exact_blue}")
        if not exact_blue:
            print("No blue detected after processing. Moving forward.")
            self.left_motor.reset_encoder()
            self.right_motor.reset_encoder()
            self.drive_straight(speed=-100, duration=1)  # Advance forward
            self.update_position(0,self.heading)
            return

        # Initialize non-blue intervals
        non_blue_intervals = []

        # Starting point
        current_position = SWEEP_START

        for interval in exact_blue:
            start_blue, end_blue = interval

            non_blue_start = current_position
            non_blue_end = start_blue
            current_position = end_blue


            # Ensure start and end are in order
            #if start_blue > end_blue:
                #start_blue, end_blue = end_blue, start_blue

            # If there's a gap before this blue interval
            #if start_blue > current_position:
            non_blue_intervals.append((non_blue_start, non_blue_end))

            # Update current position
            #current_position = end_blue

        # Check for gap after the last blue interval
        if current_position < SWEEP_END:
            non_blue_intervals.append((current_position, SWEEP_END))

        print(f"Non-blue intervals: {non_blue_intervals}")

        # Define minimum acceptable gap width (in degrees)
        MIN_GAP_WIDTH = 40  # Adjust based on your robot's size and safety margin

        # Define forward direction range
        FORWARD_MIN = 50
        FORWARD_MAX = 130


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
                if gap_start < FORWARD_MIN and gap_end > FORWARD_MAX:
                    forward_gap = interval
                    break
        sweep_min_gap = 15
        if forward_gap and len(blue_intervals) > 1:
            # Safe to move forward
            print("Clear path directly ahead. Moving forward.")
            self.left_motor.reset_encoder()
            self.right_motor.reset_encoder()
            self.drive_straight(speed=-200, duration=1)
            self.update_position(0,self.heading)


        else:

            #find biggest gap
            for interval in non_blue_intervals:
                gap_start, gap_end = interval
                gap_width = abs(gap_end - gap_start)

                if gap_width >= MIN_GAP_WIDTH:
                    if gap_width > best_gap:
                        best_gap = gap_width
                        selected_interval = interval

                    #print(f" gap end {gap_end} gab start {gap_start}")

            # Determine turn angle based on gap location relative to 90 degrees
            print(f"selected interval {selected_interval}")
            if selected_interval is not None:
                gap_start, gap_end = selected_interval
                if gap_end <= 90:
                # Gap is entirely to the left; aim for the end of the gap

                    target_angle = gap_end - sweep_min_gap
                elif gap_start >= 90:
                # Gap is entirely to the right; aim for the start of the gap
                    target_angle = gap_start + sweep_min_gap

                else:

                    target_angle = (gap_start + gap_end) / 2
                    # Gap spans over 90 degrees; aim for the point closest to 90 degrees
                    #if (90 - gap_start) < (gap_end - 90):
                        #print("in lower")
                        #target_angle = gap_end  - sweep_min_gap
                        # Closer to the start
                    #else:
                        #print(
                        #target_angle = gap_start  + sweep_min_gap# Closer to the end

                    # Calculate turn angle from current heading (assuming 90° is straight ahead)
                turn_angle = target_angle - 90

                # Normalize turn_angle to [-180, 180]
                if turn_angle > 180:
                    turn_angle -= 360
                elif turn_angle < -180:
                    turn_angle += 360

                selected_turn_angle = turn_angle


            if best_gap and selected_turn_angle is not None:
                # Limit the turn angle to the maximum allowed
                MAX_TURN_ANGLE = 180  # Allows for drastic turns when necessary
                turn_angle = max(min(selected_turn_angle, MAX_TURN_ANGLE), -MAX_TURN_ANGLE)

                print(f"Turning towards gap at angle {turn_angle:.2f}°.")


                self.turn(turn_angle)
                self.drive_straight(speed=-100, duration=0.5)

                self.update_position(0, self.heading)

            else:
                # No suitable gap found
                print("No suitable gap found. Stopping.")
                self.left_motor.reset_encoder()
                self.right_motor.reset_encoder()

                self.drive_straight(speed=100, duration=0.5)
                self.update_position(0, self.heading)
                self.turn(90)



def main():
    try:

        robot = IntegratedRobot()
        input("Press Enter to start")
        robot.reset()
        direction = 1
        robot.arm_motor.set_power(50)
        robot.arm_motor.set_dps(SWEEP_SPEED)

        # Start thread for sensor monitoring
        sweep_thread = threading.Thread(target=robot.thread_arm_sensors)
        sweep_thread.start()

        while True:
            # Perform a sweep to detect blue areas
            # turns left
            # does this turn right or left?
            if direction == 1:
                print("sweeping fowards")
                is_blue, is_poop, block_position = robot.sweep(start=SWEEP_START, end=SWEEP_END, direction=direction)

            else:
                print("sweeping backwards")
                is_blue, is_poop, block_position = robot.sweep(start=SWEEP_END, end=SWEEP_START, direction=direction)

            if is_blue:
                print(f" sweep start {SWEEP_START} sweep end {SWEEP_END}")
                robot.come_back_to_zero_degrees()
                blue_intervals = robot.blue_sweep(SWEEP_START, SWEEP_END)
                direction = -1

                # Avoid the detected blue areas using the class method
                if blue_intervals:
                    print(f"Blue intervals detected: {blue_intervals}")
                    robot.avoid_blue(blue_intervals)

                else:
                    robot.left_motor.reset_encoder()
                    robot.right_motor.reset_encoder()

                    robot.drive_straight(speed=-200, duration=0.5)
                    robot.update_position(0, robot.heading)
                    print(f"Position: {robot.position}")

            elif block_position != -999:
                if is_poop:
                    robot.turn_to_cube(block_position)
                    robot.collect_cube()
                else:
                    if block_position > 90:
                        robot.turn(block_position - 15 - 90)
                    else:
                        robot.turn(block_position + 15 - 90)

            else:
                # center_angle, distance = robot.detect_block()
                # center_angle, distance = None

                # if center_angle is not None and distance is not None:
                # print(f"Cube detected at angle {center_angle}° with distance {distance} cm.")

                # Step 4: Turn to align with the cube using the updated turn_to_cube method
                # robot.turn_to_cube(center_angle)

                # Step 5: Move toward the cube
                # robot.go_towards_cube()

                # Step 6: Recheck surroundings after reaching the cube
                # print("Rechecking surroundings after reaching the cube...")
                # distances, angles = robot.sweep(start_angle=0, end_angle=180, step=5)
                # center_angle, distance = detector.detect_block()


                if robot.cube_detection_process() :
                    continue
                print("No cube detected. Exiting...")
                robot.left_motor.reset_encoder()
                robot.right_motor.reset_encoder()

                robot.drive_straight(speed=-200, duration=0.5)
                robot.update_position(0, robot.heading)
                print(f"Position: {robot.position}")

            # Short delay to prevent overloading the CPU
            direction *= -1
            time.sleep(DELAY)

    except BaseException as e:
        print(traceback.format_exc())
        print(e)
    finally:
        print(robot.position[0], robot.position[1])
        robot.map.printMap()
        print("Exiting program")
        reset_brick()
        exit()


if __name__ == "__main__":
    main()



if __name__ == "__main__":
    main()
