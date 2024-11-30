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
        self.heading = 45        # Current orientation in degrees
        self.shared_data = {
            "rgb": [],
            "distance": 0.0,
            "current_angle": 0
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
        blue_dominance_threshold = 50
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
        if direction > 0:

            for i in range(start, end + sweep_direction, sweep_direction):
                self.arm_motor.set_position(i)
                rgb = self.shared_data["rgb"]
                self.distances.append(self.shared_data["distance"])
                self.angles.append(i)
                is_blue = self.is_blue(rgb) if rgb else False

                if is_blue:
                    return True
                time.sleep(DELAY)
        

        else:
            for i in range(start, end + sweep_direction, sweep_direction):
                self.arm_motor.set_position(i)
                rgb = self.shared_data["rgb"]
                is_blue = self.is_blue(rgb) if rgb else False

                if is_blue:
                    return True
                time.sleep(DELAY)
                
        return water_detected
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
        self.turn(turn_angle)



        # Step 2: Calculate movement time based on distance
        dist_per_second = self.WHEEL_DIAMETER * math.pi * (300 / 360)  # Distance per second at 300 dps
        move_time = distance / dist_per_second  # Calculate time required to cover the distance

        print(f"Driving straight for {move_time:.2f} seconds toward the cube.")

        # Step 3: Drive straight toward the cube with optional heading correction
        self.drive_to_target_with_correction(center_angle, distance)
        print("Reached the cube.")
    def go_towards_cube(self):
        """
        Moves the robot straight towards the cube while keeping it in front.
        Stops if the robot is at least 4 cm away from the cube.
        """
        print("Moving towards cube...")

        while True:
            distance = self.front_us.get_value()
            if distance and distance <= 4:  # Stop if the robot is within 4 cm of the cube
                print(f"Cube reached. Final distance: {distance} cm.")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)
                break

            if distance is not None:
                print(f"Current distance: {distance} cm. Moving forward.")
                self.left_motor.reset_encoder()
                self.right_motor.reset_encoder()
                self.drive_straight(speed=-200, duration=0.1)
                self.update_position(0,self.heading)
            else:
                print("No valid distance reading. Stopping motors to avoid collision.")
                self.left_motor.set_dps(0)
                self.right_motor.set_dps(0)
                break

            time.sleep(0.1)  # Allow for smooth updates
    
    def come_back_to_zero_degrees(self):
        
        start = self.arm_motor.get_position()
        for i in range(start, SWEEP_START, -2):
            self.arm_motor.set_position(i)
            time.sleep(DELAY)
            
            
    def turn_to_cube(self, center_angle):
        """
        Turns the robot towards the detected cube, starting about 10 degrees before the given angle.
        The turn is limited to ±30 degrees for controlled movement.
        """
        MAX_TURN_ANGLE = 30  # Limit the turn to ±30 degrees
        PRE_TURN_OFFSET = 10  # Start turning 10 degrees before the center angle
        print(f"Preparing to turn towards cube at approximate angle {center_angle}°...")

        # Step 1: Rotate the arm to face forward (90°)
        self.arm_motor.set_position(90)
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
        self.turn(turn_angle)
        time.sleep(0.5)  # Allow time for the turn to complete

        # Step 4: Fine-tune alignment by analyzing distances
        print("Fine-tuning alignment with cube...")
        fine_tune_step = 2  # Smaller steps for fine-tuning
        while True:
            distance = self.front_us.get_value()
            if distance and distance < 30:  # Detect the cube based on distance drop
                print(f"Cube aligned. Distance: {distance} cm.")
                break

            # Make fine adjustments to center on the cube
            self.turn(fine_tune_step)  # Adjust turning step for fine-tuning
            time.sleep(0.2)  # Increase delay for better sensor readings

        print("Alignment with cube complete.")
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
                
                self.drive_straight(speed=100, duration=1)
                self.update_position(0, self.heading)
                self.turn(90)

    def go_to_wall(self):
        #set the arm forward:
        self.arm_motor.set_position(90)
        time.sleep(2.0)
        
        while True:
            distance = self.front_us.get_value()
            if distance and distance <= 5:
                print("Wall Detected, Stoppping")
                break
            self.drive_straight(speed=-200, duration=0.5)
            time.sleep(0.01)
        self.arm_motor.set_position(180)
        self.drive_straight(speed=-200, duration=0.75)
        
        return True
    def align_to_wall(self, tolerance=5, target_distance=110):
        print("Starting wall alignment...")
        total_adjustments = 0
        max_adjustments = 15
        first_detection = False
        initial_turns = 0  # Counter for initial turns
        
        # Set ultrasonic sensor to point left
        self.arm_motor.set_position(180)
        time.sleep(1)
        
        # Initial large turns to find wall
        while True:
            # First do 2 turns without checking distance
            if initial_turns < 2:
                print(f"Initial turn {initial_turns + 1}...")
                self.turn(15)
                time.sleep(0.1)
                self.drive_straight(speed=-150, duration=0.3)
                initial_turns += 1
                continue
                
            distance = self.front_us.get_value()
            if distance is None:
                print("No valid distance reading. Retrying...")
                time.sleep(0.1)
                continue
                
            print(f"Current distance: {distance} cm")
            
            # If this is our first detection of a wall
            if not first_detection and distance <= target_distance:
                print("First wall detected (likely corner). Continuing turn...")
                first_detection = True
                # Continue turning for 2 seconds to get past the corner
                start_time = time.time()
                while time.time() - start_time < 2:
                    self.turn(10)
                    self.drive_straight(speed=-150, duration=0.3)
                    time.sleep(0.1)
                continue
                
            # After we've passed the corner, check for the actual target wall
            if first_detection and distance <= target_distance + 10:
                print("Found target wall, starting fine adjustment...")
                break
                
            # Turn left in larger increments until wall is detected
            print("Turning left to find wall...")
            self.turn(10)
            time.sleep(0.1)
        
        # Fine adjustment phase
        while total_adjustments < max_adjustments:
            distance = self.front_us.get_value()
            if distance is None:
                print("No valid distance reading. Retrying...")
                time.sleep(0.1)
                continue

            error = distance - target_distance
            print(f"Current distance: {distance} cm. Error: {error:.2f} cm.")

            if abs(error) <= tolerance:
                print("Aligned with wall at target distance.")
                break

            if error > tolerance:  # Too far from wall (positive error)
                print("Too far from wall. Turning right to get closer...")
                self.turn(-5)
                self.drive_straight(speed=-150, duration=0.3)
            elif error < tolerance:  # Too close to wall (negative error)
                print("Too close to wall. Turning left to move away...")
                self.turn(5)
                self.drive_straight(speed=-150, duration=0.3)
                
            total_adjustments += 1
            time.sleep(0.1)
        
        if total_adjustments >= max_adjustments:
            print("Warning: Maximum adjustment attempts reached")
        else:
            print("Wall alignment complete.")
#     def align_to_wall(self, tolerance=0.5, target_distance=2.5):
#         print("Starting wall alignment...")
#         total_turn=0
#         self.arm_motor.set_position(0)  # Ensure ultrasonic sensor is pointing right
#         time.sleep(1)
#         self.turn(30)
#         self.drive_straight(-100,0.75)
#         self.turn(30)
#         self.drive_straight(-100,0.75)
#         self.turn(30)
#         self.drive_straight(-100,0.75)
#         while True:
#             distance = self.front_us.get_value()
#             if distance is None:
#                 print("No valid distance reading. Retrying...")
#                 time.sleep(0.1)
#                 continue
# 
#             error = distance - target_distance
#             print(f"Current distance: {distance} cm. Error: {error:.2f} cm.")
# 
#             # Check if the distance is within the acceptable tolerance
#             if abs(error) <= tolerance or total_turn >= 5:
#                 print("Aligned with the wall.")
#                 break
# 
#             # Incremental adjustment
#             if error > tolerance :  # Too far from the wall
#                 print("Too far from the wall. Turning LEFT and moving further...")
#                 print("error is " + str(error))
#                 self.turn(-5)  # Turn slightly left
#                 self.drive_straight(speed=-200, duration=0.3)  # Move closer
#                 total_turn += 1
#             elif error < tolerance:  # Too close to the wall
#                 print("Too close to the wall. Turning RIGHT and moving closer...")
#                 print("error is " + str(error))
#                 self.turn(5)  # Turn slightly right
#                 self.drive_straight(speed=-200, duration=0.3)  # Move away
#                 total_turn += 1
#             print(total_turn)
#             # Small forward movement after each adjustment
#             #print("Making a small forward move...")
#             #self.drive_straight(speed=-200, duration=0.4)
#         self.turn(-10)
#         print("Wall alignment complete.")
    # Phase 2: Align with the wall
        #print("Aligning with the wall...")
        #self.arm_motor.set_position(0)
        #self.turn(45)  # Initial turn to face along the wall
        #self.drive_straight(speed=-200,duration=0.5)
        #self.turn(45)
        #while True:
            #distance = self.front_us.get_value
        #while True:
            #distance = robot.front_us.get_value()
            #if distance and abs(distance - 5) <= 1:  # Maintain ~5 cm
                #break
            #elif distance > 5:
                #robot.drive_straight(speed=100, duration=0.1)  # Move closer
            #elif distance < 5:
                #robot.drive_straight(speed=-100, duration=0.1)  # Move away
    
    def hug_wall_until_yellow(self,target_distance=5, tolerance=2):
        """
        Guides the robot to hug the wall and stop when the color sensor detects yellow.
        
        Parameters:
        - robot: The IntegratedRobot instance.
        - target_distance: Desired distance (in cm) from the wall.
        - tolerance: Acceptable range for maintaining the distance (in cm).
        """
        print("Starting wall hugging...")
            
        self.arm_motor.set_position(90)  # Ensure ultrasonic sensor is pointing forward

        while True:
            # Step 1: Read sensor values
            distance = self.front_us.get_value()
            rgb = self.floor_cs.get_rgb()

            # Step 2: Check for yellow
            if self.is_yellow(rgb):
                print("Yellow detected! Stopping wall hugging.")
                break

    #         # Step 3: Maintain distance from the wall
    #         if distance is None:
    #             print("No valid distance reading. Assuming straight path.")
    #         else:
    #             error = distance - target_distance
    #             print(f"Current distance: {distance:.2f} cm. Error: {error:.2f} cm.")
    # 
    #             if error > tolerance:  # Too far from the wall
    #                 print("Too far from the wall. Turning slightly left.")
    #                 robot.turn(-5)  # Turn slightly left
    #             elif error < -tolerance:  # Too close to the wall
    #                 print("Too close to the wall. Turning slightly right.")
    #                 robot.turn(5)  # Turn slightly right

            # Step 4: Move forward
            print("Moving forward along the wall...")
            self.drive_straight(speed=-200, duration=0.2)

            # Step 5: Handle corners
            front_distance = self.front_us.get_value()
            if front_distance is not None and front_distance <= 5:  # Corner detected
                self.arm_motor.set_position(180)
                self.drive_straight(-200, 1)
                print("Corner detected. Turning 90 degrees to the right.")
                self.turn(30)
                self.drive_straight(-200,1)
                self.turn(30)
                self.drive_straight(-200,1)
                self.turn(30)
                self.arm_motor.set_position(90)

        print("Wall hugging complete. Stopping robot.")
    def drop_off_items(self):
        print("Starting drop off sequence...")
        
        # Point sensor left to check wall
        self.arm_motor.set_position(180)
        time.sleep(1)
        
        # Move forward onto green square
        print("Moving forward to green square...")
        self.drive_straight(speed=-200, duration=5.0)
        
        # Turn left 90 degrees to face second yellow square
        print("Turning left to face second yellow area...")
        self.turn(45)
        self.drive_straight(speed=-200, duration=1.0)
        self.turn(45)
        # Point sensor forward
        self.arm_motor.set_position(90)
        time.sleep(1)
        
        # Move forward until we detect green (meaning we've left yellow)
        print("Moving forward until green detected...")
        while True:
            rgb = self.floor_cs.get_rgb()
            if not self.is_yellow(rgb):  # Assuming is_yellow() function exists
                print("Left yellow area, stopping...")
                self.drive_straight(speed=-200, duration=1.5)
                break
            self.drive_straight(speed=-200, duration=0.5)
            time.sleep(0.1)
        
        # Point sensor left again
        print("Positioning for drop off...")
        self.arm_motor.set_position(180)
        time.sleep(1)
        
        # Open hatch to drop items
        
        print("Opening hatch to drop items...")
        self.door_motor.set_limits(30)
        self.door_motor.reset_encoder()
        self.door_motor.set_position(-90)
        
        # Back up
        print("Backing up...")
        self.drive_straight(speed=200, duration=5)

    


def main():
    try:
        robot = IntegratedRobot()
        input("Press Enter to start")
        robot.reset()
        direction = 1
        #robot.arm_motor.reset_encoder()
        #robot.arm_motor.set_power(20)
        #robot.arm_motor.set_dps(50)
        robot.arm_motor.set_limits(70,220)
        #self.arm_motor.reset_encoder()

        # Start thread for sensor monitoring
        #sweep_thread = threading.Thread(target=robot.thread_arm_sensors)
        #sweep_thread.start()

        
        robot.go_to_wall()
        print("DONE")
        robot.align_to_wall()
        print("DONEALIGNING")
        time.sleep(1)
        robot.hug_wall_until_yellow()
        print("GOT HOME NOW DUMPING")
        time.sleep(1)
        robot.drop_off_items()
        print("ARRIVEDHOME")
            

    except BaseException as e:
        print(traceback.format_exc())
        print(e)
    finally:
        print(robot.position[0],robot.position[1])
        robot.map.printMap()
        print("Exiting program")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
 