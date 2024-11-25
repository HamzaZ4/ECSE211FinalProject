from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import utils.brick
import time
import math
import random
import traceback
import threading
from MapModule import *
#from isBlock import *

SWEEP_SPEED = 220  # Speed in degrees per second
DELAY = 0.01  # Delay between degrees for smooth and accurate readings
SWEEP_START = 0
SWEEP_END = 200

def calculate_x(distance, heading):
    """
    Calculate x displacement considering direction of movement
    
    :param distance: Distance traveled (can be positive or negative)
    :param heading: Current heading in degrees
    :return: X displacement
    """
    return distance * math.cos(math.radians(heading))
    
def calculate_y(distance, heading):
    """
    Calculate y displacement considering direction of movement
    
    :param distance: Distance traveled (can be positive or negative)
    :param heading: Current heading in degrees
    :return: Y displacement
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
        self.right_motor = Motor ("B")
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
            "rgb" : [],
            "distance" : 0.0,
            "current_angle" : 0
        }
        #self.lock = threading.Lock()
        wait_ready_sensors()
        self.arm_motor.set_limits(70, SWEEP_SPEED)
        #self.arm_motor.set_power(60)
        #self.arm_motor.set_dps(SWEEP_SPEED)
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

    def is_red(self,rgb):
        r, g, b = rgb
        red_dominance_threshold = 50
        return r > g + red_dominance_threshold and r > b + red_dominance_threshold
   
    def is_blue(self, rgb):
        r, g, b = rgb
        blue_dominance_threshold = 50
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold
    
    def find_exact_blue(self, blue_intervals):
        
        real_blue_intervals = []
        interval = blue_intervals[0]
        if interval == []:
            return None
        if len(interval) < 2:
            return interval[0]
            
        cur_start, cur_end = interval[0]
        for i in range(len(interval) - 1):
            next_start, next_end = interval[i + 1]
                
            if abs(cur_end - next_start) < 20:
                cur_end = next_end
            else:
                real_blue_intervals.append((cur_start, cur_end))
                cur_start = next_start
            return (cur_start, cur_end)
        
    def thread_arm_sensors(self):
        
        while True:
            self.shared_data["rgb"] = self.floor_cs.get_rgb()
            self.shared_data["distance"] = self.front_us.get_value()
            self.shared_data["current_angle"] = self.gyro.get_abs_measure()
            time.sleep(DELAY)
            
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
        signed_distance = self.calculate_distance(left_counts, right_counts) 

        # Calculate x and y displacements
        current_x = calculate_x(signed_distance, self.heading)
        current_y = calculate_y(signed_distance, self.heading)

        # Update total position
        self.position[0] += current_x
        self.position[1] += current_y
        

    # Function to drive the robot straight
    def drive_straight(self, speed=-200, duration = 0.2):
        
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
            
            #print(f"Error: {current_error}, Left: {left_speed}, Right: {right_speed}")

            
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        
    # Function to turn the robot
    def turn(self, angle):
        """
        self.arm_motor.reset_encoder()
        self.arm_motor.set_limits(power = 20)
        self.arm_motor.set_position(-85)
        time.sleep(0.2)
        """
        initial_heading = self.gyro.get_abs_measure()
        target_heading = initial_heading + angle

        if angle > 0:  # Turn right
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(0)
        else:  # Turn left
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(-100)

        while abs(self.gyro.get_abs_measure() - target_heading) > 2:
            pass

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        #self.arm_motor.set_position(85)
        self.heading = target_heading % 360  # Update global heading
        
    def avoidCube(self):
        distance = self.front_us.get_raw_value()
        print(distance)
        if distance <= 6:
            print("obstacle!")
            self.turn(random.choice([-90, 90]))  # Turn left or right
            print("turn finished")
    
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


    
def main():
    try:
        robot = IntegratedRobot()
        input("Press Enter to start")
        robot.reset()
        robot.arm_motor.set_power(50)
        robot.arm_motor.set_dps(100)
        robot.arm_motor.set_position(34)

        #make threads here
        sweep_thread = threading.Thread(target=robot.thread_arm_sensors)
        sweep_thread.start()

        #might need to do something to restart if error occurs
        
        current_sweep_data = []

        while True:
            blue_intervals = robot.sweep()
            
            print(blue_intervals)
            time.sleep(DELAY)
            
            exact_blue = robot.find_exact_blue(blue_intervals)
            print(f"blue: {exact_blue}")
            

            # Read motor encoders
            #left_counts = robot.left_motor.get_encoder()
            #right_counts = robot.right_motor.get_encoder()
            
                           # Calculate distance traveled
            #distance = robot.calculate_distance(left_counts, right_counts)

            #odometry
            #need to make sure that a backwards movement results in a negative change in position
            #current_x = calculate_x(distance, robot.heading)
            #current_y = calculate_y(distance, robot.heading)

            #maybe something like if heading > 180* or <180* 
            

            #robot.Map.updateMap(total_x,total_y)
           
           
            #TAGET AQUISITION

            #newdirection = 0 #sweep funciton call for nearest block
            #robot.turn(newdirection)
            #robot.drive_straight()

            #COLLECTION OR AVOIDANCE
            
            #if(distance from US is <3cm):
                #setsweep boolean to true
            #with robot.lock:
                #robot.shared_data["block_collection"] = True #moves over to 145*
                #robot.shared_data["block_approach_angle"] = 90

            #go forward 5 cm 
            #robot.drive_straight()
            #if (robot.is_blue(robot.floor_cs.get_rgb())):
                #avoid block here
                #set block_approach_angle to 0 for right and 180 for left
                #remember to update the map that this square is checked
                #pass
            #else:
                #robot.drive_straight()
                #with robot.lock:
                    #robot.shared_data["block_approach_angle"] = 5
                    #if (robot.is_blue(robot.floor_cs.get_rgb)):
                        #avoid block again
                        #update position and map
                        #pass
                    #else:
                        #collect block code
                            #approach
                            #update position and map
                        #pass
            

            # Update position
            #robot.update_position(distance, robot.heading)
            #robot.avoidCube()

            # Reset encoders
            # we need to think of another time to reset encoders
            # I think we need to track total distance as well as current heading
            #at least we need to increment total x and y before resetting
            #robot.left_motor.reset_encoder()
            #robot.right_motor.reset_encoder()

            # Drive straight for 1 second
            #robot.drive_straight(speed=-200, duration = 0.2)
            #robot.avoidCube()

            # Print position and heading
            #print(f"Position: {robot.position}, Heading: {robot.heading}°")
            
            #  robot.avoidCube()
            #print(robot.floor_cs.get_rgb())
                
    except BaseException as e:
        #print(traceback.format_exc())
        print(e)
        pass
    finally:
        print("exiting program")
        reset_brick()
        exit()
        
        
    
if __name__ == "__main__":
    main()



