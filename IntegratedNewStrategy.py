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

        self.Map
        self.shared_data = {
            "angle": 0,
            "distance": 0,
            "is_blue": False,
            "is_yellow": False,
            "block_collection" : False,
            "block_approach_angle" : -999
        }
        self.lock = threading.Lock()
        wait_ready_sensors()
        print("Robot initialized")




# Function to reset sensors and motors
    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def is_yellow(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        return b <= 75 and r >= 200 and g >= 170
    
    def normalize_angle(self, angle):
        """Normalize angle to be between 0 and 359 degrees"""
        return angle % 360
    
    def is_blue(self, rgb):
        r, g, b = rgb
        blue_dominance_threshold = 50
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold

    def sweep(self):
        try:
            self.arm_motor.reset_encoder()
            current_position = 0
            direction = 1
            time.sleep(DELAY)

            while not self.stop_event.is_set():

                with self.lock:
                    get_out_of_way = self.shared_data["block_collection"]
                    direction_move = self.shared_data["block_approach_angle"]
                if get_out_of_way:
                    if direction_move == 90:
                        self.arm_motor.set_position(145)
                    elif direction_move > 90:
                        self.arm_motor.set_position(180)
                    else:
                        self.arm_motor.set_position(0)

                
                else:
                    current_position = current_position + direction
                    self.arm_motor.set_position(current_position)

                rgb = self.floor_cs.get_rgb()
                is_blue = self.is_blue(rgb) if rgb else False
                is_yellow = self.is_yellow(rgb) if rgb else False

                distance = self.front_us.get_value()
                with self.lock:
                    self.shared_data["angle"] = current_position
                    self.shared_data["distance"] = distance
                    self.shared_data["is_blue"] = is_blue
                    self.shared_data["is_yellow"] = is_yellow

                time.sleep(DELAY)
                print(f"Angle: {current_position}°, Distance: {distance:.2f} cm, Is Blue: {is_blue}")

                if current_position > SWEEP_END or current_position < SWEEP_START:
                    direction *= -1

        except Exception as e:
            print(f"Sweep error: {e}")

    def get_gyro_angle(self):
        """Get the normalized gyro angle"""
        angle = self.gyro.get_abs_measure()
        if angle is not None:
            return self.normalize_angle(angle)
        return self.target_angle
    
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
                
    
def main():
    try:
        robot = IntegratedRobot()
        input("Press Enter to start")
        robot.reset()
        

        #make threads here
        sweep_thread = threading.Thread(target=robot.sweep)
        sweep_thread.start()

        #might need to do something to restart if error occurs

        while True:
            # Read motor encoders
            left_counts = robot.left_motor.get_encoder()
            right_counts = robot.right_motor.get_encoder()

            # Calculate distance traveled
            distance = robot.calculate_distance(left_counts, right_counts)

            #odometry
            #need to make sure that a backwards movement results in a negative change in position
            current_x = calculate_x(distance, robot.heading)
            current_y = calculate_y(distance, robot.heading)

            #maybe something like if heading > 180* or <180* 
            

            robot.Map.updateMap(total_x,total_y)
           
           
            #TAGET AQUISITION

            newdirection = 0 #sweep funciton call for nearest block
            robot.turn(newdirection)
            robot.drive_straight()

            #COLLECTION OR AVOIDANCE
            
            #if(distance from US is <3cm):
                #setsweep boolean to true
            with robot.lock:
                robot.shared_data["block_collection"] = True #moves over to 145*
                robot.shared_data["block_approach_angle"] = 90

            #go forward 5 cm 
            robot.drive_straight()
            if (robot.is_blue(robot.floor_cs.get_rgb())):
                #avoid block here
                #set block_approach_angle to 0 for right and 180 for left
                #remember to update the map that this square is checked
                pass
            else:
                robot.drive_straight()
                with robot.lock:
                    robot.shared_data["block_approach_angle"] = 5
                    if (robot.is_blue(robot.floor_cs.get_rgb)):
                        #avoid block again
                        #update position and map
                        pass
                    else:
                        #collect block code
                            #approach
                            #update position and map
                        pass
            

            # Update position
            robot.update_position(distance, robot.heading)
            #robot.avoidCube()

            # Reset encoders
            # we need to think of another time to reset encoders
            # I think we need to track total distance as well as current heading
            #at least we need to increment total x and y before resetting
            robot.left_motor.reset_encoder()
            robot.right_motor.reset_encoder()

            # Drive straight for 1 second
            robot.drive_straight(speed=-200, duration = 0.2)
            #robot.avoidCube()

            # Print position and heading
            print(f"Position: {robot.position}, Heading: {robot.heading}°")
            
            #  robot.avoidCube()
            print(robot.floor_cs.get_rgb())
                
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

