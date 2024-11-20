from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import utils.brick
import time
import math
import random
import traceback
from MapModule import *

def calculate_x(distance, heading):
    return distance * math.cos(heading)
    
def calculate_y(distance, heading):
    return distance * math.sin(heading)


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
        wait_ready_sensors()
        print("Robot initialized")



# Function to reset sensors and motors
    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def normalize_angle(self, angle):
        """Normalize angle to be between 0 and 359 degrees"""
        return angle % 360
    
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
        # Convert heading to radians for trigonometry
        heading_radians = math.radians(angle)
        # Update coordinates
        self.position[0] += distance * math.cos(heading_radians)
        self.position[1] += distance * math.sin(heading_radians)

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
        total_x = 0
        total_y = 0

        

        #make threads here

        while True:
            # Read motor encoders
            left_counts = robot.left_motor.get_encoder()
            right_counts = robot.right_motor.get_encoder()

            # Calculate distance traveled
            distance = robot.calculate_distance(left_counts, right_counts)

            #odometry
            current_x = calculate_x(distance, robot.heading)
            current_y = calculate_y(distance, robot.heading)

            total_x += current_x
            total_y += current_y

            robot.Map.updateMap(total_x,total_y)
           
           
            #TAGET AQUISITION

            newdirection = 0 #sweep funciton call for nearest block
            robot.turn(newdirection)
            robot.drive_straight()

            #COLLECTION OR AVOIDANCE
            
            #if(distance from US is <3cm):
                #setsweep boolean to true

            #check to see if other side is blue  
            #sweep function call for is floor blue
            

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

