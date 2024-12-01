import time
import math
from utils.brick import *

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

        self.MAX_CORRECTION = 60
        self.last_error = 0
        self.integral = 0

        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_COUNTS_PER_ROTATION = 360
        self.AXLE_LENGTH = 10.5

        self.target_angle = 0
        self.position = [0, 0]
        self.heading = 0

        wait_ready_sensors()
        self.arm_motor.set_limits(70, 200)
        print("Robot initialized")

    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def move_towards_cube(self, distance):
        """
        Moves the robot towards the detected cube.
        """
        print(f"Moving towards cube at {distance} cm...")
        self.left_motor.set_power(50)
        self.right_motor.set_power(50)
        time.sleep(distance / 10)  # Move based on distance (adjust scaling as needed)
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
        print("Reached the cube.")

    def turn_away_from_wall(self):
        """
        Turns the robot 90 degrees away from the wall.
        """
        print("Wall detected! Turning away...")
        self.left_motor.set_power(-50)
        self.right_motor.set_power(50)
        time.sleep(1.0)  # Adjust time for 90-degree turn
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
        print("Turned away from the wall.")

class CubeDetector:
    DISTANCE_JUMP_THRESHOLD = 7  # Minimum change to trigger detection (cm)
    VERIFICATION_SWEEPS = 2
    TOLERANCE = 2  # Allowed difference between verification sweeps (cm)

    def __init__(self, robot):
        self.robot = robot
        self.distances = []
        self.angles = []

    def smooth_data(self, data, window_size=5):
        """
        Smooth noisy sensor data using a moving average.
        """
        smoothed = []
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            smoothed.append(sum(data[start:end]) / (end - start))
        return smoothed

    def perform_sweep(self, start_angle=0, end_angle=180, step=5):
        """
        Perform a sweep and collect raw distances and angles within the specified range.
        """
        raw_distances = []
        angles = []

        print("Performing sweep...")
        for angle in range(start_angle, end_angle + 1, step):
            self.robot.arm_motor.set_position(angle)
            time.sleep(0.2)  # Allow motor and sensor to stabilize
            distance = self.robot.front_us.get_raw_value()
            raw_distances.append(distance)
            angles.append(angle)
            print(f"{angle}deg, {distance}cm")

        # Store raw distances directly
        self.distances = raw_distances
        self.angles = angles

        print(f"Raw distances: {self.distances}")
        print(f"Angles: {self.angles}")

    def detect_block(self):
        """
        Detect blocks based on distance and angular span, excluding wall-related logic.
        """
        print("Detecting blocks...")
        detection_radius = 10  # cm
        block_distance_threshold = 10  # cm for identifying blocks

        blocks = []
        candidates = []
        current_interval = []

        print("Starting refined block detection...")

        # Group intervals of distances within the detection radius
        for i in range(len(self.distances)):
            if self.distances[i] <= detection_radius:
                current_interval.append((self.angles[i], self.distances[i]))
            else:
                if current_interval:
                    candidates.append(current_interval)
                    current_interval = []

        if current_interval:  # Add the last interval if it exists
            candidates.append(current_interval)

        print(f"Candidate intervals: {candidates}")

        # Analyze each candidate
        for interval in candidates:
            angles_in_interval = [pair[0] for pair in interval]
            distances_in_interval = [pair[1] for pair in interval]

            angular_span = angles_in_interval[-1] - angles_in_interval[0]
            distance_variation = max(distances_in_interval) - min(distances_in_interval)

            print(f"Analyzing interval {angles_in_interval[0]}° to {angles_in_interval[-1]}°:")
            print(f"  Angular span: {angular_span}°")
            print(f"  Distance variation: {distance_variation} cm")

            # Check for a single-point or narrow block
            if angular_span <= 25 and all(d < block_distance_threshold for d in distances_in_interval):
                print(f"Detected block at {angles_in_interval[0]}° to {angles_in_interval[-1]}°.")
                blocks.append((angles_in_interval[0], angles_in_interval[-1]))

        print(f"Detected blocks: {blocks}")
        return {"blocks": blocks}

    def verify_block(self, angle):
        """
        Perform a second sweep at the suspected angle to verify consistency.
        """
        self.robot.arm_motor.set_position(angle)
        time.sleep(0.2)
        first_distance = self.robot.front_us.get_raw_value()

        # Perform additional sweeps for verification
        for _ in range(self.VERIFICATION_SWEEPS):
            time.sleep(0.5)
            self.robot.arm_motor.set_position(angle)
            second_distance = self.robot.front_us.get_raw_value()

            if abs(first_distance - second_distance) > self.TOLERANCE:
                # Distance is inconsistent; likely not a block
                return False

        return True

# Main program
if __name__ == "__main__":
    try:
        robot = IntegratedRobot()
        detector = CubeDetector(robot)

        print("Starting block detection...")
        detector.perform_sweep()

        blocks = detector.detect_block()
        if blocks:
            print(f"Blocks detected at angles: {blocks}")
        else:
            print("No blocks detected.")

    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        print("Exiting program.")
        reset_brick()
        exit()
