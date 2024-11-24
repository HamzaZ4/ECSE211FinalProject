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

        self.MAX_CORRECTION = 60  # Maximum correction for straight-line movement
        self.last_error = 0
        self.integral = 0

        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_COUNTS_PER_ROTATION = 360
        self.AXLE_LENGTH = 10.5  # distance between wheels in cm

        self.target_angle = 0
        self.position = [0, 0]  # [x, y] in cm
        self.heading = 0  # Current orientation in degrees

        wait_ready_sensors()
        self.arm_motor.set_limits(70, 200)
        print("Robot initialized")

    def reset(self):
        """
        Reset sensors and motors to initial state.
        """
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def shutdown(self):
        """
        Stop motors and cleanup resources.
        """
        self.left_motor.stop()
        self.right_motor.stop()
        self.arm_motor.stop()
        print("Motors stopped and resources cleaned up.")

class CubeDetector:
    DISTANCE_JUMP_THRESHOLD = 10  # cm
    MAX_CUBE_WIDTH = 30  # cm
    ANGLE_DIFF_THRESHOLD = 30  # degrees
    TOLERANCE = 5  # cm for verification consistency

    def __init__(self, robot):
        self.robot = robot
        self.distances = []
        self.angles = []

    def sweep_and_measure(self):
        """
        Perform a sweep and record distances and angles.
        """
        self.distances = []
        self.angles = []
        for angle in range(0, 181, 10):  # Sweep in steps of 10 degrees
            self.robot.arm_motor.set_position(angle)
            time.sleep(0.2)  # Allow sensor to stabilize
            distance = self.robot.front_us.get_raw_value()
            self.distances.append(distance)
            self.angles.append(angle)

    def detect_jumps(self):
        """
        Detect jumps in distance measurements and validate potential cubes.
        """
        for i in range(1, len(self.distances)):
            jump = abs(self.distances[i] - self.distances[i - 1])

            if jump > self.DISTANCE_JUMP_THRESHOLD:
                print(f"Jump detected at angle {self.angles[i]}°: {jump} cm")

                # 1. Check for symmetry
                if not self.check_symmetry(i):
                    print("Likely a corner. Skipping.")
                    continue

                # 2. Check width consistency
                if not self.check_width_consistency(i):
                    print("Width inconsistent. Likely a corner.")
                    continue

                # 3. Check angle difference
                if not self.check_jump_angle(i, i + 1):
                    print("Large angle difference. Likely a corner.")
                    continue

                # 4. Verify by revisiting the angle
                if self.verify_cube_at_angle(self.angles[i]):
                    print("Cube confirmed!")
                else:
                    print("Verification failed. Not a cube.")

    def check_symmetry(self, jump_index):
        """
        Check for symmetry by looking for another jump within 20-25 degrees
        with a similar jump value.
        """
        current_jump = abs(self.distances[jump_index] - self.distances[jump_index - 1])
        current_angle = self.angles[jump_index]

        for i in range(len(self.distances)):
            # Skip the current jump index
            if i == jump_index:
                continue

            # Check if the angle difference is within 20-25 degrees
            angle_diff = abs(self.angles[i] - current_angle)
            if 20 <= angle_diff <= 25:
                # Check if the jump value is similar
                neighboring_jump = abs(self.distances[i] - self.distances[i - 1])
                if abs(current_jump - neighboring_jump) < self.DISTANCE_JUMP_THRESHOLD / 2:
                    print(f"Symmetry detected: Current Jump = {current_jump}, Neighboring Jump = {neighboring_jump}")
                    return True

        print("No symmetry detected.")
        return False

    def check_width_consistency(self, jump_index):
        """
        Check the width between two jumps for consistency.
        """
        if jump_index + 1 >= len(self.distances):
            return False

        start_distance = self.distances[jump_index]
        end_distance = self.distances[jump_index + 1]
        width = abs(start_distance - end_distance)

        if width <= self.MAX_CUBE_WIDTH:
            print(f"Width consistent: {width} cm")
            return True

        print(f"Width inconsistent: {width} cm")
        return False

    def check_jump_angle(self, start_jump_index, end_jump_index):
        """
        Check if the angle difference between jumps is small, indicating a cube.
        """
        if end_jump_index >= len(self.angles):
            return False

        angle_diff = abs(self.angles[end_jump_index] - self.angles[start_jump_index])
        if angle_diff < self.ANGLE_DIFF_THRESHOLD:
            print(f"Angle difference small: {angle_diff}°")
            return True

        print(f"Angle difference too large: {angle_diff}°")
        return False

    def verify_cube_at_angle(self, angle):
        """
        Rotate to a specific angle and recheck distances to confirm a cube.
        """
        self.robot.arm_motor.set_position(angle)
        time.sleep(0.5)
        distance = self.robot.front_us.get_raw_value()
        time.sleep(0.5)
        rechecked_distance = self.robot.front_us.get_raw_value()

        if abs(distance - rechecked_distance) < self.TOLERANCE:
            print(f"Consistent distance at angle {angle}°. Likely a cube.")
            return True

        print(f"Inconsistent distance at angle {angle}°. Not a cube.")
        return False

# Example usage
if __name__ == "__main__":
    try:
        robot = IntegratedRobot()
        detector = CubeDetector(robot)

        print("Starting sweep and cube detection...")
        detector.sweep_and_measure()  # Perform a sweep
        detector.detect_jumps()       # Analyze jumps and validate cubes

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        print("exiting program")
        reset_brick()
        exit()
