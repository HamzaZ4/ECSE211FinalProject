#from more import move_forward
from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import time
import math
import traceback

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

        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_COUNTS_PER_ROTATION = 360

        self.target_angle = 0
        self.heading = 0  # Current orientation in degrees
        wait_ready_sensors()
        self.arm_motor.set_limits(70, SWEEP_SPEED)
        print("Robot initialized")

    def reset(self):
        """Resets the robot sensors and encoders."""
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def normalize_angle(self, angle):
        """Normalize angle to be between 0 and 359 degrees."""
        return angle % 360

    def get_gyro_angle(self):
        """Get the normalized gyro angle from the gyro sensor."""
        value = self.gyro.get_value()
        if value is not None:
            if isinstance(value, (list, tuple)):
                angle = value[0]
            else:
                angle = value
            return self.normalize_angle(angle)
        return self.heading

    def turn(self, angle):
        """Turns the robot by a specified angle."""
        initial_heading = self.get_gyro_angle()
        target_heading = self.normalize_angle(initial_heading + angle)
        print(f"Turning from {initial_heading}° to {target_heading}°.")

        if angle < 0:  # Turn right
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)
        else:  # Turn left
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)

        while True:
            current_heading = self.get_gyro_angle()
            heading_error = target_heading - current_heading
            heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

            if abs(heading_error) < 2:
                break
            time.sleep(DELAY)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print(f"Turn complete. New heading: {self.get_gyro_angle()}°.")

    def drive_straight(self, speed, duration):
        """Drives the robot straight for a specified duration."""
        self.left_motor.set_dps(speed)
        self.right_motor.set_dps(speed)
        time.sleep(duration)
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

    def drive_to_target_with_correction(self, target_angle, distance):
        """Drives the robot toward a target angle with heading correction."""
        print(f"Driving toward target angle: {target_angle}° for distance: {distance} cm.")
        start_time = time.time()
        while time.time() - start_time < distance / 10:  # Approximation for drive time
            current_angle = self.get_gyro_angle()
            heading_error = target_angle - current_angle
            heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

            correction = self.KP * heading_error
            self.left_motor.set_dps(-200 + correction)
            self.right_motor.set_dps(-200 - correction)

            time.sleep(DELAY)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

class CubeDetector:
    def __init__(self, robot):
        self.robot = robot
        self.distances = []
        self.angles = []

    def perform_sweep(self, start_angle=0, end_angle=180, step=5):
        """Performs a sweep to collect distances and angles."""
        print("Performing sweep...")
        self.distances = []
        self.angles = []
        for angle in range(start_angle, end_angle + 1, step):
            self.robot.arm_motor.set_position(angle)
            time.sleep(0.2)
            distance = self.robot.front_us.get_value()
            self.distances.append(distance)
            self.angles.append(angle)
            print(f"{angle}°: {distance} cm")
        print("Sweep complete.")

    def detect_block(self):
        """Detects blocks based on sweep data."""
        print("Detecting blocks...")
        detection_radius = 10  # cm
        min_angle = 20
        max_angle = 140
        max_span = 25

        blocks = []
        current_block = []

        for i, distance in enumerate(self.distances):
            angle = self.angles[i]
            if min_angle <= angle <= max_angle and distance <= detection_radius:
                current_block.append((angle, distance))
            else:
                if current_block:
                    blocks.append(current_block)
                    current_block = []

        if current_block:
            blocks.append(current_block)

        valid_blocks = []
        for block in blocks:
            if len(block) > 1:
                start_angle = block[0][0]
                end_angle = block[-1][0]
                span = end_angle - start_angle
                if span <= max_span:
                    center_angle = (start_angle + end_angle) / 2
                    min_distance = min([d[1] for d in block])
                    valid_blocks.append((center_angle, min_distance))

        if not valid_blocks:
            print("No valid blocks detected.")
            return None, None

        closest_block = min(valid_blocks, key=lambda b: b[1])
        print(f"Closest block detected at {closest_block[0]}° with distance {closest_block[1]} cm.")
        return closest_block

    def move_towards_cube(self, center_angle, distance):
        """Aligns with and moves toward the cube, adding 10 cm to the measured distance."""
        if center_angle is None or distance is None:
            print("No cube detected.")
            return

        # Align with the cube
        print(f"Aligning to center angle: {center_angle}°...")
        self.robot.turn(center_angle - 90)

        # Move towards the cube for the distance + 10 cm
        adjusted_distance = distance + 10
        print(f"Moving towards cube. Adjusted distance: {adjusted_distance} cm.")
        self.robot.drive_to_target_with_correction(center_angle, adjusted_distance)
        print("Reached the target location.")

if __name__ == "__main__":
    try:
        robot = IntegratedRobot()
        detector = CubeDetector(robot)
        robot.reset()

        # Perform sweep and detect block
        detector.perform_sweep()
        center_angle, distance = detector.detect_block()

        # Move towards detected block with adjusted distance
        if center_angle is not None:
            detector.move_towards_cube(center_angle, distance)
        else:
            print("No cube detected.")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        print("Shutting down.")
        reset_brick()
