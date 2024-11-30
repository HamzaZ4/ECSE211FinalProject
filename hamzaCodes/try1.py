# from more import move_forward
from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import time
import math
import traceback

SWEEP_SPEED = 220  # Speed in degrees per second
DELAY = 0.01  # Delay between readings for smooth and accurate readings
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
        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.ENCODER_COUNTS_PER_ROTATION = 360

        wait_ready_sensors()
        self.arm_motor.set_limits(70, SWEEP_SPEED)
        print("Robot initialized")

    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def get_gyro_angle(self):
        """Get the normalized gyro angle from the gyro sensor."""
        value = self.gyro.get_value()
        if value is not None:
            if isinstance(value, list) or isinstance(value, tuple):
                angle = value[0]
            else:
                angle = value
            return angle % 360
        return 0  # Default to 0 if reading fails

    def turn(self, angle):
        """
        Turn the robot by a specified angle.
        """
        initial_heading = self.get_gyro_angle()
        target_heading = (initial_heading + angle) % 360
        print(f"Turning from {initial_heading:.2f}° to {target_heading:.2f}°.")

        # Determine turn direction
        if angle < 0:  # Turn right
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)
        else:  # Turn left
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)

        # Wait until the robot reaches the target heading
        start_time = time.time()
        MAX_TURN_DURATION = 5  # seconds
        while time.time() - start_time < MAX_TURN_DURATION:
            current_heading = self.get_gyro_angle()
            if abs(current_heading - target_heading) < 2:
                break
            time.sleep(DELAY)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print(f"Turn complete. New heading: {self.get_gyro_angle():.2f}°")

    def drive_to_target_with_correction(self, target_angle, distance):
        """
        Drives the robot toward the target angle while continuously correcting its heading.
        """
        print(f"Driving toward target angle: {target_angle:.2f}° for distance: {distance:.2f} cm.")
        dist_per_second = self.WHEEL_CIRCUMFERENCE * (300 / 360)  # Distance per second at 300 dps
        drive_time = distance / dist_per_second

        start_time = time.time()
        while time.time() - start_time < drive_time:
            current_angle = self.get_gyro_angle()
            heading_error = target_angle - current_angle
            correction = self.KP * heading_error

            left_speed = -200 - correction
            right_speed = -200 + correction
            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)

            time.sleep(DELAY)

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
            print(f"{angle}°: {distance} cm")

        self.distances = distances
        self.angles = angles
        print("Sweep complete.")
        return distances, angles

    def detect_block(self):
        """
        Enhanced detection to classify objects as blocks or walls.
        """
        print("Detecting blocks and walls...")

        detection_radius = 10  # cm
        max_block_span = 25  # Maximum angular span for blocks (degrees)
        wall_distance_variance_threshold = 2  # Variance threshold for walls

        blocks = []
        walls = []
        current_object = []

        for i in range(len(self.distances)):
            angle = self.angles[i]
            distance = self.distances[i]

            # Check if within detection range
            if distance <= detection_radius:
                current_object.append((angle, distance))
            else:
                if current_object:
                    # Analyze the object
                    start_angle = current_object[0][0]
                    end_angle = current_object[-1][0]
                    angular_span = end_angle - start_angle
                    distances_in_object = [pair[1] for pair in current_object]
                    distance_variance = max(distances_in_object) - min(distances_in_object)

                    if angular_span > max_block_span and distance_variance < wall_distance_variance_threshold:
                        print(f"IS A WALL: Spanning {angular_span}° with variance {distance_variance:.2f}.")
                        walls.append((start_angle, end_angle))
                    else:
                        block_center = (start_angle + end_angle) // 2
                        block_distance = min(distances_in_object)
                        print(f"Detected BLOCK at {block_center}° with distance {block_distance} cm.")
                        blocks.append((block_center, block_distance))

                    current_object = []

        if current_object:
            start_angle = current_object[0][0]
            end_angle = current_object[-1][0]
            angular_span = end_angle - start_angle
            distances_in_object = [pair[1] for pair in current_object]
            distance_variance = max(distances_in_object) - min(distances_in_object)

            if angular_span > max_block_span and distance_variance < wall_distance_variance_threshold:
                print(f"IS A WALL: Spanning {angular_span}° with variance {distance_variance:.2f}.")
                walls.append((start_angle, end_angle))
            else:
                block_center = (start_angle + end_angle) // 2
                block_distance = min(distances_in_object)
                print(f"Detected BLOCK at {block_center}° with distance {block_distance} cm.")
                blocks.append((block_center, block_distance))

        return blocks, walls

    def turn_to_cube(self, center_angle):
        """
        Turns the robot towards the detected cube.
        """
        print(f"Preparing to turn towards cube at angle {center_angle}°...")
        turn_angle = (center_angle - 90 + 360) % 360
        if turn_angle > 180:
            turn_angle -= 360  # Normalize to [-180, 180]

        print(f"Calculated turn angle: {turn_angle:.2f}°.")
        self.robot.turn(turn_angle)
        print("Turn complete. Robot aligned with the cube.")

    def move_towards_cube(self, center_angle, distance):
        """
        Moves the robot toward the cube located at the specified center angle.
        """
        if center_angle is None or distance is None:
            print("No cube detected. Cannot move towards it.")
            return

        print(f"Moving towards cube at angle {center_angle}° and distance {distance} cm.")
        self.turn_to_cube(center_angle)
        self.robot.drive_to_target_with_correction(center_angle, distance + 10)  # Add 10 cm margin
        print("Reached the cube.")


if __name__ == "__main__":
    try:
        # Initialize the robot and detector
        robot = IntegratedRobot()
        detector = CubeDetector(robot)

        print("Starting cube detection and navigation...")

        # Step 1: Reset the robot's sensors and motors
        robot.reset()

        # Step 2: Perform a sweep to populate distance and angle arrays
        distances, angles = detector.perform_sweep(start_angle=0, end_angle=180, step=5)

        # Step 3: Detect the closest block
        blocks, walls = detector.detect_block()

        if blocks:
            closest_block = min(blocks, key=lambda b: b[1])
            center_angle, distance = closest_block
            detector.move_towards_cube(center_angle, distance)
        elif walls:
            print("Detected walls but no valid blocks to approach.")
        else:
            print("No objects detected.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")
        traceback.print_exc()
    finally:
        print("Exiting program.")
        reset_brick()
