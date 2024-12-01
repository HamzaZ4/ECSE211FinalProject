from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, reset_brick, wait_ready_sensors
import utils.brick
import time
import math
import traceback


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
        self.arm_motor.set_limits(70, 220)
        print("Robot initialized")

    def reset(self):
        self.left_motor.reset_encoder()
        self.arm_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def normalize_angle(self, angle):
        return angle % 360

    def angle_difference(self, target, current):
        diff = (target - current + 180) % 360 - 180
        return diff

    def get_gyro_angle(self):
        value = self.gyro.get_value()
        if value is not None:
            if isinstance(value, list) or isinstance(value, tuple):
                angle = value[0]
            else:
                angle = value
            return self.normalize_angle(angle)
        return self.heading

    def turn(self, angle):
        angle = angle * -1
        initial_heading = self.get_gyro_angle()
        target_heading = self.normalize_angle(initial_heading + angle)
        print(f"Turning from {initial_heading:.2f}° to {target_heading:.2f}° (angle: {angle:.2f}°)")

        if angle < 0:
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)
        else:
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)

        start_time = time.time()
        while True:
            current_heading = self.get_gyro_angle()
            heading_error = self.angle_difference(target_heading, current_heading)

            if abs(heading_error) < 2:
                break
            if time.time() - start_time > 5:
                break

            time.sleep(0.01)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        self.heading = self.get_gyro_angle()
        print(f"Turn complete. New heading: {self.heading:.2f}°")

    def drive_to_target_with_correction(self, target_angle, distance):
        print(f"Driving toward target angle: {target_angle:.2f}° with distance: {distance:.2f} cm.")
        dist_per_second = self.WHEEL_DIAMETER * math.pi * (300 / 360)
        drive_time = distance / dist_per_second

        start_time = time.time()
        while time.time() - start_time < drive_time:
            current_heading = self.get_gyro_angle()
            heading_error = self.angle_difference(target_angle, current_heading)

            correction = self.KP * heading_error
            left_speed = -200 - correction
            right_speed = -200 + correction

            self.left_motor.set_dps(left_speed)
            self.right_motor.set_dps(right_speed)

            time.sleep(0.01)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print("Reached target.")


class CubeDetector:
    def __init__(self, robot):
        self.robot = robot
        self.distances = []
        self.angles = []

    def perform_sweep(self, start_angle=0, end_angle=180, step=5):
        distances = []
        angles = []
        print("Performing sweep...")
        for angle in range(start_angle, end_angle + 1, step):
            self.robot.arm_motor.set_position(angle)
            time.sleep(0.2)
            distance = self.robot.front_us.get_value()
            distances.append(distance)
            angles.append(angle)
            print(f"{angle}deg: {distance}cm")

        self.distances = distances
        self.angles = angles
        print("Sweep complete.")
        return distances, angles

    def detect_block(self):
        detection_radius = 10
        blocks = []
        current_block = []

        for i in range(len(self.distances)):
            if self.distances[i] <= detection_radius:
                current_block.append((self.angles[i], self.distances[i]))
            else:
                if current_block:
                    blocks.append(current_block)
                    current_block = []

        if current_block:
            blocks.append(current_block)
        if blocks:
            closest_block = min(blocks, key=lambda block: min(pair[1] for pair in block))
            center_angle = (closest_block[0][0] + closest_block[-1][0]) // 2
            min_distance = min(pair[1] for pair in closest_block)
            print(f"Target block center at {center_angle}° and {min_distance} cm.")
            return center_angle, min_distance
        else:
            print("No blocks detected.")
            return None, None

    def move_towards_cube(self, center_angle, distance):
        if center_angle is None or distance is None:
            print("No cube detected. Cannot move towards it.")
            return

        print(f"Targeting cube at center angle: {center_angle:.2f}° with distance: {distance:.2f} cm.")
        turn_angle = (center_angle - 90 + 360) % 360
        if turn_angle > 180:
            turn_angle -= 360

        print(f"Calculated turn angle: {turn_angle:.2f}°.")
        self.robot.turn(turn_angle)
        self.robot.drive_to_target_with_correction(center_angle, distance)
        print("Reached the cube.")


if __name__ == "__main__":
    try:
        robot = IntegratedRobot()
        detector = CubeDetector(robot)

        print("Starting cube detection and navigation...")
        robot.reset()

        distances, angles = detector.perform_sweep(start_angle=0, end_angle=180, step=5)
        center_angle, distance = detector.detect_block()

        if center_angle is not None and distance is not None:
            detector.move_towards_cube(center_angle, distance)
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
