from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, EV3GyroSensor, wait_ready_sensors
import math
import time
import random

# Robot parameters
SWEEP_SPEED = 220
DELAY = 0.01
SWEEP_START = 0
SWEEP_END = 200


class IntegratedRobot:
    def __init__(self):
        print("Initializing robot...")
        self.gyro = EV3GyroSensor(3)
        self.front_us = EV3UltrasonicSensor(1)
        self.floor_cs = EV3ColorSensor(4)
        self.left_motor = Motor("C")
        self.right_motor = Motor("B")
        self.arm_motor = Motor("D")

        # PID parameters
        self.KP = 1.0
        self.MAX_CORRECTION = 60

        # Physical dimensions
        self.WHEEL_DIAMETER = 4.2
        self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
        self.AXLE_LENGTH = 10.5  # Distance between wheels in cm

        # Position tracking
        self.position = [0, 0]
        self.heading = 0

        wait_ready_sensors()
        self.arm_motor.set_limits(70, SWEEP_SPEED)
        print("Robot initialized")

    def reset(self):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.gyro.set_mode("abs")
        self.gyro.reset_measure()

    def is_blue(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        return b > r + 50 and b > g + 50

    def normalize_angle(self, angle):
        """Normalize angle to [0, 360) degrees"""
        return angle % 360

    def get_gyro_angle(self):
        """Get normalized gyro angle"""
        angle = self.gyro.get_abs_measure()
        return self.normalize_angle(angle) if angle is not None else self.heading

    def sweep(self, start=SWEEP_START, stop=SWEEP_END):
        """Sweep arm to detect nearest blue cube and return the angle"""
        print("Starting sweep...")
        self.arm_motor.set_position(start)
        time.sleep(1)  # Allow arm to reach start position
        blue_detected_angle = None
        min_distance = float('inf')

        for angle in range(start, stop + 1, 10):  # Step through angles
            self.arm_motor.set_position(angle)
            time.sleep(DELAY)

            rgb = self.floor_cs.get_rgb()
            if self.is_blue(rgb):
                distance = self.front_us.get_value()
                if distance < min_distance:
                    min_distance = distance
                    blue_detected_angle = angle

        print(f"Nearest blue cube at angle: {blue_detected_angle}, distance: {min_distance} cm")
        return blue_detected_angle, min_distance

    def turn(self, target_angle):
        """Turn robot to target angle"""
        print(f"Turning to angle: {target_angle}")
        current_angle = self.get_gyro_angle()
        angle_to_turn = target_angle - current_angle
        angle_to_turn = (angle_to_turn + 360) % 360  # Normalize to [0, 360)

        if angle_to_turn > 180:
            # Turn left
            self.left_motor.set_dps(-100)
            self.right_motor.set_dps(100)
        else:
            # Turn right
            self.left_motor.set_dps(100)
            self.right_motor.set_dps(-100)

        while abs(self.get_gyro_angle() - target_angle) > 2:
            pass

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        self.heading = target_angle

    def drive_straight(self, speed=200, duration=1.0):
        """Drive robot straight while maintaining heading"""
        print("Driving straight...")
        start_time = time.time()
        target_angle = self.get_gyro_angle()

        while time.time() - start_time < duration:
            error = self.get_gyro_angle() - target_angle
            correction = max(min(self.KP * error, self.MAX_CORRECTION), -self.MAX_CORRECTION)
            self.left_motor.set_dps(speed - correction)
            self.right_motor.set_dps(speed + correction)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

    def approach_cube(self, target_angle, distance):
        """Navigate towards the detected cube"""
        self.turn(target_angle)
        time_to_drive = distance / (self.WHEEL_CIRCUMFERENCE / 360)  # Approximate time
        self.drive_straight(duration=time_to_drive)

    def avoid_obstacle(self):
        """Avoid obstacle if detected"""
        if self.front_us.get_value() <= 6:
            print("Obstacle detected! Avoiding...")
            self.turn(random.choice([-90, 90]))


def main():
    robot = IntegratedRobot()
    robot.reset()

    while True:
        # Find nearest blue cube
        target_angle, distance = robot.sweep()

        if target_angle is not None:
            robot.approach_cube(target_angle, distance)
        else:
            print("No cube detected, searching again...")

        # Check for obstacles
        robot.avoid_obstacle()
        time.sleep(1)


if __name__ == "__main__":
    main()
