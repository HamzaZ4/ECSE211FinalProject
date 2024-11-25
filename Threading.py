import threading
import time
from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, reset_brick, wait_ready_sensors

SWEEP_SPEED = 220  # Speed in degrees per second
DELAY = 0.01  # Delay between degrees for smooth and accurate readings
SWEEP_START = 0
SWEEP_END = 200


class Robot:
    def __init__(self):
        print("Initializing robot...")
        self.gyro_sensor = EV3UltrasonicSensor(3)
        self.front_us = EV3UltrasonicSensor(1)
        self.block_cs = EV3ColorSensor(2)
        self.floor_cs = EV3ColorSensor(4)

        self.left_motor = Motor("C")
        self.right_motor = Motor("B")
        self.door_motor = Motor("A")
        self.arm_motor = Motor("D")

        print(1)
        self.stop_event = threading.Event()  # Shared flag to stop threads
        print(2)
        wait_ready_sensors()
        self.shared_data = {
            "angle": 0,
            "distance": 0,
            "is_blue": False,
            "is_yellow": False,
            "block_collection" : False,
            "block_approach_angle" : -999
        }
        self.lock = threading.Lock()
        print("Robot initialized")

    def is_blue(self, rgb):
        r, g, b = rgb
        blue_dominance_threshold = 50
        return b > r + blue_dominance_threshold and b > g + blue_dominance_threshold

    def is_yellow(self, rgb):
        r, g, b = rgb or (0, 0, 0)
        return b <= 75 and r >= 200 and g >= 170

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
                    if direction_move > 90:
                        self.arm_motor.set_position(180)
                    else:
                        self.arm_motor.set_position(0)

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

    def grid_traversal(self):
        print("TODO")

    def main(self):
        try:
            sweep_thread = threading.Thread(target=self.sweep)
            sweep_thread.start()

            # Add other threads if necessary
            while True:
                time.sleep(DELAY)
                with self.lock:
                    angle_shared = self.shared_data["angle"]
                    distance_shared = self.shared_data["distance"]
                    is_blue_shared = self.shared_data["is_blue"]
                print(f"Angle2: {angle_shared}° cm, Distance : {distance_shared}, Is Blue2: {is_blue_shared}")

        except KeyboardInterrupt:
            print("Stopping robot...")
            self.stop_event.set()
            self.arm_motor.set_power(0)
            self.left_motor.set_power(0)
            self.right_motor.set_power(0)
            reset_brick()
            print("Brick reset. Exiting program.")
        finally:
            reset_brick()


robot = Robot()
robot.main()