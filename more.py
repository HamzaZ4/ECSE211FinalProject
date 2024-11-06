import time
import math
from ev3dev.ev3 import TouchSensor, ColorSensor, UltrasonicSensor, Motor, OUTPUT_A, OUTPUT_B, INPUT_1, INPUT_2, INPUT_3, INPUT_4

# Constants
MAP_SIZE = 122  # 122 cm x 122 cm (size of the map)
BLUE_COLOR_THRESHOLD = 15  # Color detection threshold for blueish color

# Motor setup
left_motor = Motor(OUTPUT_A)
right_motor = Motor(OUTPUT_B)

# Sensor setup
touch_sensor = TouchSensor(INPUT_1)
color_sensor = ColorSensor(INPUT_2)
ultrasonic_sensor = UltrasonicSensor(INPUT_3)

# Set up sensors
color_sensor.mode = 'COL-REFLECT'  # Reflective color mode
ultrasonic_sensor.mode = 'US-DIST-CM'  # Distance in cm

# Motor control functions
def move_forward(speed=300):
    left_motor.run_forever(speed_sp=speed)
    right_motor.run_forever(speed_sp=speed)

def turn_left(speed=300):
    left_motor.run_forever(speed_sp=-speed)
    right_motor.run_forever(speed_sp=speed)

def turn_right(speed=300):
    left_motor.run_forever(speed_sp=speed)
    right_motor.run_forever(speed_sp=-speed)

def stop_motors():
    left_motor.stop()
    right_motor.stop()

# Avoid wall logic using ultrasonic sensor
def check_for_obstacles():
    distance = ultrasonic_sensor.value() / 10  # Convert from mm to cm
    return distance < 20  # If distance is less than 20 cm, there's an obstacle

# Avoid blueish color logic using color sensor
def check_for_blue_area():
    color_value = color_sensor.value()
    return color_value < BLUE_COLOR_THRESHOLD  # Blue color threshold for avoiding area

# Main loop
def main():
    print("Waiting for touch sensor to start...")
    while not touch_sensor.is_pressed:
        time.sleep(0.1)  # Wait until touch sensor is pressed to start

    print("Starting robot navigation.")
    time.sleep(1)  # Small delay to ensure initial setup

    move_forward(speed=300)  # Move forward initially

    while True:
        if check_for_obstacles():
            print("Obstacle detected, turning...")
            stop_motors()
            turn_left(speed=300)
            time.sleep(1)  # Turn for 1 second
            stop_motors()
            move_forward(speed=300)  # Resume moving forward
        elif check_for_blue_area():
            print("Blue area detected, turning...")
            stop_motors()
            turn_right(speed=300)
            time.sleep(1)  # Turn for 1 second
            stop_motors()
            move_forward(speed=300)  # Resume moving forward

        time.sleep(0.1)  # Short delay to prevent excessive CPU usage

if __name__ == "__main__":
    main()
