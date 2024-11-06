#!/usr/bin/env python3

"""
This test is used to collect data from the color sensor.
It must be run on the robot.
"""

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor, reset_brick
from time import sleep
from BrickPi import *

DELAY_SEC = 0.01
COLOR_SENSOR_DATA_FILE = "../data_analysis/color_sensor.csv"



# complete this based on your hardware setup

COLOR_SENSOR = EV3ColorSensor(2)
TOUCH_SENSOR = TouchSensor(1)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.

COLORS = {
        0: "No Color",
        1: "Black",
        2: "Blue",
        3: "Green",
        4: "Yellow",
        5: "Red",
        6: "White",
        7: "Brown"
        # Add more color IDs and names as needed
    }

def collect_color_sensor_data():
    "Collect color sensor data."
    
    try:
        #Need to create the file or overrite it if it exists
        
        color_int = (int)(COLOR_SENSOR.get_value())
        

        #The file now just appends to the created file

        while True:
            if TOUCH_SENSOR.is_pressed():
                color_int = (int)(COLOR_SENSOR.get_value())
                COLORS.get(color_int,"UNKOWN")
    except BaseException as e:
        
        print(e) # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        pass

    finally:
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()


if __name__ == "__main__":
    collect_color_sensor_data()
