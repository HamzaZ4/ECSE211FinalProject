from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, reset_brick, wait_ready_sensors
import time

inward_us = EV3UltrasonicSensor(1)
block_cs = EV3ColorSensor(3)

left_motor = Motor("C")
right_motor = Motor ("B")
door_motor = Motor("A")

block_cs.set_mode("RGB")
wait_ready_sensors()
print("done waiting")

def is_yellow():
        """Check if the block is yellow"""
        r, g, b = block_cs.get_rgb()  # Get RGB values
        if(r == None):
            r = 0
        if(g == None):
            g = 0
        if(b == None):
            b = 0
            
        print(f"RGB values: R={r} G={g} B={b}")
        # Simple heuristic to detect blueish color (you can adjust thresholds based on testing)
        if b <= 75 and r >= 200 and g >= 170:
            print("Poop detected.")
            return True
        return False

def collectingSystem():
        distance = inward_us.get_raw_value()
        if distance >= 2.5 and distance <= 3.5:
            print("Block detected")
            if is_yellow():
                 door_motor.reset_encoder()
                 door_motor.set_position(-50)
                 print("door opened")
                 time.sleep(1)
                 left_motor.set_position(-250)
                 right_motor.set_position(-250)
                 print("poop picked up")
                 time.sleep(1)
                 door_motor.set_position(50)
                 print("door closed")
                 time.sleep(1)
                 left_motor.set_position(250)
                 right_motor.set_position(250)
                 print("back to initial position")

    


if __name__ == "__main__":
    print("program started")
    try:
        while True:
            collectingSystem()
    except BaseException as e:
        print(e)
        pass
    finally:
        print("exiting program")
        reset_brick()
        exit()