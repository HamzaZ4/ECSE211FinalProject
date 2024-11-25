from utils.brick import EV3UltrasonicSensor, EV3ColorSensor, Motor, reset_brick, wait_ready_sensors
import time


class Robot:
    def __init__(self):
        self.block_cs = EV3ColorSensor(3)

        self.left_motor = Motor("C")
        self.right_motor = Motor ("B")
        self.door_motor = Motor("A")
        self.arm_motor = Motor("D")

        self.block_cs.set_mode("RGB")
        wait_ready_sensors()
        print("done waiting")


    def is_yellow(self):
            """Check if the block is yellow"""
            r, g, b = self.block_cs.get_rgb()  # Get RGB values
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
    
    def is_cube(self):
        r, g, b , l = self.block_cs.get_rgb()  # Get RGB values
        if(l == None):
            l = 0
        if l > 50 :
            return True
        return False
            
    def collect_cube(self):
        self.arm_motor.set_position(0)

        self.left_motor.set_position(-260)
        self.right_motor.set_position(-260)

        while not self.is_cube():
            print("no cube")
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(-5)
            time.sleep(0.1)
            self.left_motor.set_dps(-5)
            self.right_motor.set_dps(0)

        print("Block detected")
        if self.is_yellow():
            self.door_motor.reset_encoder()
            self.door_motor.set_position(-50)
            print("door opened")
            time.sleep(0.5)
            self.left_motor.set_position(-250)
            self.right_motor.set_position(-250)
            print("poop picked up")
            time.sleep(0.5)
            self.door_motor.set_position(50)
            print("door closed")
            time.sleep(0.5)
            self.left_motor.set_position(250)
            self.right_motor.set_position(250)
            print("back to initial position")

    
if __name__ == "__main__":
    print("program started")
    robot = Robot()
    try:
        while True:
            robot.collect_cube()
    except BaseException as e:
        print(e)
        pass
    finally:
        print("exiting program")
        reset_brick()
        exit()