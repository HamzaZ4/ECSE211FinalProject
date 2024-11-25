import math

class MemoryDataStructure:
    def __init__(self):
        # Initialize the list to store distances, one for each angle from 0 to 180
        self.distance_list = [float('inf')] * 18

        #self.angled_dic = {"0-10":[float('inf')],"11-20":[float('inf')],"21-30":[float('inf')],"31-40":[float('inf')],
        #                   "41-50":[float('inf')],"51-60":[float('inf')],"61-70":[float('inf')],"71-80":[float('inf')],
        #                   "81-90":[float('inf')],"91-100":[float('inf')],"101-110":[float('inf')],"111-120":[float('inf')],
        #                   "121-130":[float('inf')],"131-140":[float('inf')],"141-150":[float('inf')],"151-160":[float('inf')],
        #                   "":[float('inf')],"171-180":[float('inf')]}

    def update_sweep_list(self, angle, distance):
        """
        Update the distance at a specific angle in the list.
        
        :param angle: The angle at which the distance was measured (integer, 0–180).
        :param distance: The distance measured at the given angle.
        """
        index = math.floor(angle / 10)
        
        if 0 <= index <= 18:
            self.single_list[index] = distance
        else:
            raise ValueError("Angle must be between 0 and 180 degrees.")

    def find_min_distance(self):
        """
        Find the minimum distance from the list and return the value and its corresponding angle.
        
        :return: A tuple containing (min_distance, min_angle).
        """
        min_distance = min(self.single_list)
        min_angle = self.single_list.index(min_distance)
        return min_distance, min_angle

    def reset(self):
        """
        Reset the list to its initial state, with all distances set to infinity.
        """
        self.single_list = [float('inf')] * 181

# Example usage
if __name__ == "__main__":
    memory = MemoryDataStructure()

    # Simulate updating distances
    memory.update_list(45, 100.0)  # Update angle 45 with distance 100.0
    memory.update_list(90, 50.0)  # Update angle 90 with distance 50.0
    memory.update_list(135, 75.0)  # Update angle 135 with distance 75.0

    # Find the minimum distance
    min_distance, min_angle = memory.find_min_distance()
    print(f"Minimum distance: {min_distance:.2f} cm at angle {min_angle}°")

    # Reset the list
    memory.reset()
    print("List reset.")
