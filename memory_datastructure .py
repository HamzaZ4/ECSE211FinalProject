import math

class MemoryDataStructure:
    def __init__(self):
        # Initialize the list to store distances, one for each angle from 0 to 180
        self.single_list = [float('inf')] * 181

    def update_list(self, angle, distance):
        """
        Update the distance at a specific angle in the list.
        
        :param angle: The angle at which the distance was measured (integer, 0–180).
        :param distance: The distance measured at the given angle.
        """
        if 0 <= angle <= 180:
            self.single_list[angle] = distance
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
