import math

class Map:
    def __init__(self):
        MapArray = [[ False for _ in range (6)] for _ in range(6)]

        #if explored 1, else 0
        MapArray[0][0] = True
        MapArray[1][0] = True
        MapArray[0][1] = True
        
        #each square is 18cm by 18cm 
        
        #[[1][1][][][][]]
        #[[1][][][][][]]
        #[[][][][][][]]
        #[[][][][][][]]
        #[[][][][][][]]
        #[[][][][][][]]
    def update_map(self, x, y):
        '''
        Takes current postion and updates the map object accordingly

        inputs 
        robot current positon and heading

        output
        NONE

        '''
        grid_x = x // 16
        grid_y = y // 16

        #(54,70)
        self.MapArray[grid_x][grid_y] = 1 

    def should_travel_or_not(self, x, y, possible_heading, distance):
        '''
        Given current position(x,y), and the vector to an object 
        the function will return true if that square is not yet explored
        False if it has already been explored
         
        inputs:
        robot current postion
        
        Returns: 
        true if destination is already explored

        false if destination is not yet explored

        '''
        #need to cover edge case where we are scanning outside the map
        target_x = x + distance * math.cos(possible_heading)
        target_y = y + distance * math.sin(possible_heading)
        if((target_x >= 18 * 8) or (target_y >= 6 * 18)):
            return False
        return self.MapArray[target_x][target_y]
        
