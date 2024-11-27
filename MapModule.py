import math

class Map:
    def __init__(self):
        self.MapArray = [[ False for _ in range(6)] for _ in range (6)]
#         MapArray[0][0]= True
#         MapArray[0][1]= True
#         MapArray[1][0]= True
    
    def update_map(self, x,y):
        grid_x = int(x // 16)
        grid_y = int(y // 16)
        
        try:
            self.MapArray[grid_x][grid_y] = True
            return 1
        except IndexError:
            return 0
    def printMap(self):
        for i in range(6):
            print(self.MapArray[i])
        
    def should_travel_or_not(self, x, y, possible_heading, distance):
        """
        
        """
        
        target_x = int(round(x + distance * math.cos(math.radians(possible_heading)), 2))
        target_y = int(round(y + distance * math.sin(math.radians(possible_heading)), 2))
        
        if(((target_x <= 0) or (target_y <= 0)) or (target_x >= 16 * 6) or (target_y >= 16 * 6)):
            return True
        else:
            return self.MapArray[target_x % 6][target_y % 6]
        
        
        
    def ston(self, robot, possible_heading, distance):
        
        x_coord = robot.position[0]
        y_coord = robot.position[1]
        #heading_rad = round(math.radians(possible_heading), 2)
        
        target_x = round(x + distance * math.cos(math.radians(possible_heading)), 2)
        target_y = round(y + distance * math.sin(math.radians(possible_heading)), 2)
        
        #S.T.O.N == should travel or not for current heading
        ston = should_travel_or_not(x_coord, y_coord, possible_heading, distance)
        
        #if STON isn't good
        if(((target_x <= 0) or (target_y <= 0)) or (target_x >= 16 * 6) or (target_y >= 16 * 6)) and ston:
            #find a better heading
            #by incrementing through possible headings at every 15deg
            for multiplier in range(6):
                
                #left turn is positive degree -> increase the heading
                newHeading_left = round((multiplier * 15 + possible_heading),2)
                
                #right turn is negative degree -> decrease the heading
                newHeading_right = round(((-1 * multiplier * 15) + possible_heading),2)
                
                
                left = MapArray.should_travel_or_not(x_coord, y_coord,newHeading_left ,distance)
                right = MapArray.should_travel_or_not(x_coord, y_coord,newHeading_right ,distance)
                
                #if left and right are good headings
                if (not left) and (not right):
                    #check twice the distance
                    left = MapArray.should_travel_or_not(x_coord, y_coord,newHeading_left ,distance * 2)
                    right = MapArray.should_travel_or_not(x_coord, y_coord,newHeading_right ,distance * 2)
                    
                    if not left: #if left is good
                        return True, newHeading_left
                    elif not right: #if right is good
                        return True, newHeading_right
                    else:
                        continue
                    
                elif not left: #if only left is good
                    return True, newHeading_left
                elif not right: #if only right is good
                    return True, newHeading_right
                else: #check next intervals
                    continue
        elif not ston: #if the original heading was good
            return True, 0
        else:
            raise Exception("STON undefined behavior")
                