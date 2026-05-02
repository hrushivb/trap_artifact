from initial_vehicle import initial_sim
from vehicle_status import get_location
from remove_vehicle import generate_bus, delete_all_objects
import math
import time


def getObsDist(x0, y0, x1, y1):

    return math.sqrt((x0-x1)**2+(y0-y1)**2)


def attack():

    initial_sim(True)
    position = get_location(True)
    
    while position.x < 81645:
        position = get_location(True)
        pass
    
    quaternion = [0.0, 0.0, 0.980710678, 0.64278768]

    quaternion = [0.0, 0.0, 0.76604444, 0.64278761]
    
    # quaternion = [0.0, 0.0, 0.350710678, 0.64278768]
    bus = generate_bus(True, 81661.2, 50426.8, quaternion)
    # bus = generate_bus(True, 81651.6, 50462.4)

    check_x, check_y = 81661.2, 50426.8

    dist = 100

    while dist>50:
        position = get_location(True)
        dist = getObsDist(position.x, position.y, check_x, check_y)
    
    bus.delete_all_objects()

    # quaternion = [0.0, 0.0, 0.990710678, 0.64278768]
    quaternion = [0.0, 0.0, 0.400710678, 0.64278768]
    # quaternion = [0.0, 0.0, -0.400710678, -0.64278768]
    bus = generate_bus(True, 81661.2, 50426.8, quaternion)
    position = get_location(True)
    # bus = generate_bus(True, 81655.0, 50450.4)
    # time.sleep(4)
    # bus.delete_all_objects()

    # quaternion = [0.0, 0.0, 0.70710678, 0.70710678]
    
    
    # bus = generate_bus(True, 81677.9, 50357.6, quaternion)
    
    while position.x<81673.49:
        position = get_location(True)
        # obsDist = getObsDist(position.x, position.y, 81659.80257707079, 50447.25175823945)
        print('Distance to Obstacle:', position.x, position.y)


    bus.delete_all_objects()



if __name__ == '__main__':
    attack()