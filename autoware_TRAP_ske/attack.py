from initial_vehicle import initial_sim
from vehicle_status import get_location
from remove_vehicle import generate_bus, delete_all_objects, start_moving_bus
#from remove_vehicle import dummy_bus_publisher
import math
import time
import rclpy


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

    #bus_publisher = DummyBusPublisher()

    
    #bus = bus_publisher.publish_dummy_bus(True, 81654.7, 50459.9, quaternion)
    bus_publisher = generate_bus(True, 81654.6, 50459.9, quaternion)
    #rclpy.spin(bus_publisher)
    time.sleep(5)


    #bus.delete_all_objects()

    check = 81657.1, 50461.4

    nquaternion = [0.0, 0.0, -0.700710678, -0.90278768]
    bus_publisher.modify_bus_position(
        new_x=81655, new_y=50459.9, new_quaternion=nquaternion
    )

    start_moving_bus(bus_publisher, 0.0, -0.5, nquaternion, 10)
    #bus = generate_bus(True, 81654.7, 50459.9, quaternion)

    # bus = generate_bus(True, 81655.0, 50450.4)
    # time.sleep(4)
    # bus.delete_all_objects()

    # quaternion = [0.0, 0.0, 0.70710678, 0.70710678]
    
    
    # bus = generate_bus(True, 81677.9, 50357.6, quaternion)
    
    while position.x<81673.49:
        position = get_location(True)
        # obsDist = getObsDist(position.x, position.y, 81659.80257707079, 50447.25175823945)
        #print('Distance to Obstacle:', position.x, position.y)

    while position.x<81655:
        print('Not Collided')
    
    print('Collided')
    bus_publisher.delete_all_objects()



if __name__ == '__main__':
    attack()