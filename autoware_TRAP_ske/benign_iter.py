from initial_vehicle import initial_sim, stop_sim
from vehicle_status import get_location
from remove_vehicle import generate_bus, delete_all_objects
import rclpy
import time
import os
import numpy as np
import itertools
import csv
import sys

from scipy.spatial.transform import Rotation as R


def write_csv(csv_file, headers, lists):
    # Find the maximum length among the lists
    max_len = max(len(lst) for lst in lists)
    
    # Pad shorter lists with None or empty strings
    padded_lists = [lst + [None] * (max_len - len(lst)) for lst in lists]
    
    # Transpose the lists so that they can be written as columns
    rows = zip(*padded_lists)
    
    # Write to CSV file
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write the header
        writer.writerow(headers)
        # Write the rows
        writer.writerows(rows)


def calculate_intervals(x1, y1, x2, y2, interval=11):
    # Calculate the Euclidean distance between the points
    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Calculate the number of intervals
    num_intervals = int(distance // interval)
    
    # Get the unit direction vector from (x1, y1) to (x2, y2)
    direction_x = (x2 - x1) / distance
    direction_y = (y2 - y1) / distance
    
    initial = [(x1, y1)]
    final = [(x2, y2)]
    # Calculate the points
    points = [(x1 + i * interval * direction_x, y1 + i * interval * direction_y) for i in range(1, num_intervals + 1)]
    
    points = initial+points+final
    return points

def find_interval(x, intervals):
    for i, (start, end) in enumerate(intervals):
        if start < x <= end:
            return i  # Return the index of the interval
    return None  # Return None if x doesn't fall into any interval



def attack(first, x_pos, y_pos, quaternion, yaw_deg):

    x_pos_list = []
    y_pos_list = []
    time_list = []
    initial_sim(True)

    start_time = time.time()

    position = get_location(True)
    position = get_location(True)

    x_pos_list.append(position.x)
    y_pos_list.append(position.y)
    time_list.append(time.time() - start_time)
    
    while position.x < 81645:
        x_pos_list.append(position.x)
        y_pos_list.append(position.y)
        time_list.append(time.time() - start_time)
        position = get_location(True)
        pass
    
    pos_counter = 0
    
    bus = generate_bus(True, x_pos, y_pos, quaternion)
    # bus = generate_bus(True, 81651.6, 50462.4)

    prev_x, prev_y = position.x, position.y
    while position.x<81693.4:
        position = get_location(True)
        if [prev_x, prev_y] == [position.x, position.y]:
            pos_counter += 1
        else:
            pos_counter = 0
        
        if pos_counter >= 10:
            break
        prev_x, prev_y = position.x, position.y
        x_pos_list.append(position.x)
        y_pos_list.append(position.y)
        time_list.append(time.time() - start_time)
        
    
    time.sleep(4)
    bus.delete_all_objects()

    print('Stopping Simulation', x_pos, y_pos, yaw_deg)
    column_headings = ['X Position', 'Y Position', 'Time']
    write_csv('./logs/'+str(x_pos)+'_'+str(y_pos)+'_'+str(yaw_deg)+'.csv', column_headings, [x_pos_list, y_pos_list, time_list])



if __name__ == '__main__':

    if len(sys.argv) > 1:
        # Retrieve the argument (JSON string)
        x_pos = float(sys.argv[1])
        y_pos = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
        print(x_pos, y_pos, yaw_deg)
    
    yaw_rad = np.deg2rad(yaw_deg)  # Convert degrees to radians
    # Create a quaternion for rotation around Z-axis
    rotation = R.from_euler('z', yaw_rad)
    quaternion = rotation.as_quat()  # Get quaternion as [x, y, z, w]


    attack(True, x_pos, y_pos, quaternion, yaw_deg)






    