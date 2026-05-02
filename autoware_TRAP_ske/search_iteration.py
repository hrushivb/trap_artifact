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



def attack(first, intervals, combination, number):
    print(combination)
    x_pos_list = []
    y_pos_list = []
    time_list = []
    initial_sim(first)
    start_time = time.time()
    position = get_location(True)
    x_pos_list.append(position.x)
    y_pos_list.append(position.y)
    time_list.append(time.time() - start_time)
    while position.x == 81662.17:
        position = get_location(True)
        if not first:
            os.system("ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}")

    x_pos_list.append(position.x)
    y_pos_list.append(position.y)
    time_list.append(time.time() - start_time)
    bus = generate_bus(True, 81682.234375, 50338.73828125)

    curr_position = get_location(False)
    curr_x = curr_position.x
    prev_x = 81662.17

    switch = 0
    print(curr_x, prev_x)
    while not (prev_x == curr_x) or not(curr_x > 81687.234375):
        prev_x = curr_x
        curr_position = get_location(True)
        x_pos_list.append(curr_position.x)
        y_pos_list.append(curr_position.y)
        time_list.append(time.time() - start_time)
        curr_x = curr_position.x
        interval_pos = find_interval(curr_x, intervals)
        if interval_pos is None:
            print('AV passed obstacle')
            break
        if combination[interval_pos] == switch:
            print('Already in same state')
        else:
            if combination[interval_pos] == 1:
                print('Removal Attack On')
                switch = 1
                bus.delete_all_objects()
            elif combination[interval_pos] == 0:
                print('Removal Attack Off')
                switch = 0
                bus = generate_bus(True, 81682.234375, 50338.73828125)
                
                
        print(prev_x, curr_x, '-----------------')

    print('Stopping Simulation', number, len(x_pos_list))
    column_headings = ['Combination', 'X Position', 'Y Position', 'Time']
    write_csv('./logs/'+str(number)+'.csv', column_headings, [list(combination), x_pos_list, y_pos_list, time_list])

    time.sleep(5)
    exit()

if __name__ == '__main__':

    points = calculate_intervals(81662.17,50436.71, 81682.234375, 50338.73828125)
    ranges = [point[0] for point in points]
    intervals = [[ranges[i], ranges[i + 1]] for i in range(len(ranges) - 1)]
    combinations = list(itertools.product([0, 1], repeat=len(intervals)))

    if len(sys.argv) > 1:
        # Retrieve the argument (JSON string)
        number = sys.argv[1]
        first = sys.argv[2]
        print(first, number)


    attack(bool(first), intervals, combinations[int(number)], int(number))

    # for i in range(1, len(combinations)):
    #     attack(False, intervals, combinations[i], i)






    