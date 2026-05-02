import numpy as np
import itertools
import subprocess

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




points = calculate_intervals(81662.17,50436.71, 81682.234375, 50338.73828125)
ranges = [point[0] for point in points]
intervals = [[ranges[i], ranges[i + 1]] for i in range(len(ranges) - 1)]
combinations = list(itertools.product([0, 1], repeat=len(intervals)))

for i in range(len(combinations)):
    if i<354:
        continue
    try:
        # Run the other script with the JSON string as an argument
        if i == 0:
            first = True
        else:
            first = False
        subprocess.run(["python", "search_iteration.py", str(i), str(first)])
    except subprocess.CalledProcessError as e:
        print(f"Error running script: {e}")