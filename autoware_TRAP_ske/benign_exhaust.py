import numpy as np
import itertools
import subprocess


i = 0 
x_start, y_start = 81668.7, 50427.3
x_end, y_end = 81659.1, 50426.8
x_pos, y_pos = 81663.80257707079, 50423.25175823945

start = np.array([81668.7, 50427.3])
end = np.array([81659.1, 50426.8])

# Calculate the Euclidean distance between the points
distance = np.linalg.norm(end - start)

# Calculate the number of points at 0.5m intervals (including endpoints)
num_points = int(distance // 0.5) + 1

# Create the direction vector (normalized)
direction = (end - start) / distance

# Generate points at 0.5m intervals
points = [start + direction * (i * 0.5) for i in range(num_points)]


for point in points:
    x_pos = point[0]
    y_pos = point[1]
    for yaw_deg in range(0, 361, 20):  # 361 to include 360 degrees

        try:
        # Run the other script with the JSON string as an argument
            if i == 0:
                first = True
            else:
                first = False
            subprocess.run(["python3", "/home/ruoyu/autoware/aw-planner-fuzzer/benign_iter.py", str(x_pos), str(y_pos), str(yaw_deg)])
        except subprocess.CalledProcessError as e:
            print(f"Error running script: {e}")
    
