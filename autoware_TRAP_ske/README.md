
# Autoware planner fuzzer Guide

  

Before running the code, please first install ROS2 and Autoware.

  

## 1. Installing ROS2 Humble

  

To install ROS2 Humble, follow the official ROS2 installation guide:

  

[ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#setup-sources)

  

Make sure to follow the instructions for your specific operating system.

  

## 2. Installing Autoware

  

After installing ROS2 Humble, you can proceed with installing Autoware. Follow the source installation guide provided by the Autoware Foundation:

  

[Autoware Source Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)

  
  

## 3. Scripts

  

### 1. initial_sim.py

  

This script sets up the initial simulation by:

- Publishing the initial pose of the vehicle

- Publishing a goal pose

- Changing the operation mode to autonomous

  

### 2. generate_bus.py

  

This script generates/delete a dummy bus object in the simulation:

- Creates and publishes a bus object at a specified location

- Provides functionality to delete all objects in the simulation

  

### 3. get_location.py

  

This script retrieves the current position of the vehicle:

- Uses TF2 to get the transform between 'map' and 'base_link' frames

- Returns the current position of the vehicle

  
  

## Usage

  

A sample example script is in attack.py. To start the attack, first run the planning simulation of Autoware:
```
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/nishishinjuku_autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

After the RViz is popped out, run the attack script:
```
python3 attack.py
```

Then the ego vehicle controlled by the Autoware should spawn and drive to the preset goal. An obstacle will be spawned and deleted during the driving. 