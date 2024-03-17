## Iris_Gazebo 

This ROS package provides essential functionality for controlling a copter's flight mode, executing basic flight operations, and conducting waypoint missions.

**mode.py**: This file enables users to get or set the flight mode of the copter. It contains functions to interface with the flight mode of the copter, allowing seamless integration into your ROS project.

**flight_task_control.py**: This file contains necessary basic flight operations. It includes functions for takeoff, landing, hovering, and other fundamental flight tasks. It serves as a foundational component for controlling the copter's flight behavior.

**waypoint_mission.py**: This file facilitates waypoint missions for the copter. It allows users to define a series of waypoints that the copter should navigate through autonomously. This is useful for tasks such as surveillance, mapping, or exploration

**param.py**: This file provides functionality to get and set parameters of the copter. It enables users to adjust various parameters such as flight control gains, maximum velocity, or mission-specific parameters on-the-fly.

## Usage 

1. Make sure your ROS environment is properly set up.

2. Launch the necessary ROS nodes for communication with the copter and other peripherals.

3. Run the desired Python scripts according to your requirements. For example:

```
rosrun iris_controller waypoint_mission.py
```
