# Iris_Gazebo

## Overview 
The iris_gazebo ROS package facilitates the simulation of the Iris drone within the Gazebo environment. It integrates the Iris drone model with three essential sensors: camera, LiDAR, and sonar. These sensors enable realistic simulation scenarios, crucial for various robotic applications ranging from navigation to perception tasks.

## Requirements
ROS (Robot Operating System): Ensure that ROS is installed on your system.
Gazebo Simulator: Gazebo is required for simulating the drone environment.
Compatible Versions: This package is tested on ROS Noetic and Gazebo 11.

## Installation
1. Clone the Repository: Begin by cloning this repository into your ROS workspace:
    ```
    git clone <repository_url>
    ```
2. Build the Package: Navigate to your ROS workspace and build the package:
    ```
    cd <your_ros_workspace>
    catkin build 
    ```
3. Dont forget to add the gazebo path for the models, after adding path reset your computer. 

## Usage
### Launch Files
The package offers three launch files, each initializing the Gazebo environment with the Iris drone equipped with a specific sensor:

runway.launch: Simulates the Iris drone with a camera sensor.
iris_with_lidar.launch: Simulates the Iris drone with a LiDAR sensor.
iris_with_sonar.launch: Simulates the Iris drone with a sonar sensor.

```
roslaunch iris_gazebo <launch_file_name>.launch
```
Example of Iris with Camera:

![Screenshot from 2024-03-14 12-49-30](https://github.com/peakyquest/Ardupilot-ROS/assets/162409782/01f0cbc1-0712-4aa8-957a-95a2769e8f1d)

Example or Iris with Sonar:
![Screenshot from 2024-03-14 12-48-41](https://github.com/peakyquest/Ardupilot-ROS/assets/162409782/d6218c88-c07b-497c-8c91-0e74444644a9)

Example of Iris with Lidar:
 
![Screenshot from 2024-03-14 12-59-50](https://github.com/peakyquest/Ardupilot-ROS/assets/162409782/b78079fb-1be3-4006-aeac-0b6efa38a890)











