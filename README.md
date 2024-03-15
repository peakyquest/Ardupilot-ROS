# Ardupilot-ROS
## Prerequisites

Before cloning the this repository, you should have the following packages and components installed:

**ROS (Robot Operating System)**: This is the fundamental framework for your robot's software. Make sure you have ROS installed and set up on your system. You can choose a ROS distribution depending on your requirements (e.g., ROS Melodic, ROS Noetic).


**Ardupilot: ArduCopter** is a part of the ArduPilot project. You need to install ArduPilot on your system. ArduPilot is a comprehensive autopilot software suite that supports various vehicles, including copters (e.g., ArduCopter). You'll typically need to compile the ArduPilot source code and set up the appropriate environment variables..

**MAVProxy**: MAVProxy is a command-line ground control station (GCS) that can be used to communicate with ArduPilot vehicles. Install MAVProxy on your system to monitor and control your drone from the command line.

**MAVROS** ROS package: MAVROS is a ROS package that provides communication and interface capabilities between ROS and MAVLink-based autopilots like ArduPilot. Install MAVROS in your ROS workspace to facilitate communication between ROS and your ArduCopter.

**Ardupilot Gazebo plugins** : These plugins are specific to Gazebo simulations and help simulate ArduCopter behavior in Gazebo. You should clone the repository that contains these plugins and install them in your ROS workspace. You can typically find the plugins in a repository like ardupilot_gazebo.


## Packages

**iris_gazebo**

The iris_gazebo package provides simulation capabilities for the Iris quadcopter in the Gazebo simulator. It includes the necessary configurations and models to simulate the Iris drone within the Gazebo environment.

**iris_controller**

The iris_controller package enables you to control the Iris drone using MAVROS. It contains controllers and scripts to send commands to the Iris drone through MAVLink protocol, allowing for various flight operations and maneuvers.


