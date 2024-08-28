## iris_drone

This repository contains a ROS2 package for simulating an ArduPilot Iris drone in Gazebo Classic. The setup is built using ROS2 Humble and is integrated with SLAM for autonomous navigation and mapping.



## Features

- **Drone Simulation:** Simulates the ArduPilot Iris drone in Gazebo Classic.
- **ROS2 Humble Integration:** Fully compatible with ROS2 Humble.
- **SLAM Integration:** Incorporates SLAM algorithms for real-time mapping and localization.

## Prerequisites

Before running the simulation, ensure you have the following installed:

- **ROS2 Humble**: Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html).
- **Gazebo Classic**: Install Gazebo Classic as described [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- **ArduPilot**: Set up ArduPilot following the [official documentation](https://ardupilot.org/dev/docs/ros2.html).

## Installation

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/iris_drone.git
cd ~/ros2_ws
colcon build

