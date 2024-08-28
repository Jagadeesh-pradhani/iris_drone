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
```
## Usage
1. Launch the Simulation:
   Navigate to your workspace and source the setup file:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ```
   Then, launch the simulation:
   ```bash
   ros2 launch iris_drone iris_drone.launch.py
   ```

2. Control the Drone:
   You can control the drone using ROS2 topics or integrate your SLAM package to enable autonomous navigation.

## SLAM Integration
   This package is designed to work seamlessly with various SLAM algorithms. To integrate SLAM:
  1. Launch your SLAM node alongside the drone simulation.
  2. Ensure the SLAM node subscribes to the appropriate topics (e.g., /drone/odom, /drone/scan).
  3. Adjust the parameters as necessary for optimal performance.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request or open an Issue for any improvements or bug fixes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- **[ArduPilot](https://ardupilot.org/):** For providing a powerful and flexible open-source platform for drone control and simulation.
- **[Gazebo Simulator](http://gazebosim.org/):** For offering a robust simulation environment that allows for realistic testing of robotic systems.
- **[ROS2](https://index.ros.org/doc/ros2/):** For enabling seamless integration of robotics software, making complex robot applications more accessible.
- **[SLAM Community](https://openslam.org/):** For continuous contributions to the development of SLAM algorithms that enhance autonomous navigation capabilities.

