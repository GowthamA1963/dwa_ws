Here's the `README.md` written normally, without extra markdown formatting for GitHub rendering issues:

---

# Custom DWA Planner for ROS2

This repository contains the implementation of a custom Dynamic Window Approach (DWA) local planner for ROS2, specifically designed for TurtleBot3 in a Gazebo environment. The planner computes velocity commands (`cmd_vel`) based on sensor data (e.g., LaserScan, Odometry) and provides smooth navigation while avoiding obstacles.

## Requirements

* ROS2 Humble
* Gazebo
* TurtleBot3 simulation setup

## Installation

1. **Install ROS2 Humble**: Follow the official installation guide for ROS2 Humble: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html).

2. **Install Gazebo**: Gazebo can be installed by following the instructions here: [Gazebo Installation](http://gazebosim.org).

3. **Install TurtleBot3 Packages**: Install the required TurtleBot3 simulation packages by running:

   `sudo apt install ros-humble-turtlebot3-gazebo`

4. **Clone the repository**: Clone the repository into your ROS2 workspace:

   `git clone <repository-url> ~/dwa_ws/src/custom_dwa_planner`

5. **Build the workspace**: Build the workspace using `colcon`:

   `cd ~/dwa_ws`

   `colcon build`

6. **Source the workspace**: Source the workspace to set up the environment:

   `source ~/dwa_ws/install/setup.bash`

## Running the Planner

1. **Run the TurtleBot3 simulation**: First, make sure to run the TurtleBot3 simulation in Gazebo and visualize the environment in RViz. Use the following command to start Gazebo and load the environment:

   `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

2. **Start RViz with the DWA view**: Open RViz and load the `dwa_view.rviz` configuration file to visualize the robot's trajectory and sensor data:

   `rviz2 -d dwa_view.rviz`

3. **Run the custom DWA Planner node**: Now, you can run the custom DWA planner node to start generating velocity commands based on the sensor data:

   `ros2 run custom_dwa_planner dwa_planner`

## Conclusion

This setup allows for navigation of the TurtleBot3 within a simulated environment using a custom Dynamic Window Approach (DWA) local planner. The planner will compute safe velocity commands based on the robot's current state, sensor data, and the environment.

If you want to make any modifications or contribute to this project, feel free to fork and make a pull request!

