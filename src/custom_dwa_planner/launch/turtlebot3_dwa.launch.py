import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        
        # Start the custom DWA Planner node
        Node(
            package='custom_dwa_planner',  # Your package name
            executable='dwa_planner',  # Your ROS2 node name
            name='dwa_planner_node',  # The node name you want to assign
            output='screen',  # Print logs to the screen
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/scan', '/scan'),  # Change topic names if needed
                        ('/odom', '/odom'),
                        ('/cmd_vel', '/cmd_vel')]
        ),
        
        # Start the TurtleBot3 simulation in Gazebo with the correct world file
        Node(
            package='turtlebot3_gazebo',  # The package for TurtleBot3 simulation
            executable='turtlebot3_gazebo_node',  # Correct executable for launching Gazebo
            name='turtlebot3_gazebo',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--world', 'turtlebot3_world'],  # Specify the TurtleBot3 world
        ),

        # Launch RViz with the correct configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__), 'dwa_view.rviz')],  # Replace with your RViz config path
        ),
    ])
