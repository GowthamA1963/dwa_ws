from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare
import os

def generate_launch_description():
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'launch/turtlebot3_world.launch.py')
        ])
    )

    dwa_node = Node(
        package='custom_dwa_planner',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen'
    )

    return LaunchDescription([
        turtlebot3_gazebo,
        dwa_node
    ])
