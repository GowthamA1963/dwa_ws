from setuptools import setup

package_name = 'custom_dwa_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packagename', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/turtlebot3_dwa.launch.py']),  # Add launch file here
        ('share/' + package_name, ['rviz/dwa_view.rviz']),  # Add RViz config if applicable
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'dwa_planner = custom_dwa_planner.dwa_node:main',  # This runs your main function
        ],
    },
)
