#!/usr/bin/env python3
"""
Launch file for Task 3: Bug1.
This launch file only launches the Bug1 node, as the environment (robot + SLAM + Waypoint Follower)
should be launched separately using:
    ros2 launch robot_launch.launch.py

After launching the environment, launch this file:
    ros2 launch cw1_team_12 run_solution_task_3.launch

Then send goals using:
    ros2 topic pub /goal geometry_msgs/Pose2D "{x: 4.0, y: 4.0, theta: 0.0}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the use_sim_time argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Launch Bug1 node with configuration
    bug1_node = Node(
        package='cw1_team_12',
        executable='bug1_node',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time
        }],
        remappings=[
            # Configure topic remappings for Bug1 node
            ('goal', '/goal'),          # Listen for goals on /goal topic
            ('waypoint', 'waypoint'),   # Publish waypoints for the follower
            ('Odometry', 'Odometry'),   # Listen to robot odometry
            ('local_map_lines', 'local_map_lines')  # Get obstacle information
        ]
    )
    
    # Create launch description with just the Bug1 node
    return LaunchDescription([
        declare_use_sim_time,
        bug1_node
    ])

if __name__ == '__main__':
    generate_launch_description()
