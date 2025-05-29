#!/usr/bin/env python3
"""
Launch file for Task 1: Obstacle Follower.
This launch file starts the obstacle follower node that implements the solution for Task 1.
Note: This should be launched after the robot_launch.launch.py is running.

The node uses a parameters.json file for persistent storage of runtime data.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import json

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('cw1_team_12')
    
    # Define the path for parameters.json
    parameters_path = os.path.join(pkg_share, 'config', 'parameters.json')
    
    # Reset parameters.json with default values
    default_params = {
        "last_position": None,
        "position_check_time": None,
        "stuck_counter": 0
    }
    
    try:
        with open(parameters_path, 'w') as f:
            json.dump(default_params, f)
    except Exception as e:
        print(f"Warning: Could not reset parameters.json: {e}")
    
    # Declare the use_sim_time argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Launch Obstacle Follower node
    obstacle_follower_node = Node(
        package='cw1_team_12',
        executable='obstacle_follower_node',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
            "auto_start": True,
            "parameters_path": parameters_path
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        obstacle_follower_node
    ])

if __name__ == '__main__':
    generate_launch_description()
