#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_forklift_gazebo = get_package_share_directory('forklift_gazebo')
    pkg_forklift_description = get_package_share_directory('forklift_description')
    pkg_forklift_control = get_package_share_directory('forklift_control')
    pkg_forklift_recognition = get_package_share_directory('forklift_recognition')
    print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" + pkg_forklift_recognition)

    joy_node = Node(
        package = "joy",
        executable = "joy_node"
    )

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_gazebo, 'launch', 'start_world_launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_description, 'launch', 'spawn_forklift_launch.launch.py'),
        )
    )     

    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_control, 'launch', 'forklift_control.launch.py'),
        )
    )  

    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forklift_recognition, 'launch', 'launch_yolov8.launch.py'),
        )
    )  

    return LaunchDescription([
        joy_node,
        start_world,
        spawn_robot_world,
        spawn_robot_control,
        spawn_yolo,
    ])
