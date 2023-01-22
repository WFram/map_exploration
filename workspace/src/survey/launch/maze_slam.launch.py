#!/usr/bin/env python3

# Written by Andrey Penkovskiy <earl.freedom.ea@gmail.com>, January 2023

import os
from re import S
from tracemalloc import start

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import ExecuteProcess, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    tb3_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_survey_path = "/workspace/src/survey"
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = pkg_survey_path + "/worlds/maze_built.world"
    
    rviz_config_path = os.path.join(pkg_survey_path, 'rviz/config_tb3.rviz')
    
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_params_path = os.path.join(pkg_survey_path, 'params', 'nav2_params_tb3.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')

    slam_toolbox_dir = FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    slam_toolbox_launch_dir = os.path.join(slam_toolbox_dir, 'launch')
    slam_toolbox_params_path = os.path.join(pkg_survey_path, 'params', 'slam_toolbox_tb3.yaml')
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
     )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    start_rviz_cmd = Node(package='rviz2',
                          executable='rviz2',
                          name='rviz2',
                          output='screen',
                          arguments=['-d', rviz_config_path])

    start_ros2_navigation_cmd = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
               launch_arguments = {'params_file': nav2_params_path}.items())

    start_slam_toolbox_cmd = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')),
          launch_arguments = {'slam_params_file' : slam_toolbox_params_path}.items())
    
    tf2_base_link_camera_cmd = Node(package = "tf2_ros",
                                     executable = "static_transform_publisher",
                                     arguments = ["0", "0", "0.128", "-0.5", "0.5", "-0.5", "0.5", "base_link", "camera"],
                                     output = "screen")
    
    tf2_base_link_fake_camera_cmd = Node(package = "tf2_ros",
                                     executable = "static_transform_publisher",
                                     arguments = ["0", "0", "0.128", "0", "0", "0", "1", "base_link", "fake_camera"],
                                     output = "screen")

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    # ld.add_action(tf2_base_link_camera_cmd)
    # ld.add_action(tf2_base_link_fake_camera_cmd)

    return ld
