#  dsr_bringup2 by Minsoo Song (minsoo.song@doosan.com)
#  Copyright (c) 2024 Doosan Robotics
# modified for Rokey project

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, GroupAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from dsr_bringup2.utils import read_update_rate, show_git_info
import yaml


def print_launch_configuration_value(context, *args, **kwargs):
    # Evaluate and print the 'gz' (Gazebo) LaunchConfiguration value
    gz_value = LaunchConfiguration('gz').perform(context)
    print(f'LaunchConfiguration gz: {gz_value}')
    return gz_value

def generate_launch_description():
    # Define Launch Arguments
    ARGUMENTS = [ 
        DeclareLaunchArgument('name',      default_value='dsr01',    description='Robot name/namespace'),
        DeclareLaunchArgument('model',     default_value='m0609',    description='Robot model (m0609, a0509, etc.)'),
        DeclareLaunchArgument('color',     default_value='white',    description='Robot color'),
        DeclareLaunchArgument('gripper',   default_value='none',     description='Gripper type'),
        DeclareLaunchArgument('mobile',    default_value='none',     description='Mobile base type'),
        DeclareLaunchArgument('remap_tf',  default_value='false',    description='Remap TF for multi-robot setups'),
        DeclareLaunchArgument('rviz',      default_value='true',     description='Launch RViz'),
        DeclareLaunchArgument('gz',        default_value='false',    description='Use Gazebo simulation'),
    ]

    # Load Robot Description (URDF/Xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("dsr_description2"), "xacro", 
                             PythonExpression(["'", LaunchConfiguration('model'), ".urdf.xacro'"])]), " ",
        "name:=", LaunchConfiguration('name'), " ",
        "model:=", LaunchConfiguration('model'), " ",
        "color:=", LaunchConfiguration('color'), " ",
        "gripper:=", LaunchConfiguration('gripper'), " ",
        "mobile:=", LaunchConfiguration('mobile'), " ",
    ])
    robot_description = {"robot_description": robot_description_content}

    # Node: Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration('name'),
        output="both",
        parameters=[robot_description],
    )

    # Node: Doosan Emulator (Simulates robot hardware)
    run_emulator_node = Node(
        package="dsr_launcher2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        arguments=[LaunchConfiguration('model'), LaunchConfiguration('name')],
        output="screen",
    )

    # Node: RViz Visualizer
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "rviz", "default.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Controller Spawner: Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    # Controller Spawner: Doosan Robot Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # Sequential Launch Handling (Event Handlers)
    # Start Robot Controller after Joint State Broadcaster finishes
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Start RViz after Robot Controller is ready
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )
    
    nodes = [
        robot_state_pub_node,
        run_emulator_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller,
        delay_rviz,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
