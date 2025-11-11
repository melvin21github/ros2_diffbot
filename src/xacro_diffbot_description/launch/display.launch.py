import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare # Recommended path finder

def generate_launch_description():
    # 1. FIND PACKAGE SHARE PATH (Recommended Method)
    # Use FindPackageShare to robustly locate the installed package data path
    pkg_share_path = FindPackageShare('xacro_diffbot_description')

    # 2. CONSTRUCT XACRO FILE PATH (Recommended Method)
    # Use PathJoinSubstitution to safely combine path segments
    xacro_file_path = PathJoinSubstitution([
        pkg_share_path, 
        'urdf', 
        'diffbot.xacro'
    ])

    # 3. CONVERT XACRO TO URDF (Recommended Method)
    # Use the Command substitution to execute 'xacro' at launch time
    robot_description_content = Command([
        'xacro ',
        xacro_file_path
    ])
    
    # 4. Define Nodes (Same as before, but with substitution variable)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}] # Pass the Command substitution
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])