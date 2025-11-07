from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #  Use your actual package name 'diffbot_description'
    pkg_path = get_package_share_directory('diffbot_description')
    
    #  Use your actual file name 'diffbot.urdf'
    urdf_path = os.path.join(pkg_path, 'urdf', 'diffbot.urdf')
    
    # --- Read the file content ---
    try:
        with open(urdf_path, 'r') as file:
            robot_description_content = file.read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_path}.")
        # Exit or raise error if the core file is missing
        return LaunchDescription([]) 
    # -----------------------------

    return LaunchDescription([
        # joint_state_publisher handles the 'continuous' wheel joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_gui': True, 'source_list': ['joint_states']}]
        ),
        # robot_state_publisher reads the full URDF (including fixed joints like 'caster')
        # and publishes ALL transforms (dynamic from JSP, static from URDF).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}] 
        ),
        # RViz2 is launched to visualize the result
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])