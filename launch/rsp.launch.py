import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # Declare the xacro_file argument for GUI selection
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(get_package_share_directory('butler'), 'description', 'robot.urdf.xacro'),
        description='Path to the URDF.xacro file'
    )
    # Declare the use_sim_time argument, but cast it as a boolean
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    # Process the xacro file based on the provided path
    def process_xacro(context):
        xacro_file_path = LaunchConfiguration('xacro_file').perform(context)
        robot_description_config = xacro.process_file(xacro_file_path)
        return robot_description_config.toxml()

    # Create robot_state_publisher node after processing the xacro file
    def create_robot_state_publisher(context):
        robot_description = process_xacro(context)
        # Ensure we convert use_sim_time into a boolean
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
            )
        ]

    return LaunchDescription([
        xacro_file_arg,  # Argument for xacro file selection
        use_sim_time_arg,  # Argument for sim time toggle
        OpaqueFunction(function=create_robot_state_publisher)  # Call function to create node
    ])
