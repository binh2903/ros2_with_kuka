import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('kuka_kr3r540')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'kuka_kr3r540.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Include Gazebo launch from gazebo_ros
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Spawn entity using Gazebo ROS spawn_entity.py
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'kuka_kr3r540', '-file', urdf_file],
        output='screen'
    )
    
    # Static TF from base_link to base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        static_tf
    ])
