import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_nav_pkg')
    nav2_share = get_package_share_directory('nav2_bringup')
    
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    rviz_config_file = os.path.join(pkg_share, 'config', 'default.rviz')

    # Nav2 Stack (The Brain)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'True'
        }.items()
    )

    # Static TF map -> odom (Assuming no SLAM drift)
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Static TF base_link -> camera_link (Camera is mounted slightly forward)
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # RViz (The Visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_visualizer',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        nav2_launch,
        tf_map_odom,
        tf_base_camera,
        rviz_node
    ])