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

    # Nav2 Stack (The Brain)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'True'
        }.items()
    )

    # RViz (The Visualizer)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_visualizer'
    )

    return LaunchDescription([
        nav2_launch,
        rviz_node
    ])