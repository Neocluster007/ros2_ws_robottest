from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_dir = get_package_share_directory('brobot_navigation')

def generate_launch_description():



    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': os.path.join(pkg_dir, 'maps', 'map.yaml')
            }]
        ),
        
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'amcl.yaml')]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'planner_server.yaml')]
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'nav2_params.yaml')]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'bt_navigator.yaml')]
        ),

        # Waypoint follower (optional)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'nav2_params.yaml')]
        ),
        
    ])
