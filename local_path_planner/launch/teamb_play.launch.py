#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import LifecycleNode 
from launch.actions import ExecuteProcess, TimerAction
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teamb_obstacle_detector',
            executable='teamb_obstacle_detector_node',
            #parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        # Node(
        #     package='teamb_local_map_creator',
        #     executable='teamb_local_map_creator_node',
        #     parameters=[{'use_sim_time': True}],
            # parameters=[{'use_sim_time': False}],
        # ),
        Node(
            package='teamb_localizer',
            executable='teamb_localizer_node',
            # parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='teamb_global_path_planner',
            executable='teamb_global_path_planner_node',
            #parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='teamb_local_goal_creator',
            executable='teamb_local_goal_creator_node',
            #parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='teamb_local_path_planner',
            executable='teamb_local_path_planner_node',
            #parameters=[{'use_sim_time': True}],
             parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser'],
        ),
        
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/user/ws/src/last/chibi25_b/map/b_map_fixed.yaml'
            }]
        ),

        TimerAction(
            period=1.0,  # 秒数は状況に応じて調整（map_serverが準備できるくらい待つ）
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    name='map_server_lifecycle',
                    output='screen',
                    arguments=['map_server']
                )
            ]
        )

        

        
        
    ])


