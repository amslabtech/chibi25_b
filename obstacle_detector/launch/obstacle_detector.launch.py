from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teamb_obstacle_detector',
            executable='teamb_obstacle_detector_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='teamb_local_map_creator',
            executable='teamb_local_map_creator_node',
            parameters=[{'use_sim_time': True}],
            #parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1','/base_link', '/laser']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        )
    ])