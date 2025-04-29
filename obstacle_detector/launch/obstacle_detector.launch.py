# # launchファイルはxmlの方を使う

# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='teamb_obstacle_detector',
#             executable='teamb_obstacle_detector_node',
#             name='teamb_obstacle_detector',
#             output='screen',
#             parameters=[
#                 'src/chibi25_b/obstacle_detector/config/obstacle_detector.yaml',
#                 # {'use_sim_time': True}
#             ],
#             arguments=['--ros-args', '--params-file', 'src/chibi25_b/obstacle_detector/config/obstacle_detector.yaml'],
#         ),
#     ])



