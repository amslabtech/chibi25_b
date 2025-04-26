import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the path to the map.yaml file as a launch argument
    map_yaml_file = DeclareLaunchArgument(
        'b_map_fixed_yaml_file',
        default_value=os.path.join(
            os.getcwd(), 'install', 'your_package_name', 'share', 'your_package_name', 'map', 'map.yaml'
        ),
        description='Path to the map.yaml file'
    )

    # Define the path to your custom package executable
    your_custom_package_path = os.path.join(os.getcwd(), 'install', 'your_package_name', 'lib', 'your_package_name')

    # Launch your custom package node
    custom_package_node = Node(
        package='teamb_global_path_planner', # Replace with your actual package name
        executable='teamb_global_path_planner_node',  # Replace with your actual executable name
        name='custom_node',
        output='screen',
        # Add any additional parameters or arguments as needed
    )

    # Launch the map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('b_map_fixed_yaml_file')
            }
        ]
    )

    # Bring up the map server node using nav2_util
    lifecycle_map_server_node = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='map_server_lifecycle',
        output='screen',
        arguments=['map_server']
    )

    # Create the launch description with all the defined nodes
    ld = LaunchDescription()

    # Add the map_yaml_file argument to the launch description
    ld.add_action(map_yaml_file)

    # Add your custom package node, map server node, and lifecycle_map_server_node to the launch description
    ld.add_action(custom_package_node)
    ld.add_action(lifecycle_map_server_node)
    ld.add_action(map_server_node)

    return ld