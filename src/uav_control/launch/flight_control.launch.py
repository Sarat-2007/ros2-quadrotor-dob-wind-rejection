from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_node = Node(
        package='uav_control',
        executable='flight_controller_node',
        name='flight_controller_node',
        output='screen'
    )

    trajectory_node = Node(
        package='uav_control',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen'
    )

    return LaunchDescription([
        controller_node,
        trajectory_node
    ])
