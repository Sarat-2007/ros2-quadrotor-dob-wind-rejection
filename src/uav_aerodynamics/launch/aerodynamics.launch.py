from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wind_field_node = Node(
        package='uav_aerodynamics',
        executable='wind_field_node',
        name='wind_field_node',
        output='screen'
    )

    drag_node = Node(
        package='uav_aerodynamics',
        executable='aerodynamic_drag_node',
        name='aerodynamic_drag_node',
        output='screen'
    )

    return LaunchDescription([
        wind_field_node,
        drag_node
    ])
