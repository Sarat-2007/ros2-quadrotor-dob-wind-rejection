from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_gui_node = Node(
        package='uav_dashboard',
        executable='uav_teleop_gui',
        name='uav_teleop_dashboard',
        output='screen'
    )

    return LaunchDescription([
        teleop_gui_node
    ])
