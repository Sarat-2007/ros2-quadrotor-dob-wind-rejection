from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gui_node = Node(
        package='uav_dashboard',
        executable='live_tuning_gui',
        name='live_tuning_gui',
        output='screen'
    )

    return LaunchDescription([
        gui_node
    ])
