import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = get_package_share_directory('uav_description')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'quadrotor.urdf.xacro')
    rviz_config = os.path.join(desc_pkg, 'rviz', 'flight_sim.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. 3D Quadrotor State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 2. Dynamic Wind Field Generator
    wind_node = Node(
        package='uav_aerodynamics',
        executable='wind_field_node',
        name='wind_field_node',
        output='screen'
    )

    # 3. Aerodynamic Drag Calculation Engine
    aero_node = Node(
        package='uav_aerodynamics',
        executable='aerodynamic_drag_node',
        name='aerodynamic_drag_node',
        output='screen'
    )

    # 4. Flight Controller, Dynamics & Dynamic Rotor Joint State Publisher
    flight_node = Node(
        package='uav_control',
        executable='flight_controller_node',
        name='flight_controller_node',
        output='screen'
    )

    # 5. Trajectory Generator
    traj_node = Node(
        package='uav_control',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen'
    )

    # 6. RViz2 3D Real-Time Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 7. Interactive GUI Studio & Disturbance Observer Monitor
    gui_node = Node(
        package='uav_dashboard',
        executable='live_tuning_gui',
        name='live_tuning_gui',
        output='screen'
    )

    # 8. Pilot Remote Controller GUI (Standalone Cockpit Transmitter)
    teleop_gui_node = Node(
        package='uav_dashboard',
        executable='uav_teleop_gui',
        name='uav_teleop_gui',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        wind_node,
        aero_node,
        flight_node,
        traj_node,
        rviz_node,
        gui_node,
        teleop_gui_node
    ])
