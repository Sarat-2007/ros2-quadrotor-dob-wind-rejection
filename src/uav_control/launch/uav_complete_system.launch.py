import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = get_package_share_directory('uav_description')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'quadrotor.urdf.xacro')
    rviz_config = os.path.join(desc_pkg, 'rviz', 'flight_sim.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # -------------------------------------------------------------
    # Launch Configurations & Arguments (All Default to True)
    # -------------------------------------------------------------
    enable_rviz = LaunchConfiguration('rviz', default='true')
    enable_teleop = LaunchConfiguration('teleop', default='true')
    enable_dashboard = LaunchConfiguration('dashboard', default='true')
    enable_wind = LaunchConfiguration('wind', default='true')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 3D real-time visualizer'
    )

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='Launch Pilot Remote Controller GUI (Transmitter / D-Pad HUD)'
    )

    declare_dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch Live Tuning & Disturbance Observer Studio Dashboard'
    )

    declare_wind_arg = DeclareLaunchArgument(
        'wind',
        default_value='true',
        description='Launch Dynamic Wind & 3D Aerodynamic Drag simulation nodes'
    )

    # -------------------------------------------------------------
    # 1. 3D Quadrotor Robot State Publisher
    # -------------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # -------------------------------------------------------------
    # 2. Dynamic Wind Field Generator Node
    # -------------------------------------------------------------
    wind_node = Node(
        package='uav_aerodynamics',
        executable='wind_field_node',
        name='wind_field_node',
        output='screen',
        condition=IfCondition(enable_wind)
    )

    # -------------------------------------------------------------
    # 3. Aerodynamic Drag Calculation Engine Node
    # -------------------------------------------------------------
    aero_node = Node(
        package='uav_aerodynamics',
        executable='aerodynamic_drag_node',
        name='aerodynamic_drag_node',
        output='screen',
        condition=IfCondition(enable_wind)
    )

    # -------------------------------------------------------------
    # 4. 6-DOF Flight Controller & Physics Simulation Node
    # -------------------------------------------------------------
    flight_node = Node(
        package='uav_control',
        executable='flight_controller_node',
        name='flight_controller_node',
        output='screen'
    )

    # -------------------------------------------------------------
    # 5. Autonomous Trajectory Generator Node
    # -------------------------------------------------------------
    traj_node = Node(
        package='uav_control',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen'
    )

    # -------------------------------------------------------------
    # 6. RViz2 3D Visualizer Node
    # -------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz)
    )

    # -------------------------------------------------------------
    # 7. Pilot Remote Controller GUI (Standalone Cockpit Transmitter)
    # -------------------------------------------------------------
    teleop_gui_node = Node(
        package='uav_dashboard',
        executable='uav_teleop_gui',
        name='uav_teleop_gui',
        output='screen',
        condition=IfCondition(enable_teleop)
    )

    # -------------------------------------------------------------
    # 8. Live Tuning & Disturbance Observer Studio Dashboard
    # -------------------------------------------------------------
    dashboard_gui_node = Node(
        package='uav_dashboard',
        executable='live_tuning_gui',
        name='live_tuning_gui',
        output='screen',
        condition=IfCondition(enable_dashboard)
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_teleop_arg,
        declare_dashboard_arg,
        declare_wind_arg,
        rsp_node,
        wind_node,
        aero_node,
        flight_node,
        traj_node,
        rviz_node,
        teleop_gui_node,
        dashboard_gui_node
    ])
