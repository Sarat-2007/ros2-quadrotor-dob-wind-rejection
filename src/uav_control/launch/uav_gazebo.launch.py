import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_pkg = get_package_share_directory('uav_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'quadrotor.urdf.xacro')
    world_file = os.path.join(desc_pkg, 'worlds', 'uav_world.world')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Launch Configurations
    gazebo_gui = LaunchConfiguration('gui', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='false')
    enable_dashboard = LaunchConfiguration('dashboard', default='false')

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "true" to display Gazebo 3D simulation window'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Optionally launch RViz2 alongside Gazebo'
    )

    declare_dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='false',
        description='Set to "true" to also open GCS teleop dashboard inside this launch'
    )

    # 1. Start Gazebo Server & Client (Official Physics Simulator with State Plugin)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': gazebo_gui, 'world': world_file}.items()
    )

    # 2. 3D Quadrotor Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 3. Spawn Drone Model Entity into Gazebo World
    spawn_drone_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_quadrotor_uav',
        arguments=['-entity', 'quadrotor_uav', '-topic', 'robot_description', '-z', '0.12'],
        output='screen'
    )

    # 4. 6-DOF Flight Controller & Aerodynamic Disturbance Observer (DOB)
    flight_node = Node(
        package='uav_control',
        executable='flight_controller_node',
        name='flight_controller_node',
        output='screen'
    )

    # 5. Dynamic Wind Field Generator
    wind_node = Node(
        package='uav_aerodynamics',
        executable='wind_field_node',
        name='wind_field_node',
        output='screen'
    )

    # 6. Aerodynamic Drag Calculation Engine
    aero_node = Node(
        package='uav_aerodynamics',
        executable='aerodynamic_drag_node',
        name='aerodynamic_drag_node',
        output='screen'
    )

    # 7. Official Ground Control Station (GCS) Dashboard
    gcs_gui_node = Node(
        package='uav_dashboard',
        executable='uav_teleop_gui',
        name='uav_gcs_dashboard',
        output='screen',
        condition=IfCondition(enable_dashboard)
    )

    return LaunchDescription([
        declare_gui_arg,
        declare_rviz_arg,
        declare_dashboard_arg,
        gazebo_launch,
        rsp_node,
        spawn_drone_node,
        flight_node,
        wind_node,
        aero_node,
        gcs_gui_node
    ])
