import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # RUTAS 
    pkg_share = get_package_share_directory('tesis_robot_control')
    nav2_params = os.path.join(pkg_share, 'config', 'mi_nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'config', 'mapa.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    web_dir = os.path.expanduser('~/web_tesis')

    return LaunchDescription([
        # 1. ROSBRIDGE 
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
            )
        ),

        # 2. SERVIDOR WEB
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=web_dir,
            output='screen'
        ),

        # 3. UWB CORE
        Node(
            package='tesis_robot_control',
            executable='uwb_core',
            name='uwb_core',
            output='screen'
        ),

        # 3.1 NODO IMU 
        Node(
            package='tesis_robot_control',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen'
        ),

        # 3.2 FILTRO DE KALMAN EXTENDIDO 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        ),

        # 3.3 TRANSFORMADA ESTÁTICA: MAP -> ODOM
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                    '--frame-id', 'map', 
                    '--child-frame-id', 'odom']
        ),

        # 3.4 TRANSFORMADA ESTÁTICA: BASE_LINK -> IMU_LINK
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_imu',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                    '--frame-id', 'base_link', 
                    '--child-frame-id', 'imu_link']
        ),

        # 4. CONTROLADOR MOTORES 
        Node(
            package='tesis_robot_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            remappings=[('/cmd_vel', '/cmd_vel_smoothed')],
            output='screen'
        ),

        # 5. NAV2 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
                'use_lifecycle_mgr': 'true',
                'amcl': 'false',
                'use_collision_monitor': 'false'
            }.items()
        ),
    ])