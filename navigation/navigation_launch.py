"""Dedicated localization/navigation launch; every velocity is remapped away from /cmd_vel."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    bringup = Path(get_package_share_directory('nav2_bringup'))
    params = str(root/'navigation'/'generated_params.yaml')
    mapfile = os.environ['ROBOT_NAV_MAP']
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('rplidar_ros'))/'launch'/'rplidar_a1_launch.py')),
            launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()),
        ExecuteProcess(cmd=['python3', str(root/'slam'/'laser_odom.py'),
                            '--ros-args', '-p', 'base_frame:=base_link'], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': mapfile}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', parameters=[{'autostart': True,
                  'node_names': ['map_server', 'amcl']}]),
    ]
    names = []
    for package, executable, name in [
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
    ]:
        remaps = [('cmd_vel', '/navigation/cmd_vel_raw')]
        if name == 'velocity_smoother':
            remaps.append(('cmd_vel_smoothed', '/navigation/cmd_vel_smoothed'))
        actions.append(Node(package=package, executable=executable, name=name,
                            parameters=[params], remappings=remaps, output='screen'))
        names.append(name)
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', parameters=[{'autostart': True,
                        'node_names': names, 'bond_timeout': 4.0}]))
    return LaunchDescription(actions)
