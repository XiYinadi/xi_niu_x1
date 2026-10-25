"""Dedicated localization/navigation launch; every velocity is remapped away from /cmd_vel."""
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0,str(root/'slam'))
    from laser_mount import load_mount, tf_arguments
    bringup = Path(get_package_share_directory('nav2_bringup'))
    params = str(root/'navigation'/'generated_params.yaml')
    mapping = os.environ.get('ROBOT_NAV_MODE') == 'mapping'
    mapfile = os.environ.get('ROBOT_NAV_MAP', '')
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('rplidar_ros'))/'launch'/'rplidar_a1_launch.py')),
            launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}.items()),
        ExecuteProcess(cmd=['python3', str(root/'slam'/'mpu6050_node.py')], output='screen'),
        ExecuteProcess(cmd=['python3', str(root/'slam'/'laser_imu_odom.py')], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=tf_arguments(load_mount(root/'slam'/'laser_mounting.json'))),
    ]
    if mapping:
        actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', parameters=[str(root/'slam'/'mapping_no_odom.yaml')], output='screen'))
    else:
        actions.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[params, {'yaml_filename': mapfile}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', parameters=[{'autostart': False,
                      'node_names': ['map_server', 'amcl']}]),
            ExecuteProcess(cmd=['python3',str(root/'navigation'/'navigation_startup.py'),
                '--manager','lifecycle_manager_localization','--nodes','map_server','amcl'],output='screen'),
        ])
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
                        name='lifecycle_manager_navigation', parameters=[{'autostart': False,
                        'node_names': names, 'bond_timeout': 4.0}]))
    actions.append(ExecuteProcess(cmd=['python3',str(root/'navigation'/'navigation_startup.py'),
        '--manager','lifecycle_manager_navigation','--nodes',*names],output='screen'))
    return LaunchDescription(actions)
