from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fmu_prefix = LaunchConfiguration('fmu_prefix')
    active_commander = LaunchConfiguration('active_commander')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fmu_prefix',
            default_value='',
            description='PX4 topic ön eki (tekli İHA için boş bırakın; çoklu için örn: /uav1)',
        ),
        DeclareLaunchArgument(
            'active_commander',
            default_value='',
            description='Yetkili commander_id (boş bırakılırsa tüm MissionCommand emirleri kabul edilir)',
        ),
        Node(
            package='sartek_px4_bridge',
            executable='px4_telemetry_bridge',
            name='px4_telemetry_bridge',
            output='screen',
            parameters=[{'fmu_prefix': fmu_prefix}],
        ),
        Node(
            package='sartek_px4_bridge',
            executable='px4_control_bridge',
            name='px4_control_bridge',
            output='screen',
            parameters=[{
                'fmu_prefix': fmu_prefix,
                'active_commander': active_commander,
            }],
        ),
    ])
