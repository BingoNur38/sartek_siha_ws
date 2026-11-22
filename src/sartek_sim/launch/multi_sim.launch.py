#!/usr/bin/env python3
"""
SARTEK Savaşan İHA — 5 Sanal + 1 Kameralı (6 İHA) Çoklu Simülasyon Launch Dosyası
==================================================================================
Tamamen Micro XRCE-DDS (PX4 v1.18 + Gazebo Harmonic) tabanlıdır; MAVROS kullanmaz.

Uçak Yerleşimi ve İsim Uzayları:
  • Instance 0 → gz_advanced_plane_cam  (Kendi kameralı uçağımız)
      - DDS Topic'leri : /fmu/out/* , /fmu/in/*  (sartek_px4_bridge ve QGC ile uyumlu)
      - Kamera Topic   : /camera/image_raw
      - Konum          : (0, 0, 0.30) — ana pist ortası (bağımsız kontrol)
  • Instance 1..5 → gz_advanced_plane_1 .. gz_advanced_plane_5 (5 sanal rakip İHA)
      - DDS Topic'leri : /gz_advanced_plane_1/fmu/* .. /gz_advanced_plane_5/fmu/*
      - Konumlar       : Piste paralel yanyana 25m aralıkla (çarpışmasız kalkış)
      - Kontrol        : target_offboard düğümleri ile otonom kalkış + rastgele waypoint

Kullanım:
  ros2 launch sartek_sim multi_sim.launch.py
  ros2 launch sartek_sim multi_sim.launch.py world_name:=sartek_yarisma headless:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _join_paths(*entries: str) -> str:
    return ':'.join([p for p in entries if p])


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('sartek_sim')

    px4_dir = os.path.expanduser(LaunchConfiguration('px4_dir').perform(context))
    world_name = LaunchConfiguration('world_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    agent_port = LaunchConfiguration('agent_port').perform(context)
    start_targets = LaunchConfiguration('start_targets').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )

    sim_models = os.path.join(pkg_share, 'models')
    sim_worlds = os.path.join(pkg_share, 'worlds')
    world_sdf = os.path.join(sim_worlds, f'{world_name}.sdf')
    cam_model_sdf = os.path.join(sim_models, 'advanced_plane_cam', 'model.sdf')

    px4_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    px4_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    target_model_sdf = os.path.join(px4_models, 'advanced_plane', 'model.sdf')

    px4_build = os.path.join(px4_dir, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(px4_build, 'bin', 'px4')
    px4_plugins = os.path.join(px4_build, 'src', 'modules', 'simulation', 'gz_plugins')
    px4_server_config = os.path.join(
        px4_dir, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config'
    )

    for path_label, path_val in [
        ('Gazebo dünya dosyası', world_sdf),
        ('Kameralı uçak SDF', cam_model_sdf),
        ('PX4 advanced_plane SDF', target_model_sdf),
        ('PX4 SITL binary', px4_bin),
    ]:
        if not os.path.isfile(path_val):
            raise RuntimeError(f'{path_label} bulunamadı: {path_val}')

    gz_resource_path = _join_paths(
        sim_models,
        sim_worlds,
        px4_models,
        px4_worlds,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )
    gz_plugin_path = _join_paths(
        px4_plugins,
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    )

    common_gz_env = {
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GZ_SIM_SERVER_CONFIG_PATH': px4_server_config,
        'GZ_IP': '127.0.0.1',
    }

    # 1) Gazebo Harmonic
    gz_cmd = ['gz', 'sim', '-r']
    if headless:
        gz_cmd.append('-s')
    gz_cmd.append(world_sdf)

    gazebo_proc = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env=common_gz_env,
    )

    # 2) Tek MicroXRCEAgent — 6 PX4 instance'ın tamamı UDP:8888 üzerinden bağlanır
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_agent')),
    )

    actions = [
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=gz_plugin_path),
        SetEnvironmentVariable(name='GZ_SIM_SERVER_CONFIG_PATH', value=px4_server_config),
        SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1'),
        LogInfo(
            msg=(
                f'=== SARTEK 6 İHA Simülasyonu Başlatılıyor | '
                f'Dünya: {world_name}.sdf ==='
            )
        ),
        gazebo_proc,
        TimerAction(period=1.5, actions=[xrce_agent]),
    ]

    # 3) 6. Uçak — Kendi Kameralı Uçağımız (instance=0, bağımsız kontrol)
    cam_entity_name = 'gz_advanced_plane_cam'
    cam_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gz_advanced_plane_cam',
        output='screen',
        arguments=[
            '-world', world_name,
            '-file', cam_model_sdf,
            '-name', cam_entity_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.30',
            '-Y', '0.0',
        ],
    )

    cam_px4_env = dict(common_gz_env)
    cam_px4_env.update(
        {
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '4008',
            'PX4_SIM_MODEL': 'advanced_plane',
            'PX4_GZ_WORLD': world_name,
            'PX4_GZ_MODEL_NAME': cam_entity_name,
            'PX4_GZ_NO_FOLLOW': '1',
            'PX4_UXRCE_DDS_NS': '',
            'PX4_UXRCE_DDS_PORT': agent_port,
            # Hem QGC hem de sadece ROS 2 ile kontrol edilebilsin:
            'PX4_PARAM_NAV_DLL_ACT': '0',
            'PX4_PARAM_COM_RCL_EXCEPT': '4',
            'PX4_PARAM_COM_OF_LOSS_T': '10.0',
            'PX4_PARAM_SIM_BAT_DRAIN': '0',
            'PX4_PARAM_COM_LOW_BAT_ACT': '0',
        }
    )

    cam_px4 = ExecuteProcess(
        cmd=[px4_bin, '-i', '0', '-d'],
        output='screen',
        name='px4_gz_advanced_plane_cam',
        additional_env=cam_px4_env,
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='front_camera_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_bridge')),
        arguments=['/front_camera'],
        remappings=[('/front_camera', '/camera/image_raw')],
    )

    actions.append(
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg='[T=4s] Kendi kameralı uçağımız spawn ediliyor: gz_advanced_plane_cam'),
                cam_spawn,
            ],
        )
    )
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=cam_spawn,
                on_exit=[cam_px4, camera_bridge],
            )
        )
    )

    # 4) 5 Adet Sanal Sabit Kanatlı İHA (gz_advanced_plane_1 .. gz_advanced_plane_5)
    #    (name, px4_instance, spawn_x, spawn_y, spawn_delay_s, cruise_alt_m)
    target_planes = [
        ('gz_advanced_plane_1', 1, '0.0', '25.0', 7.0, 55.0),
        ('gz_advanced_plane_2', 2, '0.0', '50.0', 10.0, 70.0),
        ('gz_advanced_plane_3', 3, '0.0', '75.0', 13.0, 85.0),
        ('gz_advanced_plane_4', 4, '0.0', '-25.0', 16.0, 100.0),
        ('gz_advanced_plane_5', 5, '0.0', '-50.0', 19.0, 115.0),
    ]

    for idx, (name, instance, sx, sy, spawn_delay, cruise_alt) in enumerate(target_planes):
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            output='screen',
            arguments=[
                '-world', world_name,
                '-file', target_model_sdf,
                '-name', name,
                '-x', sx,
                '-y', sy,
                '-z', '0.30',
                '-Y', '0.0',
            ],
        )

        px4_env = dict(common_gz_env)
        px4_env.update(
            {
                'PX4_GZ_STANDALONE': '1',
                'PX4_SYS_AUTOSTART': '4008',
                'PX4_SIM_MODEL': 'advanced_plane',
                'PX4_GZ_WORLD': world_name,
                'PX4_GZ_MODEL_NAME': name,
                'PX4_GZ_NO_FOLLOW': '1',
                'PX4_UXRCE_DDS_NS': name,
                'PX4_UXRCE_DDS_PORT': agent_port,
                # GCS/RC olmadan tam otonom kalkış ve offboard uçuş izni:
                'PX4_PARAM_NAV_DLL_ACT': '0',
                'PX4_PARAM_COM_RCL_EXCEPT': '4',
                'PX4_PARAM_COM_OF_LOSS_T': '10.0',
                'PX4_PARAM_SIM_BAT_DRAIN': '0',
                'PX4_PARAM_COM_LOW_BAT_ACT': '0',
            }
        )

        px4_proc = ExecuteProcess(
            cmd=[px4_bin, '-i', str(instance), '-d'],
            output='screen',
            name=f'px4_{name}',
            additional_env=px4_env,
        )

        actions.append(
            TimerAction(
                period=spawn_delay,
                actions=[
                    LogInfo(
                        msg=f'[T={spawn_delay:.0f}s] Sanal İHA spawn ediliyor: '
                        f'{name} (instance={instance}, pos=({sx}, {sy}))'
                    ),
                    spawn_node,
                ],
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_node,
                    on_exit=[px4_proc],
                )
            )
        )

        # Otonom kalkış + rastgele waypoint düğümü
        if start_targets:
            ctrl_delay = 28.0 + (idx * 2.0)
            target_ctrl = Node(
                package='sartek_sim',
                executable='target_offboard',
                name=f'target_offboard_{name}',
                output='screen',
                parameters=[
                    {
                        'namespace': name,
                        'target_system': instance + 1,
                        'takeoff_altitude': 45.0,
                        'cruise_altitude': cruise_alt,
                        'waypoint_threshold': 45.0,
                        'seed': 100 + instance,
                    }
                ],
            )
            actions.append(
                TimerAction(
                    period=ctrl_delay,
                    actions=[
                        LogInfo(
                            msg=f'[T={ctrl_delay:.0f}s] Otonom uçuş başlatılıyor: '
                            f'{name} (seyir irtifası {cruise_alt:.0f}m)'
                        ),
                        target_ctrl,
                    ],
                )
            )

    return actions


def generate_launch_description():
    default_px4 = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'px4_dir',
                default_value=default_px4,
                description='PX4-Autopilot dizini',
            ),
            DeclareLaunchArgument(
                'world_name',
                default_value='sartek_yarisma',
                description='sartek_sim/worlds altındaki .sdf dünya adı (uzantısız)',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='true ise Gazebo GUI açılmadan (-s) çalışır',
            ),
            DeclareLaunchArgument(
                'start_agent',
                default_value='true',
                description='MicroXRCEAgent udp4 oturumunu başlat',
            ),
            DeclareLaunchArgument(
                'start_bridge',
                default_value='true',
                description='Kameralı uçak için /front_camera -> /camera/image_raw köprüsü',
            ),
            DeclareLaunchArgument(
                'start_targets',
                default_value='true',
                description='5 sanal uçak için otonom kalkış ve rastgele waypoint takibini başlat',
            ),
            DeclareLaunchArgument(
                'agent_port',
                default_value='8888',
                description='MicroXRCEAgent UDP portu',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
