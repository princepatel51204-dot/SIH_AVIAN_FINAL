"""Gate 2 Nav2 bringup: nav2_bringup's navigation_launch.py, trimmed to
exactly what smart_explorer uses (navigate_to_pose, compute_path_to_pose),
minus route_server/collision_monitor/docking_server -- none of which this
mission needs. collision_monitor in particular has an independent bug (its
own footprint-approach transform lookup gets stuck extrapolating to a fixed
future timestamp, e.g. "requested time 314.74 vs latest 76.8", reproduced
identically across multiple clean restarts) that hangs the whole
lifecycle_manager activation GROUP, since it manages all nodes as one unit.
Obstacle avoidance already comes from the controller's own MPPI CostCritic
against the live costmap; collision_monitor is a redundant safety layer we
don't need for this mission and can't currently get past.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file, root_key='',
            param_rewrites={'autostart': autostart}, convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        SetParameter('use_sim_time', use_sim_time),

        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level],
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level], remappings=remappings),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level], remappings=remappings),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level],
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level], remappings=remappings),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level], remappings=remappings),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[configured_params],
             arguments=['--ros-args', '--log-level', log_level],
             # ROOT CAUSE of every prior Gate 2 attempt's 0 goals reached:
             # velocity_smoother subscribes on the generic name 'cmd_vel'
             # (remapped below to controller_server's real output,
             # 'cmd_vel_nav') but PUBLISHES on its own distinct hardcoded
             # name 'cmd_vel_smoothed' -- untouched by that remap. Nothing
             # was listening to 'cmd_vel_smoothed'; GarudaNEX's
             # cmd_vel_bridge subscribes to plain '/cmd_vel'. MPPI was
             # computing real, correct velocities (verified: up to 0.46 m/s
             # on /cmd_vel_nav) that never reached the vehicle -- confirmed
             # by comparing to real TF position over a controlled 15 s
             # window: 0.094 m net displacement while /cmd_vel_nav showed
             # sustained ~0.3-0.4 m/s. Not CPU (load ~6-9 of 16 cores
             # throughout), not MPPI tuning, not a distance/timeout issue.
             remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                      ('cmd_vel_smoothed', '/cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             arguments=['--ros-args', '--log-level', log_level],
             parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}]),
    ])
