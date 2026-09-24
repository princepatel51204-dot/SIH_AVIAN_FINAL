"""3D perception for Gate 2: gz point cloud -> ROS -> Octomap only.

Deliberately does NOT flatten the cloud to a second /scan (that topic is
already served by gz_bridge's native 2D lidar_2d_v2 bridge, which SLAM
Toolbox is tuned against). Octomap instead projects its live 3D voxel map
into a 2D layer for Nav2 (see nav2_sih.yaml octomap_layer), bounded to a
band around cruise altitude so the whole vertical column under a pier
doesn't read as one solid blocked footprint.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    sensor = LaunchConfiguration('sensor')
    occ_min_z = LaunchConfiguration('occupancy_min_z')
    occ_max_z = LaunchConfiguration('occupancy_max_z')

    gz_points = ['/world/', world, '/model/', model,
                 '/link/link/sensor/', sensor, '/scan/points']

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gate2_points_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[[*gz_points, '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']],
        remappings=[(gz_points, '/points_raw')],
    )

    octomap = Node(
        package='octomap_server', executable='octomap_server_node',
        name='gate2_octomap', output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'map',
            'base_frame_id': 'base_footprint',
            'resolution': 0.2,
            'sensor_model.max_range': 25.0,
            'filter_ground_plane': False,
            'occupancy_min_z': occ_min_z,
            'occupancy_max_z': occ_max_z,
        }],
        remappings=[('cloud_in', '/points_raw')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sih_avian_final'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0'),
        DeclareLaunchArgument('sensor', default_value='garudanex_lidar_3d'),
        DeclareLaunchArgument('occupancy_min_z', default_value='5.0'),
        DeclareLaunchArgument('occupancy_max_z', default_value='11.0'),
        bridge, octomap,
    ])
