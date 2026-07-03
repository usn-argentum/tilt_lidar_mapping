from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sweep_amplitude_deg', default_value='22.5'),
        DeclareLaunchArgument('sweep_period_s', default_value='2.0'),
        DeclareLaunchArgument('simulate', default_value='false'),

        Node(
            package='tilt_lidar_mapping',
            executable='servo_sweep_node',
            name='tilt_servo_node',
            output='screen',
            parameters=[{
                'sweep_amplitude_deg': LaunchConfiguration('sweep_amplitude_deg'),
                'sweep_period_s': LaunchConfiguration('sweep_period_s'),
                'simulate': LaunchConfiguration('simulate'),
            }],
        ),
        Node(
            package='tilt_lidar_mapping',
            executable='scan_to_cloud_node',
            name='tilt_scan_to_cloud',
            output='screen',
        ),
        # NOTE: LDS-02 driver node is NOT launched here - bring it up separately
        # (e.g. ldlidar_stl_ros2 or whatever driver publishes /scan for your unit).
        # This launch file assumes /scan already exists.
    ])
