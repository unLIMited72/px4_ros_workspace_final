# drone_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_ns = LaunchConfiguration('px4_ns')
    drone_id = LaunchConfiguration('drone_id')

    return LaunchDescription([
        # -------------------------
        # Launch Arguments
        # -------------------------
        DeclareLaunchArgument(
            'px4_ns',
            default_value='px4_0',
            description='PX4 namespace (e.g. px4_0, px4_1, px4_2)'
        ),
        DeclareLaunchArgument(
            'drone_id',
            default_value='px4_0',
            description='Drone ID used in GCS messages (e.g. px4_0, px4_1, px4_2)'
        ),

        # -------------------------
        # px4_status_node
        # -------------------------
        Node(
            package='px4_sim_cpp',
            executable='px4_status_node',
            name='px4_status_node',
            output='screen',
            parameters=[{
                'px4_ns': px4_ns,
                'drone_id': drone_id,
            }]
        ),

        # -------------------------
        # gyro_control_node (GyroOffboardController)
        # -------------------------
        Node(
            package='px4_sim_cpp',
            executable='gyro_control_node',
            name='gyro_control_node',
            output='screen',
            parameters=[{
                'px4_ns': px4_ns,
                'drone_id': drone_id,
                # 필요하면 이후에 target_system_id 같은 것도 추가 가능
            }]
        ),

        # -------------------------
        # mission_control_node (DroneOffboardMissionControl)
        # -------------------------
        Node(
            package='px4_sim_cpp',
            executable='mission_control_node',
            name='mission_control_node',
            output='screen',
            parameters=[{
                'px4_ns': px4_ns,
                'drone_id': drone_id,
                # 'cruise_speed_mps': 3.0,
                # 'climb_speed_mps': 1.0,
            }]
        ),
    ])
