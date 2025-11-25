# gcs_core.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # PX4 인스턴스 관리 (SITL 여러 개 스폰)
        Node(
            package='px4_sim_py',
            executable='px4_instance_manager',
            name='px4_instance_manager',
            output='screen'
        ),

        # PX4 상태 통합 → /gcs/ui_status 등으로 보내는 노드
        Node(
            package='px4_sim_py',
            executable='px4_status_aggregator',
            name='px4_status_aggregator',
            output='screen'
        ),

        # React GCS <-> ROS2 미션/상태 게이트웨이
        Node(
            package='px4_sim_py',
            executable='gcs_mission_gateway',
            name='gcs_mission_gateway',
            output='screen'
        ),

        # 멀티 드론 미션 플래너
        Node(
            package='px4_sim_cpp',
            executable='mission_planner_node',
            name='mission_planner_node',
            output='screen'
        ),

        # Gyro 제어 RAW JSON → GyroControl msg 변환 게이트웨이
        Node(
            package='px4_sim_cpp',
            executable='gcs_gyro_gateway_node',
            name='gcs_gyro_gateway_node',
            output='screen'
        ),
    ])
