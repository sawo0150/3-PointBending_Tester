import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 파라미터 파일 경로 찾기
    config_file = os.path.join(
        get_package_share_directory('experiment_manager'),
        'config',
        'specimen_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='experiment_manager',
            executable='experiment_manager_node',
            name='experiment_manager_node',
            output='screen',
            emulate_tty=True,
            # 2. 파라미터 파일 로드
            parameters=[config_file]
        )
    ])