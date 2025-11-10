import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # 1. hardware_interface 패키지의 런치 파일 경로 찾기
    # [주의] 로그에 적어주신 'serial_node_pump.launch.py'를 사용합니다.
    #      이 파일이 'hardware_interface/launch/'에 있는지 확인하세요.
    hardware_interface_pkg = get_package_share_directory('hardware_interface')
    hardware_launch_file = os.path.join(
        hardware_interface_pkg,
        'launch',
        'serial_node_pump.launch.py' # <- 이 파일명을 사용합니다
    )

    # 2. vision_processor 패키지의 런치 파일 경로 찾기
    vision_processor_pkg = get_package_share_directory('vision_processor')
    vision_launch_file = os.path.join(
        vision_processor_pkg,
        'launch',
        'tracker.launch.py'
    )

    # 3. experiment_manager 패키지의 런치 파일 경로 찾기
    experiment_manager_pkg = get_package_share_directory('experiment_manager')
    manager_launch_file = os.path.join(
        experiment_manager_pkg,
        'launch',
        'manager.launch.py'
    )

    return LaunchDescription([
        # 1. 하드웨어 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch_file)
        ),
        
        # 2. 비전 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch_file)
        ),
        
        # 3. 매니저 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manager_launch_file)
        )
    ])