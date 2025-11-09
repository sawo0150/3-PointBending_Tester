from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_interface',
            executable='arduino_serial_node',
            name='arduino_serial_node',
            output='screen', # 터미널에 로그 출력
            emulate_tty=True, # 로그가 즉시 보이도록 설정
            parameters=[
                # 아두이노 포트 확인
                {'port': '/dev/ttyACM0'}, 
                {'baud_rate': 115200},
                
                # [!!! 중요 !!!] 이 값을 보정해야 합니다.
                # 아래 'calibration_factor 찾는 법' 참고
                {'calibration_factor': 0.0000185} # 예시 값
            ]
        )
    ])