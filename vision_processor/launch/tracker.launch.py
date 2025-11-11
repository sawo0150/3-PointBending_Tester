import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 여기서 모든 설정값을 관리합니다.
    cam_index = 2
    target_width = 640
    roi_rect = [295, 100, 70, 200]  # [x, y, w, h]
    hsv_lower = [0, 0, 0]          # [h, s, v]
    hsv_upper = [180, 255, 70]     # [h, s, v]
    min_area = 50
    
    # [중요] 픽셀-밀리미터 변환 상수 (보정 필요)
    # 예: 카메라 앞에서 1mm 자를 대고 몇 픽셀인지 확인
    # 1mm에 20 픽셀이라면 -> 1 / 20 = 0.05
    pixel_to_mm_ratio = 0.14534883

    return LaunchDescription([
        Node(
            package='vision_processor',
            executable='deflection_tracker_node',
            name='deflection_tracker_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'cam_index': cam_index},
                {'target_width': target_width},
                {'roi_rect': roi_rect},
                {'hsv_lower': hsv_lower},
                {'hsv_upper': hsv_upper},
                {'min_area': min_area},
                {'pixel_to_mm_ratio': pixel_to_mm_ratio}
            ]
        )
    ])