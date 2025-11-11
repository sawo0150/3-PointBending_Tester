import rclpy
from rclpy.node import Node
import cv2
import numpy as np

# ROS 2 메시지 및 서비스 타입
from std_msgs.msg import Float64
from std_srvs.srv import Empty

# [중요] deflection_tracker.py 파일에서 DotTracker 클래스를 임포트합니다.
try:
    from vision_processor.deflection_tracker import DotTracker
except ImportError:
    print("\n[오류] from vision_processor.deflection_tracker import DotTracker 실패.")
    print("    deflection_tracker.py 파일이 deflection_tracker_node.py와 같은 폴더에 있는지,")
    print("    __init__.py 파일이 있는지 확인하세요.\n")
    exit()


class DeflectionTrackerNode(Node):
    def __init__(self):
        super().__init__('deflection_tracker_node')

        # --- ROS 2 파라미터 선언 ---
        self.declare_parameter('cam_index', 2)
        self.declare_parameter('target_width', 640)
        # 런치 파일에서 리스트로 전달: [x, y, w, h]
        self.declare_parameter('roi_rect', [295, 100, 70, 200]) 
        # 런치 파일에서 리스트로 전달: [h, s, v]
        self.declare_parameter('hsv_lower', [0, 0, 0])
        self.declare_parameter('hsv_upper', [180, 255, 70])
        self.declare_parameter('min_area', 50)
        # [핵심] pixel -> mm 변환 비례 상수
        self.declare_parameter('pixel_to_mm_ratio', 0.05) # 예: 1 픽셀 = 0.05 mm

        # --- 파라미터 값 가져오기 ---
        cam_index = self.get_parameter('cam_index').get_parameter_value().integer_value
        self.cam_index = cam_index # [추가] 재연결을 위해 cam_index를 self에 저장
        self.target_width = self.get_parameter('target_width').get_parameter_value().integer_value
        roi_rect = self.get_parameter('roi_rect').get_parameter_value().integer_array_value
        hsv_lower = self.get_parameter('hsv_lower').get_parameter_value().integer_array_value
        hsv_upper = self.get_parameter('hsv_upper').get_parameter_value().integer_array_value
        min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        self.pixel_to_mm_ratio = self.get_parameter('pixel_to_mm_ratio').get_parameter_value().double_value

        self.get_logger().info(f"--- Deflection Tracker Node Started ---")
        self.get_logger().info(f"Cam Index: {cam_index}, Target Width: {self.target_width}")
        self.get_logger().info(f"ROI Rect: {roi_rect}")
        self.get_logger().info(f"Pixel-to-mm Ratio: {self.pixel_to_mm_ratio}")
        self.get_logger().info(f"---------------------------------------")

        # --- DotTracker 클래스 초기화 ---
        self.tracker = DotTracker(roi_rect=tuple(roi_rect),
                                  color_lower=hsv_lower,
                                  color_upper=hsv_upper,
                                  min_contour_area=min_area)

        # --- OpenCV 카메라 초기화 (cv2.VideoCapture) ---
        self.cap = cv2.VideoCapture(self.cam_index) # [수정] self.cam_index 사용
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {cam_index}")
            rclpy.shutdown()
            return

        # --- 처짐(deflection) 계산을 위한 상태 변수 ---
        self.zero_pixel_y = None          # 0점 조절 시의 Y 픽셀 위치
        self.last_tracked_pixel_y = None  # 가장 최근에 추적된 Y 픽셀 위치
        self.current_deflection_mm = 0.0  # 현재 처짐 (mm)

        # --- ROS 2 퍼블리셔 & 서비스 ---
        # 1. /deflection 토픽 (mm 단위)
        self.deflection_pub = self.create_publisher(Float64, '/deflection', 10)
        
        # 2. /tare_deflection 서비스 (0점 조절)
        self.tare_service = self.create_service(Empty, '/tare_deflection', self.tare_deflection_callback)

        # --- 메인 루프 타이머 ---
        # 30Hz (1/30초)마다 timer_callback 함수를 실행
        timer_period = 1.0 / 30.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node setup complete. Starting camera loop...")

    def timer_callback(self):
        """ 메인 루프. 카메라 프레임 읽기, 추적, 발행, 시각화를 수행 """
        
        ret, frame = self.cap.read()

        # --- [수정] Fallback 로직 ---

        if not ret:
            self.get_logger().warn("Failed to read frame. Attempting to reconnect camera...")
            try:
                # 1. 기존 캡처 객체 해제
                self.cap.release()
            except Exception as e:
                self.get_logger().warn(f"Error releasing old camera capture: {e}")
            
            # 2. 카메라 재연결 시도
            self.cap = cv2.VideoCapture(self.cam_index)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to reconnect camera. Will retry next tick.")
                return # 이번 프레임은 포기, 다음 콜백에서 다시 시도
            else:
                self.get_logger().info("Successfully reconnected to camera.")
                # 재연결 후 첫 프레임 읽기
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("Failed to read frame even after reconnect. Skipping frame.")
                    return
        # --- Fallback 로직 끝 ---

        # 1. 프레임 리사이즈
        original_h, original_w = frame.shape[:2]
        ratio = self.target_width / float(original_w)
        target_height = int(original_h * ratio)
        resized_frame = cv2.resize(frame, (self.target_width, target_height))

        # 2. DotTracker로 점 추적 (디버그 모드 켜기)
        tracked_position, debug_imgs, current_area = self.tracker.track(resized_frame, debug_viz=True)

        # 3. 처짐(deflection) 계산 및 발행
        if tracked_position:
            self.last_tracked_pixel_y = tracked_position[1] # 현재 Y픽셀 저장

            # 0점이 아직 설정되지 않았다면, 첫 추적 위치를 0점으로 설정
            if self.zero_pixel_y is None:
                self.zero_pixel_y = self.last_tracked_pixel_y
                self.get_logger().info(f"Initial deflection zero point set to Y={self.zero_pixel_y} pixels.")
            
            # (현재 Y픽셀 - 0점 Y픽셀) * 비례상수
            # (OpenCV는 Y가 아래로 갈수록 증가하므로, (현재 - 0점)이 맞습니다)
            deflection_pixels = self.last_tracked_pixel_y - self.zero_pixel_y
            self.current_deflection_mm = deflection_pixels * self.pixel_to_mm_ratio

            # Float64 메시지로 /deflection 토픽 발행
            msg = Float64()
            msg.data = self.current_deflection_mm
            self.deflection_pub.publish(msg)

        # 4. 시각화 (디버그)
        # 4-1. 메인 창 (추적 결과 + 텍스트)
        self.tracker.draw_visualization(resized_frame, tracked_position)
        
        # 텍스트 추가: 좌표, 면적, 처짐(mm)
        if tracked_position:
            pos_text = f"Pos: {tracked_position[0]}, {tracked_position[1]}"
            cv2.putText(resized_frame, pos_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        area_text = f"Area: {current_area:.0f} (Min: {self.tracker.min_area})"
        cv2.putText(resized_frame, area_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        deflection_text = f"Deflection: {self.current_deflection_mm:.3f} mm"
        cv2.putText(resized_frame, deflection_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Deflection Tracker Node", resized_frame)

        # 4-2. 디버그 창 (ROI, Mask) - 2배 확대
        if debug_imgs:
            if debug_imgs['ROI_Debug'] is not None:
                h, w = debug_imgs['ROI_Debug'].shape[:2]
                resized_roi_debug = cv2.resize(debug_imgs['ROI_Debug'], (w*2, h*2), interpolation=cv2.INTER_NEAREST)
                cv2.imshow("ROI Debug (Contours) - 2x", resized_roi_debug)
            if debug_imgs['Mask'] is not None:
                h, w = debug_imgs['Mask'].shape[:2]
                resized_mask = cv2.resize(debug_imgs['Mask'], (w*2, h*2), interpolation=cv2.INTER_NEAREST)
                cv2.imshow("Mask (Thresholded) - 2x", resized_mask)
        
        # [필수] cv2.imshow()가 작동하려면 waitKey가 필요합니다.
        cv2.waitKey(1) 

    def tare_deflection_callback(self, request, response):
        """ /tare_deflection 서비스가 호출되었을 때 실행되는 콜백 """
        if self.last_tracked_pixel_y is not None:
            # 현재 추적된 Y픽셀 위치를 새로운 0점으로 설정
            self.zero_pixel_y = self.last_tracked_pixel_y
            self.get_logger().info(f"Deflection zero point RE-CALIBRATED to Y={self.zero_pixel_y} pixels.")
        else:
            self.get_logger().warn("Cannot tare deflection: No dot is currently being tracked.")
        
        return response # Empty.srv는 응답 내용이 없습니다.

    def destroy_node(self):
        """ 노드 종료 시 카메라와 cv2 창 해제 """
        self.get_logger().info("Shutting down node...")
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DeflectionTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()