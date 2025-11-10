import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import pygame
import threading
import os
import csv
import cv2 # [추가] OpenCV 임포트
from datetime import datetime

# ROS 2 메시지 및 서비스 타입
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty

# 계산용
import numpy as np

class ExperimentManagerNode(Node):
    def __init__(self):
        super().__init__('experiment_manager_node')

        # --- 1. 파라미터 선언 및 가져오기 (단위: mm) ---
        self.declare_parameter('specimen_b', 12.7)
        self.declare_parameter('specimen_h', 2.8)
        self.declare_parameter('span_l', 100.0)
        self.declare_parameter('csv_save_directory', '~/bending_test_data')

        self.b_mm = self.get_parameter('specimen_b').get_parameter_value().double_value
        self.h_mm = self.get_parameter('specimen_h').get_parameter_value().double_value
        self.l_mm = self.get_parameter('span_l').get_parameter_value().double_value
        
        # CSV 저장 경로 처리 ('~'를 실제 홈 디렉토리로 확장)
        self.csv_save_path = os.path.expanduser(
            self.get_parameter('csv_save_directory').get_parameter_value().string_value
        )
        
        # --- 2. 계산을 위한 SI 단위 변환 (m) ---
        self.b_m = self.b_mm / 1000.0
        self.h_m = self.h_mm / 1000.0
        self.l_m = self.l_mm / 1000.0
        
        self.get_logger().info(f"Specimen Params (b, h, L): {self.b_mm}mm, {self.h_mm}mm, {self.l_mm}mm")
        self.get_logger().info(f"Data will be saved to: {self.csv_save_path}")

        # --- 3. 노드 상태 변수 ---
        self.experiment_running = False
        self.data_log = [] # (time_s, force_N, deflection_mm, stress_MPa, strain)
        self.start_time = None
        
        # 실시간 데이터 저장을 위한 변수 (스레드간 공유)
        self.current_force_N = 0.0
        self.current_deflection_mm = 0.0
        self.current_stress_MPa = 0.0
        self.current_strain = 0.0
        self.data_lock = threading.Lock() # 스레드 안전을 위한 잠금

        # --- 4. ROS 2 구독자 (데이터 수신) ---
        self.force_sub = self.create_subscription(
            Float64, '/force', self.force_callback, 10
        )
        self.deflection_sub = self.create_subscription(
            Float64, '/deflection', self.deflection_callback, 10
        )

        # --- 5. ROS 2 퍼블리셔 (명령 및 시각화) ---
        self.pump_pub = self.create_publisher(Bool, '/pump_control', 10)
        # [실시간 그래프용] rqt_plot에서 볼 수 있도록 Stress, Strain 발행
        self.stress_pub = self.create_publisher(Float64, '/stress', 10)
        self.strain_pub = self.create_publisher(Float64, '/strain', 10)

        # --- 6. ROS 2 서비스 클라이언트 (영점 조절) ---
        self.tare_force_client = self.create_client(Empty, '/tare_loadcell')
        self.tare_deflection_client = self.create_client(Empty, '/tare_deflection')
        
        # 서비스 서버가 준비될 때까지 대기
        while not self.tare_force_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/tare_loadcell service not available, waiting...')
        while not self.tare_deflection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/tare_deflection service not available, waiting...')
        self.get_logger().info("All required services are connected.")

    # --- 7. 콜백 함수 (데이터 업데이트) ---
    def force_callback(self, msg):
        with self.data_lock:
            self.current_force_N = msg.data

    def deflection_callback(self, msg):
        with self.data_lock:
            self.current_deflection_mm = msg.data

    # --- 8. 핵심 계산 및 로깅 루프 ---
    def calculation_loop(self):
        """ 10Hz 타이머로 Stress, Strain을 계산하고 로깅하는 함수 """
        if not self.experiment_running:
            return

        with self.data_lock:
            F = self.current_force_N
            delta_mm = self.current_deflection_mm
        
        delta_m = delta_mm / 1000.0

        # [계산] Stress (Stress = 3FL / 2bh^2) (단위: Pa)
        # 분모가 0이 되는 것 방지
        denominator_stress = 2 * self.b_m * (self.h_m ** 2)
        if denominator_stress == 0:
            stress_Pa = 0.0
        else:
            stress_Pa = (3 * F * self.l_m) / denominator_stress
        
        # Pa -> MPa (MegaPascals)
        stress_MPa = stress_Pa / 1e6

        # [계산] Strain (Strain = 6 * delta * h / L^2) (단위: 없음)
        denominator_strain = self.l_m ** 2
        if denominator_strain == 0:
            strain = 0.0
        else:
            strain = (6 * delta_m * self.h_m) / denominator_strain

        # [rqt_plot용] 토픽 발행
        self.stress_pub.publish(Float64(data=stress_MPa))
        self.strain_pub.publish(Float64(data=strain))

        # [데이터 저장]
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        with self.data_lock:
            self.current_stress_MPa = stress_MPa
            self.current_strain = strain
            self.data_log.append((
                current_time, 
                F, 
                delta_mm, 
                stress_MPa, 
                strain
            ))

    # --- 9. 실험 제어 함수 ---
    def start_experiment(self):
        if self.experiment_running:
            return
        
        self.experiment_running = True
        self.data_log = [] # 데이터 로그 초기화
        self.start_time = self.get_clock().now()
        self.pump_pub.publish(Bool(data=True)) # 펌프 작동
        self.get_logger().info("===== EXPERIMENT STARTED =====")

    def stop_experiment(self):
        if not self.experiment_running:
            return
            
        self.experiment_running = False
        self.pump_pub.publish(Bool(data=False)) # 펌프 중지
        self.get_logger().info("===== EXPERIMENT STOPPED =====")
        self.save_csv()
        
    def save_csv(self):
        # 1. 저장 디렉토리 생성 (없으면)
        os.makedirs(self.csv_save_path, exist_ok=True)
        
        # 2. 파일명 생성을 위한 타임스탬프 (CSV와 PNG가 공유)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # --- 3. CSV 파일 작성 ---
        filename = os.path.join(self.csv_save_path, f"bending_test_{timestamp}.csv")
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # 헤더
                writer.writerow([
                    "Time (s)", 
                    "Force (N)", 
                    "Deflection (mm)", 
                    "Stress (MPa)", 
                    "Strain"
                ])
                # 데이터
                writer.writerows(self.data_log)
            self.get_logger().info(f"Data successfully saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV file: {e}")

        # --- 4. [추가] 플롯 이미지 저장 ---
        try:
            # 1. 캔버스 생성 (show_window=False로 화면 표시 안함)
            canvas = self.draw_plot(show_window=False) 
            
            # 2. 이미지 파일명 생성 (CSV와 동일한 타임스탬프)
            img_filename = os.path.join(self.csv_save_path, f"bending_test_{timestamp}.png")
            
            # 3. 이미지 저장
            cv2.imwrite(img_filename, canvas)
            self.get_logger().info(f"Plot image successfully saved to {img_filename}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to save plot image: {e}")

    # --- 10. 서비스 호출 함수 ---
    def call_tare_force(self):
        self.get_logger().info("Requesting Load Cell Tare (0점 조절)...")
        self.tare_force_client.call_async(Empty.Request())

    def call_tare_deflection(self):
        self.get_logger().info("Requesting Deflection Zero (0점 조절)...")
        self.tare_deflection_client.call_async(Empty.Request())

    # --- [수정] OpenCV 플로팅 함수를 클래스 메소드로 이동 ---
    def draw_plot(self, show_window=True):
        """ OpenCV를 사용해 Stress-Strain 커브를 실시간으로 그립니다. """
        PLOT_W, PLOT_H = 500, 500
        MARGIN = 60 # 상하좌우 여백

        # 1. 흰색 캔버스 생성
        canvas = np.full((PLOT_H, PLOT_W, 3), 255, dtype=np.uint8)
        
        # 2. 데이터 잠금 및 복사
        with self.data_lock: # [수정] node. -> self.
            data = list(self.data_log) # [수정] node. -> self.

       # 3. 데이터가 없으면 빈 축만 그림
        plot_w_inner = PLOT_W - 2 * MARGIN
        plot_h_inner = PLOT_H - 2 * MARGIN
        
        # 축 그리기 (X: Strain, Y: Stress)
        cv2.rectangle(canvas, (MARGIN, MARGIN), (PLOT_W - MARGIN, PLOT_H - MARGIN), (0,0,0), 1)
        cv2.putText(canvas, "Strain", (PLOT_W // 2 - 30, PLOT_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
        cv2.putText(canvas, "Stress (MPa)", (5, PLOT_H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)

        if not data:
            if show_window: # [추가] show_window 플래그 확인
                cv2.imshow("Stress-Strain Curve", canvas)
            return canvas # [수정] 캔버스 반환

        # 4. 데이터 스케일링을 위한 최대값 찾기
        strains = [d[4] for d in data]
        stresses = [d[3] for d in data]
        
        max_strain = max(strains) if strains else 1e-6
        max_stress = max(stresses) if stresses else 1e-6
        if max_strain == 0: max_strain = 1e-6
        if max_stress == 0: max_stress = 1e-6

        # 5. 축 레이블 (최대값) 추가
        cv2.putText(canvas, f"{max_strain:.5f}", (PLOT_W - MARGIN - 10, PLOT_H - MARGIN + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
        cv2.putText(canvas, f"{max_stress:.2f}", (MARGIN - 50, MARGIN + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        # 6. 데이터 -> 픽셀 변환 함수
        def to_pixel(s_e, s_s):
            # s_e = strain (x-axis), s_s = stress (y-axis)
            px = int(MARGIN + (s_e / max_strain) * plot_w_inner)
            py = int((PLOT_H - MARGIN) - (s_s / max_stress) * plot_h_inner) # Y-axis is inverted
            return (px, py)

        # 7. 그래프 그리기 (연속선)
        if len(data) > 1:
            pt1 = to_pixel(data[0][4], data[0][3])
            for i in range(1, len(data)):
                pt2 = to_pixel(data[i][4], data[i][3])
                cv2.line(canvas, pt1, pt2, (255, 0, 0), 2) # 파란색 선
                pt1 = pt2

        # 8. 창 표시
        if show_window: # [추가] show_window 플래그 확인
            cv2.imshow("Stress-Strain Curve", canvas)
        return canvas # [수정] 캔버스 반환


# --- Pygame GUI 및 메인 실행 로직 ---

def draw_gui(screen, font, node_state):
    """ Pygame 화면을 그리는 함수 """
    screen.fill((0, 0, 0)) # 검은색 배경
    
    # 폰트 색상
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    CYAN = (0, 255, 255)

    # 1. 상태
    status_text = "EXPERIMENT RUNNING" if node_state['running'] else "IDLE"
    status_color = GREEN if node_state['running'] else RED
    text = font.render(f"[STATUS] {status_text}", True, status_color)
    screen.blit(text, (20, 20))

    # 2. 실시간 데이터
    text = font.render(f"Force (N):       {node_state['F']:.3f}", True, WHITE)
    screen.blit(text, (20, 60))
    text = font.render(f"Deflection (mm): {node_state['d']:.3f}", True, WHITE)
    screen.blit(text, (20, 90))
    text = font.render(f"Stress (MPa):    {node_state['S']:.3f}", True, CYAN)
    screen.blit(text, (20, 120))
    text = font.render(f"Strain:          {node_state['E']:.6f}", True, CYAN)
    screen.blit(text, (20, 150))

    # 3. 조작법
    y_offset = 200
    text = font.render("--- Controls ---", True, WHITE)
    screen.blit(text, (20, y_offset))
    text = font.render("[T] : Tare Load Cell (Force Zero)", True, WHITE)
    screen.blit(text, (20, y_offset + 30))
    text = font.render("[Z] : Zero Deflection (Vision Zero)", True, WHITE)
    screen.blit(text, (20, y_offset + 60))
    text = font.render("[S] : START Experiment (Start Pump)", True, GREEN)
    screen.blit(text, (20, y_offset + 90))
    text = font.render("[Q] : STOP Experiment (Stop Pump & Save)", True, RED)
    screen.blit(text, (20, y_offset + 120))
    text = font.render("[ESC] : Exit Program", True, WHITE)
    screen.blit(text, (20, y_offset + 150))
    
    pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    
    # --- 1. ROS 2 노드 및 Executor 초기화 ---
    # Pygame(메인스레드)과 ROS 콜백(별도스레드)을 함께 사용하기 위해
    # MultiThreadedExecutor를 사용합니다.
    executor = MultiThreadedExecutor()
    node = ExperimentManagerNode()
    executor.add_node(node)

    # ROS 2 스핀을 별도의 스레드에서 실행
    ros_thread = threading.Thread(target=executor.spin)
    ros_thread.start()

    # --- 2. Pygame 초기화 (메인 스레드에서 실행) ---
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((500, 400))
    pygame.display.set_caption("Experiment Manager")
    font = pygame.font.SysFont('Consolas', 20) # 'Consolas'나 'DejaVu Sans Mono' 같은 고정폭 글꼴
    clock = pygame.time.Clock()

    # --- 3. ROS 2 타이머 설정 ---
    # 10Hz (0.1초마다)로 계산 루프 실행
    calculation_timer = node.create_timer(0.1, node.calculation_loop)
    plot_window_active = False # [추가] 플롯 창 상태 추적

    running = True
    try:
        while running:
            # --- 4. Pygame 이벤트 처리 (키보드 입력) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_t: # Tare Force
                        node.call_tare_force()
                    elif event.key == pygame.K_z: # Zero Deflection
                        node.call_tare_deflection()
                    elif event.key == pygame.K_s: # Start
                        node.start_experiment()
                        plot_window_active = True # [추가] 플롯 창 열기
                    elif event.key == pygame.K_q: # Stop/Quit
                        node.stop_experiment()
                        plot_window_active = False # [추가] 플롯 창 닫기
                        cv2.destroyWindow("Stress-Strain Curve")
                    elif event.key == pygame.K_ESCAPE:
                        running = False
                        if plot_window_active: # [추가] 종료 시 플롯 창 닫기
                            cv2.destroyWindow("Stress-Strain Curve")

            # --- 5. GUI 그리기 ---
            # 노드에서 최신 데이터를 가져와 GUI에 전달
            with node.data_lock:
                current_state = {
                    'running': node.experiment_running,
                    'F': node.current_force_N,
                    'd': node.current_deflection_mm,
                    'S': node.current_stress_MPa,
                    'E': node.current_strain,
                }
            draw_gui(screen, font, current_state)

            # --- 5b. [추가] CV2 플로팅 ---
            if plot_window_active:
                # [수정] 클래스 메소드 호출
                node.draw_plot(show_window=True) # CV2 창에 그래프 그리기
            
            # cv2.imshow/destroyWindow가 작동하려면 waitKey(1)이 필요
            cv2.waitKey(1)

            # 초당 30프레임으로 GUI 루프 제한
            clock.tick(30)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        # --- 6. 종료 처리 ---
        node.get_logger().info("Shutting down...")
        if node.experiment_running:
            node.stop_experiment() # 실행 중이었다면 데이터 저장
        
        # ROS 2 종료
        rclpy.shutdown()
        ros_thread.join() # ROS 스레드가 끝날 때까지 대기
        
        # Pygame 종료
        cv2.destroyAllWindows() # [추가] 모든 OpenCV 창 닫기
        pygame.quit()
        node.destroy_node()

if __name__ == '__main__':
    main()