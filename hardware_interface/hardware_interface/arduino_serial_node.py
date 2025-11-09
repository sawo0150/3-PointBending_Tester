import rclpy
from rclpy.node import Node
import serial
import time
import threading

from std_msgs.msg import Float64
from std_srvs.srv import Empty  # 0점 조절(Tare) 서비스용

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # ROS 2 파라미터 선언 (포트, 보드레이트, 보정값)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # [중요] N(뉴턴) 변환을 위한 보정값
        # 이 값은 나중에 런치 파일에서 설정합니다. (지금은 임시값)
        self.declare_parameter('calibration_factor', 0.0001) 

        # 파라미터 값 가져오기
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.calibration_factor = self.get_parameter('calibration_factor').get_parameter_value().double_value

        self.get_logger().info(f'Connecting to port {self.port} at {self.baud_rate} bps...')

        # 시리얼 포트 연결
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            time.sleep(2) # 아두이노 리셋 대기
            self.get_logger().info('Serial connection established.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            rclpy.shutdown()
            return

        # ROS 2 퍼블리셔 (N 단위 힘)
        # (토픽 이름을 제안서의 /raw_force에서 /force로 변경. 더 직관적임)
        self.force_publisher_ = self.create_publisher(Float64, '/force', 10)

        # ROS 2 서비스 (0점 조절)
        self.tare_service_ = self.create_service(
            Empty,
            '/tare_loadcell',
            self.tare_callback)
        
        # (펌프 구독자 관련 코드 제거됨)

        # 시리얼 읽기/처리를 위한 별도 스레드 시작
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_loop)
        self.thread.start()

    def read_serial_loop(self):
        """ 시리얼 포트에서 지속적으로 데이터를 읽고 파싱하는 스레드 """
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    # 아두이노에서 "L:<raw_value>" 형식으로 데이터가 오는지 확인
                    if line.startswith('L:'):
                        raw_value_str = line.split(':')[1]
                        raw_value = float(raw_value_str)
                        
                        # N 단위로 변환
                        force_newtons = raw_value * self.calibration_factor
                        
                        # Float64 메시지 생성 및 발행
                        msg = Float64()
                        msg.data = force_newtons
                        self.force_publisher_.publish(msg)
                    
                    # 아두이노의 디버그 메시지(예: "ACK: TARE complete")도 로그로 출력
                    elif line.startswith('ACK:'):
                        self.get_logger().info(f'Arduino msg: {line}')
                        
            except Exception as e:
                self.get_logger().warn(f'Error reading serial: {e}')

    def tare_callback(self, request, response):
        """ /tare_loadcell 서비스를 받았을 때 아두이노에 0점 조절 명령 전송 """
        try:
            # 0점 조절 (아두이노가 'TARE\n'을 인식)
            self.ser.write(b'TARE\n')
            self.get_logger().info('Sent: TARE command to Arduino')
        except Exception as e:
            self.get_logger().error(f'Failed to send tare command: {e}')
        
        return response # Empty 응답 반환

    def destroy_node(self):
        """ 노드 종료 시 스레드 및 시리얼 포트 정리 """
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info('Serial port closed. Node shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoSerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()