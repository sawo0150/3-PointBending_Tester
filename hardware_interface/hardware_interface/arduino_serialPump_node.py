import rclpy
from rclpy.node import Node
import serial
import time
import threading

from std_msgs.msg import Float64, Bool  # 1. [수정] Bool 임포트 추가
from std_srvs.srv import Empty

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # --- 파라미터 선언 (기존과 동일) ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('calibration_factor', 0.000185) # (예시 값이니 보정 필요)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.calibration_factor = self.get_parameter('calibration_factor').get_parameter_value().double_value

        self.get_logger().info(f'Connecting to port {self.port} at {self.baud_rate} bps...')

        # --- 시리얼 연결 (기존과 동일) ---
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            time.sleep(2) 
            self.get_logger().info('Serial connection established.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            rclpy.shutdown()
            return

        # --- 퍼블리셔 (기존과 동일) ---
        self.force_publisher_ = self.create_publisher(Float64, '/force', 10)

        # --- 서비스 (기존과 동일) ---
        self.tare_service_ = self.create_service(
            Empty,
            '/tare_loadcell',
            self.tare_callback)
        
        # 2. [추가] 펌프 제어를 위한 구독자
        self.pump_subscriber_ = self.create_subscription(
            Bool,
            '/pump_control',  # 이 토픽으로 True/False를 받음
            self.pump_callback,
            10)
        self.get_logger().info('Subscribing to /pump_control topic for pump control.')

        # --- 시리얼 읽기 스레드 (기존과 동일) ---
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_loop)
        self.thread.start()

    # --- read_serial_loop (기존과 동일) ---
    def read_serial_loop(self):
        """ 시리얼 포트에서 지속적으로 데이터를 읽고 파싱하는 스레드 """
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line.startswith('L:'):
                        raw_value_str = line.split(':')[1]
                        raw_value = float(raw_value_str)
                        force_newtons = raw_value * self.calibration_factor
                        
                        msg = Float64()
                        msg.data = force_newtons
                        self.force_publisher_.publish(msg)
                    
                    elif line.startswith('ACK:'):
                        self.get_logger().info(f'Arduino msg: {line}')
                        
            except Exception as e:
                self.get_logger().warn(f'Error reading serial: {e}')

    # --- tare_callback (기존과 동일) ---
    def tare_callback(self, request, response):
        """ /tare_loadcell 서비스를 받았을 때 아두이노에 0점 조절 명령 전송 """
        try:
            self.ser.write(b'TARE\n')
            self.get_logger().info('Sent: TARE command to Arduino')
        except Exception as e:
            self.get_logger().error(f'Failed to send tare command: {e}')
        return response

    # 3. [추가] 펌프 토픽 수신 시 호출될 콜백 함수
    def pump_callback(self, msg):
        """ /pump_control 토픽을 받았을 때 아두이노에 명령 전송 """
        try:
            if msg.data:
                # 펌프 켜기 (P_ON)
                self.ser.write(b'P_ON\n')
                self.get_logger().info('Sent: P_ON (Pump Start) to Arduino')
            else:
                # 펌프 끄기 (P_OFF)
                self.ser.write(b'P_OFF\n')
                self.get_logger().info('Sent: P_OFF (Pump Stop) to Arduino')
        except Exception as e:
            self.get_logger().error(f'Failed to send pump command: {e}')

    # --- destroy_node (기존과 동일) ---
    def destroy_node(self):
        # 노드 종료 전 펌프를 끄는 것이 안전할 수 있습니다.
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(b'P_OFF\n')
                self.get_logger().info('Sent P_OFF before shutting down.')
        except Exception as e:
            self.get_logger().warn(f'Could not send P_OFF on shutdown: {e}')
            
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info('Serial port closed. Node shutting down.')
        super().destroy_node()

# --- main (기존과 동일) ---
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