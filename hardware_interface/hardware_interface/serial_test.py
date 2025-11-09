import serial
import time

# ⚠️ [수정 필요] 아두이노가 연결된 포트 이름으로 변경하세요!
#   - 리눅스/Mac 예시: '/dev/ttyACM0' 또는 '/dev/ttyUSB0'
#   - 윈도우 예시: 'COM3' 또는 'COM4'
#   - (아두이노 IDE의 '도구' -> '포트' 메뉴에서 확인 가능)
PORT_NAME = '/dev/ttyACM0'
BAUD_RATE = 115200  # 아두이노 스케치의 보드레이트와 일치

try:
    # 1. 시리얼 포트 연결
    ser = serial.Serial(PORT_NAME, BAUD_RATE, timeout=1.0)
    print(f"Connecting to {PORT_NAME} at {BAUD_RATE} bps...")
    
    # 2. 아두이노 리셋 대기
    time.sleep(2) 
    
    print("Reading data... (Ctrl+C to stop)")
    
    while True:
        # 3. 시리얼 버퍼에 데이터가 있는지 확인
        if ser.in_waiting > 0:
            
            try:
                # 4. 한 줄 읽고(readline), UTF-8로 디코딩(decode), 양끝 공백 제거(strip)
                line = ser.readline().decode('utf-8').strip()
                
                # 5. 아두이노의 setup()에서 보낸 디버그 메시지나, 실제 값(숫자) 출력
                if line: # 빈 줄이 아니면 출력
                    print(f"Raw Value: {line}")
                    
            except UnicodeDecodeError:
                # 가끔 데이터가 깨져서 들어올 경우 예외 처리
                print("Serial data decode error. Skipping...")

except serial.SerialException as e:
    print(f"Error: Could not open port {PORT_NAME}.")
    print(f"Details: {e}")
    print("Please check the PORT_NAME and device connection.")

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    # 6. 프로그램 종료 시 포트 닫기
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print(f"Port {PORT_NAME} closed.")