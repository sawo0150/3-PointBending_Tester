import cv2
import numpy as np
import sys

# --- [설정] ---
CAM_INDEX = 2      # find_cameras.py로 찾은 카메라 인덱스
RESIZE_WIDTH = 640
RESIZE_HEIGHT = 480
WINDOW_NAME = "HSV & ROI Finder Tool"
# ----------------

# --- 글로벌 데이터 (콜백 함수와 공유하기 위함) ---
# 마우스 위치와 해당 위치의 HSV 값을 저장할 딕셔너리
mouse_data = {
    'x': 0,
    'y': 0,
    'hsv': (0, 0, 0),
    'current_frame': None # 콜백이 참조할 현재 프레임
}

def mouse_event_handler(event, x, y, flags, param):
    """
    마우스 이벤트가 발생할 때 호출되는 콜백 함수.
    마우스가 움직일 때마다(EVENT_MOUSEMOVE) 글로벌 데이터 딕셔너리를 업데이트합니다.
    """
    if event == cv2.EVENT_MOUSEMOVE:
        # 1. 마우스 (x, y) 좌표 업데이트
        mouse_data['x'] = x
        mouse_data['y'] = y

        # 2. 콜백이 참조할 프레임이 있는지, 좌표가 범위 내인지 확인
        frame = mouse_data['current_frame']
        if frame is not None and y < frame.shape[0] and x < frame.shape[1]:
            
            # 3. (x, y) 위치의 BGR 픽셀 값 가져오기
            bgr_pixel = frame[y, x]
            
            # 4. BGR -> HSV 변환
            # cvtColor는 3D 배열(이미지)을 기대하므로 1x1x3 크기의 배열로 만듦
            bgr_array = np.uint8([[bgr_pixel]])
            hsv_array = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)
            
            # 5. HSV 값 업데이트
            mouse_data['hsv'] = hsv_array[0][0]

def main():
    # 1. 카메라 캡처 객체 생성
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"오류: 카메라 인덱스 {CAM_INDEX}번을 열 수 없습니다.")
        sys.exit()

    print("--- HSV & ROI Finder Tool ---")
    print("카메라 피드를 시작합니다. 'q' 키를 누르면 종료됩니다.")
    print("\n[사용법]")
    print("1. 추적할 검은색 점 위에 마우스를 올리고 [명도(V)] 값의 범위를 확인하세요.")
    print("   -> (예: 점은 V=40, 주변은 V=100 -> 상한값(UPPER) V를 50~60으로 설정)")
    print("2. 점이 움직일 영역(ROI)의 좌상단, 우하단 (x, y) 좌표를 기록하세요.")

    # 3. [중요] 마우스 콜백을 연결하려면 창을 먼저 생성해야 함
    cv2.namedWindow(WINDOW_NAME)
    
    # 4. [중요] 생성된 창에 마우스 이벤트 핸들러(콜백 함수) 연결
    cv2.setMouseCallback(WINDOW_NAME, mouse_event_handler)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("오류: 카메라에서 프레임을 읽을 수 없습니다.")
            break

        # 5. 프레임 리사이즈
        resized_frame = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))
        
        # 6. [중요] 콜백 함수가 최신 프레임을 참조할 수 있도록 글로벌 데이터 업데이트
        # .copy()를 사용해 안전하게 전달
        mouse_data['current_frame'] = resized_frame.copy()

        # 7. 화면에 표시할 정보 텍스트 준비
        xy_text = f"Pos (x, y): ({mouse_data['x']}, {mouse_data['y']})"
        hsv_text = f"HSV (H, S, V): {mouse_data['hsv']}"

        # 8. 텍스트 가독성을 위해 검은색 배경 사각형 그리기
        cv2.rectangle(resized_frame, (0, 0), (280, 70), (0, 0, 0), -1)

        # 9. 프레임에 텍스트 그리기 (흰색)
        cv2.putText(resized_frame, xy_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(resized_frame, hsv_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 10. 최종 프레임을 창에 표시
        cv2.imshow(WINDOW_NAME, resized_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("'q' 키 입력으로 프로그램을 종료합니다.")
            break

    # 11. 리소스 해제
    cap.release()
    cv2.destroyAllWindows()
    print("카메라 리소스 해제 완료.")

if __name__ == "__main__":
    main()