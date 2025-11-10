import cv2
import sys

# --- [수정 필요] ---
# 위 find_cameras.py 스크립트로 찾은 번호를 입력하세요.
CAM_INDEX = 2 
# ---------------

# --- [추가됨] 원하는 해상도 설정 ---
RESIZE_WIDTH = 640
RESIZE_HEIGHT = 480
# --------------------------------

def main():
    # 1. 카메라 캡처 객체 생성
    cap = cv2.VideoCapture(CAM_INDEX)

    # 2. 카메라가 열렸는지 확인
    if not cap.isOpened():
        print(f"오류: 카메라 인덱스 {CAM_INDEX}번을 열 수 없습니다.")
        print("먼저 find_cameras.py 스크립트를 실행하여 올바른 인덱스를 확인하세요.")
        sys.exit()

    print(f"카메라 피드를 {RESIZE_WIDTH}x{RESIZE_HEIGHT} 크기로 시작합니다. 'q' 키를 누르면 종료됩니다.")

    while True:
        # 3. 카메라에서 프레임 읽기
        ret, frame = cap.read()

        if not ret:
            print("오류: 카메라에서 프레임을 읽을 수 없습니다.")
            break

        # 4. [수정됨] 읽어온 프레임을 지정된 크기로 리사이즈
        resized_frame = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))

        # 5. [수정됨] 리사이즈된 프레임을 창에 표시
        cv2.imshow("Camera Feed (Resized)", resized_frame)

        # 6. 'q' 키 입력 시 종료 (기존과 동일)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("'q' 키 입력으로 프로그램을 종료합니다.")
            break

    # 7. 리소스 해제 (기존과 동일)
    cap.release()
    cv2.destroyAllWindows()
    print("카메라 리소스 해제 완료.")

if __name__ == "__main__":
    main()