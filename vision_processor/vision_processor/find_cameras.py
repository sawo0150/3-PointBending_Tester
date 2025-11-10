import cv2

def find_active_cameras(max_cameras_to_check=10):
    """
    0번부터 max_cameras_to_check-1번까지의 인덱스를 확인하여,
    성공적으로 열리고 프레임을 읽을 수 있는 카메라 인덱스 리스트를 반환합니다.
    """
    print(f"Checking for active cameras (Index 0 to {max_cameras_to_check - 1})...")
    active_cameras = []
    
    for i in range(max_cameras_to_check):
        # 1. 카메라 인덱스를 열 시도
        cap = cv2.VideoCapture(i)
        
        # 2. 열렸는지 확인 (isOpened()는 빠르지만, 가끔 부정확할 수 있음)
        if cap.isOpened():
            # 3. [중요] 실제로 프레임을 읽을 수 있는지 확인
            ret, frame = cap.read()
            
            if ret:
                print(f"  [+] 성공: 인덱스 {i}번 카메라를 찾았습니다.")
                active_cameras.append(i)
            else:
                print(f"  [-] 실패: 인덱스 {i}번은 열렸으나, 프레임을 읽을 수 없습니다.")
            
            # 4. 확인 후 즉시 리소스 해제
            cap.release()
            
    return active_cameras

if __name__ == "__main__":
    found_indices = find_active_cameras()
    
    if not found_indices:
        print("\n[결과] 활성화된 카메라를 찾지 못했습니다.")
    else:
        print(f"\n[결과] 사용 가능한 카메라 인덱스: {found_indices}")
        print(f"camera_test.py 파일의 CAM_INDEX를 {found_indices[0]} (으)로 설정하세요.")