import cv2
import numpy as np
import sys

class DotTracker:
    """
    관심 영역(ROI) 내에서 지정된 색상 범위(기본값: 검은색)의 
    가장 큰 객체를 추적하는 클래스.
    """
    
    def __init__(self, roi_rect, color_lower, color_upper, min_contour_area=10):
        """
        트래커를 초기화합니다.
        
        :param roi_rect: (x, y, w, h) 형태의 관심 영역 사각형
        :param color_lower: HSV 형식의 추적할 색상 하한값 (예: [0, 0, 0])
        :param color_upper: HSV 형식의 추적할 색상 상한값 (예: [180, 255, 50])
        :param min_contour_area: 추적할 객체의 최소 픽셀 면적 (노이즈 제거용)
        """
        self.roi_x, self.roi_y, self.roi_w, self.roi_h = roi_rect
        self.lower_hsv = np.array(color_lower, dtype=np.uint8) # [수정] 타입을 uint8로 강제
        self.upper_hsv = np.array(color_upper, dtype=np.uint8) # [수정] 타입을 uint8로 강제

        self.min_area = min_contour_area
        
        # [연속성 보장] 마지막으로 성공한 위치를 저장합니다.
        # 점을 놓쳤을 경우 이 위치를 반환합니다.
        self.last_known_position = None 

        # [추가] 디버깅을 위해 현재 면적을 저장
        self.current_area = 0.0

        print("DotTracker initialized.")
        print(f"  ROI (x,y,w,h): {roi_rect}")
        print(f"  Min Area: {min_contour_area}")

    def track(self, full_frame, debug_viz=False):
        """
        전체 이미지 프레임을 받아 ROI 내의 점을 추적합니다.
        
        :param full_frame: OpenCV BGR 이미지 프레임
        :param debug_viz: True일 경우, 디버깅용 이미지 딕셔너리를 함께 반환
        :return: (position, debug_images_dict, area)
                 position: (x, y) 좌표 또는 None
                 debug_images_dict: 'ROI_Debug', 'Mask'가 포함된 딕셔너리 또는 None
                 area: 현재 감지된 가장 큰 윤곽선의 면적        
        """
        self.current_area = 0.0 # 매 프레임 초기화

        # 1. ROI 영역만 잘라내기
        roi_frame = full_frame[self.roi_y : self.roi_y + self.roi_h, 
                               self.roi_x : self.roi_x + self.roi_w]
        
        if roi_frame.size == 0:
            print("Error: ROI is outside the frame boundaries.")
            if debug_viz:
                return self.last_known_position, {'ROI_Debug': None, 'Mask': None}, 0.0
            else:
                return self.last_known_position, None, 0.0

        # 2. BGR -> HSV 색 공간으로 변환 (색상 검출에 용이)
        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        
        # 3. 지정된 색상 범위로 마스크 생성 (검은색 점 = 흰색, 나머지 = 검은색)
        mask = cv2.inRange(hsv_frame, self.lower_hsv, self.upper_hsv)
        
        # [수정] 팽창 연산(Dilation)을 2회 적용하여 마스크의 작은 구멍을 메움
        # mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        # 4. 마스크에서 윤곽선(contours) 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        found_center = None
        target_contour = None # [수정] 디버그 시각화를 위해 변수명 변경 (largest -> target)
        
        if contours:
            
            # 5. [수정] min_area보다 큰 '유효한' 윤곽선들만 필터링
            valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_area]

            if valid_contours:
                # 6. [수정] 유효한 것들 중 '가장 아래쪽(Y 최대값)'에 있는 윤곽선 선택
                # c[:, :, 1].max()는 윤곽선을 구성하는 점들 중 Y값의 최대값을 의미함
                target_contour = max(valid_contours, key=lambda c: c[:, :, 1].max())
                
                self.current_area = cv2.contourArea(target_contour)

                # 7. 선택된 윤곽선 내에서 가장 아래쪽 점 좌표 추출
                bottom_point_roi = tuple(target_contour[target_contour[:, :, 1].argmax()][0])


                # cX_roi, cY_roi는 *ROI 내부* 좌표
                cX_roi, cY_roi = bottom_point_roi
                
                # *전체 프레임* 기준으로 좌표 변환
                found_center = (self.roi_x + cX_roi, self.roi_y + cY_roi)
                
                # 8. [연속성] 마지막 위치를 현재 위치로 갱신
                self.last_known_position = found_center

        # 9. [수정] debug_viz 플래그에 따라 시각화 이미지 반환
        if debug_viz:
            # 시각화용 ROI 프레임 생성
            viz_roi_debug = roi_frame.copy()
            cv2.drawContours(viz_roi_debug, contours, -1, (255, 0, 0), 1) # 모든 윤곽선 (파란색)
            
            # if largest_contour is not None and cv2.contourArea(largest_contour) > self.min_area:
            # [수정] target_contour가 존재하면 초록색으로 그림
            if target_contour is not None:
                # 유효한 가장 큰 윤곽선 (초록색)
                # cv2.drawContours(viz_roi_debug, [largest_contour], 0, (0, 255, 0), 2)
                cv2.drawContours(viz_roi_debug, [target_contour], 0, (0, 255, 0), 2)

            debug_images = {
                'ROI_Debug': viz_roi_debug, # 윤곽선이 그려진 ROI
                'Mask': mask              # HSV 임계값이 적용된 마스크
            }
            return self.last_known_position, debug_images, self.current_area
        
        return self.last_known_position, None, self.current_area # 디버그 모드가 아닐 경우

    def draw_visualization(self, frame_to_draw, tracked_pos):
        """
        시각화용 함수: ROI와 추적된 점을 프레임에 그립니다.
        
        :param frame_to_draw: 그림을 그릴 원본 프레임
        :param tracked_pos: (x, y) 추적된 위치
        """
        # 1. ROI 사각형 그리기 (초록색)
        cv2.rectangle(frame_to_draw, 
                      (self.roi_x, self.roi_y), 
                      (self.roi_x + self.roi_w, self.roi_y + self.roi_h), 
                      (0, 255, 0), 2)
        
        # 2. 추적된 위치에 마커 그리기 (찾았을 경우)
        if tracked_pos:
            cv2.drawMarker(frame_to_draw, 
                           tracked_pos, 
                           (0, 0, 255), # 빨간색 십자 마커
                           markerType=cv2.MARKER_CROSS, 
                           markerSize=20, 
                           thickness=2)


# --- [메인 실행 블록] ---
# 이 파일이 "python3 deflection_tracker.py"로 직접 실행될 때만 호출됩니다.
# 나중에 ROS 2 노드에서 import할 때는 이 부분은 실행되지 않습니다.
if __name__ == "__main__":
    
    # --- [설정 값] ---
    CAM_INDEX = 2      # find_cameras.py로 찾은 카메라 인덱스
    TARGET_WIDTH = 640 # 보여줄 창의 너비 (높이는 비율에 맞춰 자동 조절)
    
    # ROI (x, y, 너비, 높이) - [!!! 여기를 수정하세요 !!!]
    # (예: 640x480 영상의 중앙 영역)
    ROI_RECT = (295, 100, 70, 200) 
    
    # 검은색 추적을 위한 HSV 임계값
    # H(색상): 0-180, S(채도): 0-255, V(명도): 0-255
    # 검은색은 V(명도)가 매우 낮은 영역입니다.
    LOWER_BLACK_HSV = [0, 0, 0]      # 하한값
    UPPER_BLACK_HSV = [180, 255, 70] # 상한값 (V를 50 이하로 제한)
    MIN_AREA = 10 # 최소 50 픽셀 이상이어야 점으로 인식 (노이즈 제거)
    # ------------------

    # 1. DotTracker 클래스 객체 생성
    tracker = DotTracker(roi_rect=ROI_RECT,
                         color_lower=LOWER_BLACK_HSV,
                         color_upper=UPPER_BLACK_HSV,
                         min_contour_area=MIN_AREA)

    # 2. 카메라 캡처 시작
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"오류: 카메라 인덱스 {CAM_INDEX}번을 열 수 없습니다.")
        sys.exit()

    print(f"카메라 {CAM_INDEX}번 실행 중... 'q' 키를 누르면 종료됩니다.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("오류: 카메라 프레임을 읽을 수 없습니다.")
            break

        # 3. 원본 비율을 유지하며 640 너비로 리사이즈
        original_h, original_w = frame.shape[:2]
        ratio = TARGET_WIDTH / float(original_w)
        target_height = int(original_h * ratio)
        resized_frame = cv2.resize(frame, (TARGET_WIDTH, target_height))

        # 4. [핵심] 리사이즈된 프레임에서 점 추적
        # (주의: 리사이즈된 영상에 맞게 ROI_RECT를 설정해야 함)
        # [수정] debug_viz=True로 설정하여 디버그 이미지(roi, mask)를 함께 반환받음
        tracked_position, debug_imgs, current_area = tracker.track(resized_frame, debug_viz=True)

        # 5. [시각화] 원본 프레임에 ROI와 추적 결과 그리기
        tracker.draw_visualization(resized_frame, tracked_position)
        
        # 6. 추적된 좌표(있다면)를 창에 텍스트로 표시
        if tracked_position:
            pos_text = f"Pos: {tracked_position[0]}, {tracked_position[1]}"
            cv2.putText(resized_frame, pos_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 6-2. [추가] 실시간 면적(Area) 텍스트 표시
        area_text = f"Area: {current_area:.0f} (Min: {MIN_AREA})"
        cv2.putText(resized_frame, area_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 7. 창 보여주기
        cv2.imshow("Black Dot Tracker", resized_frame)

        # 8. [추가] 디버그 창 띄우기
        if debug_imgs:
            # [수정] 2배 확대를 위해 interpolation=cv2.INTER_NEAREST 사용
            if debug_imgs['ROI_Debug'] is not None:
                h, w = debug_imgs['ROI_Debug'].shape[:2]
                resized_roi_debug = cv2.resize(debug_imgs['ROI_Debug'], (w*2, h*2), 
                                               interpolation=cv2.INTER_NEAREST)
                cv2.imshow("ROI Debug (Contours) - 2x", resized_roi_debug)
            if debug_imgs['Mask'] is not None:
                h, w = debug_imgs['Mask'].shape[:2]
                resized_mask = cv2.resize(debug_imgs['Mask'], (w*2, h*2), 
                                          interpolation=cv2.INTER_NEAREST)
                cv2.imshow("Mask (Thresholded) - 2x", resized_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("'q' 키 입력으로 종료합니다.")
            break

    # 8. 리소스 해제
    cap.release()
    cv2.destroyAllWindows()