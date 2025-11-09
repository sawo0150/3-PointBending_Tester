#include "HX711.h"

// 1. 핀 설정 (HX711 <-> 아두이노)
const int LOADCELL_DOUT_PIN = 3;  // 데이터 출력(DT) 핀
const int LOADCELL_SCK_PIN = 2;   // 클럭(SCK) 핀

// 2. HX711 라이브러리 객체 생성
HX711 scale;

void setup() {
  // 3. 시리얼 통신 시작 (보드레이트는 115200으로 설정)
  Serial.begin(115200);
  
  // 4. HX711 모듈 시작
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("HX711 Raw Value Test");
  Serial.println("Starting Tare (0점 조절)...");
  
  // 5. 0점 조절 (20회 측정 평균으로 0점 설정)
  scale.tare(20); 
  
  Serial.println("Tare complete. Sending raw values...");
}

void loop() {
  // 6. 로드셀 측정이 준비되었는지 확인
  if (scale.is_ready()) {
    
    // 7. Raw 값 읽기 (5회 평균값)
    //    이 값은 보정(calibration)되지 않은 순수 측정값입니다.
    long raw_value = scale.get_value(5);
    
    // 8. PC로 Raw 값 전송
    Serial.println(raw_value);
    
    delay(50); // 약 20Hz (너무 빠르면 시리얼 버퍼 문제 발생 가능)
  }
}