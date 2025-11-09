#include "HX711.h"

// 핀 설정
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

HX711 scale;

void setup() {
  Serial.begin(115200); // ROS 노드의 baud_rate와 일치
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("HX711 ROS Ready.");
  
  // 시작 시 자동으로 0점 조절
  tareLoadCell();
}

void loop() {
  // 1. 로드셀 값 읽어서 "L:<값>" 형식으로 전송
  if (scale.is_ready()) {
    long raw_value = scale.get_units(5); // 5회 평균값 (get_units는 tare가 적용된 값)
    
    Serial.print("L:");
    Serial.println(raw_value);
  }

  // 2. ROS 노드로부터 명령 수신 (TARE)
  checkSerialCommands();
  
  delay(20); // 약 50Hz
}

// 0점 조절 함수 (시작 시 및 TARE 명령 수신 시 호출)
void tareLoadCell() {
  scale.tare(20); // 20회 평균으로 0점 설정
  Serial.println("ACK: TARE complete");
}

// 시리얼 명령 처리
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "TARE") {
      tareLoadCell();
    }
  }
}