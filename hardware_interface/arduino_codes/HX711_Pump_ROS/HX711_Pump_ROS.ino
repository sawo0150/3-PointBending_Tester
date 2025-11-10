#include "HX711.h"

// 핀 설정
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int PUMP_RELAY_PIN = 7; // 1. [추가] 펌프 릴레이 핀

HX711 scale;

void setup() {
  Serial.begin(115200); // ROS 노드의 baud_rate와 일치
  
  // 2. [추가] 펌프 핀 설정 (시작 시 OFF)
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, HIGH); // 릴레이가 LOW일 때 OFF라고 가정
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("HX711 & Pump ROS Ready.");
  
  // 시작 시 자동으로 0점 조절
  tareLoadCell();
}

void loop() {
  // 1. 로드셀 값 읽어서 "L:<값>" 형식으로 전송 (기존과 동일)
  if (scale.is_ready()) {
    long raw_value = scale.get_units(5); // 5회 평균값
    Serial.print("L:");
    Serial.println(raw_value);
  }

  // 2. ROS 노드로부터 명령 수신 (TARE, P_ON, P_OFF)
  checkSerialCommands();
  
  delay(20); // 약 50Hz
}

// 0점 조절 함수 (기존과 동일)
void tareLoadCell() {
  scale.tare(20); 
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
    // 3. [추가] 펌프 제어 로직
    else if (cmd == "P_ON") {
      digitalWrite(PUMP_RELAY_PIN, LOW);
      Serial.println("ACK: Pump ON");
    }
    else if (cmd == "P_OFF") {
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      Serial.println("ACK: Pump OFF");
    }
  }
}