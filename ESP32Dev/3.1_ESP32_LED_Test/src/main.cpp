/*
 * Arduino Nano ESP32 LED 점멸 테스트 (수정된 버전)
 * 환경: VS Code + PlatformIO
 * 목적: 기본 하드웨어 동작 확인
 */

#include <Arduino.h>  // PlatformIO에서는 필수!

// 핀 정의
const int LED_PIN = 26;

// 전역 변수
unsigned long previousMillis = 0;
const long interval = 1000;  // 1초 간격
bool ledState = false;

void setup() {
    // 시리얼 통신 초기화
    Serial.begin(115200);
    
    // 시리얼 포트 준비 대기
    while (!Serial && millis() < 5000) {
        delay(10);
    }
    
    // LED 핀 설정
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // 시작 메시지
    Serial.println("====================================");
    Serial.println("Arduino Nano ESP32 LED 테스트 시작");
    Serial.println("PlatformIO 환경에서 실행");
    Serial.println("====================================");
    
    // 시스템 정보 출력
    Serial.printf("칩 모델: %s\n", ESP.getChipModel());
    Serial.printf("칩 리비전: %d\n", ESP.getChipRevision());
    Serial.printf("CPU 주파수: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Flash 크기: %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);
    Serial.printf("여유 메모리: %d bytes\n", ESP.getFreeHeap());
    Serial.println();
}

void loop() {
    unsigned long currentMillis = millis();
    
    // 비차단 LED 제어 (millis() 사용)
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // LED 상태 토글
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        
        // 상태 출력
        Serial.printf("[%lu ms] LED %s\n", 
                     currentMillis, 
                     ledState ? "ON" : "OFF");
        
        // 메모리 사용량 주기적 출력 (10초마다)
        static int counter = 0;
        if (++counter >= 10) {
            counter = 0;
            Serial.printf("현재 여유 메모리: %d bytes\n", ESP.getFreeHeap());
        }
    }
    
    // 다른 작업을 위한 여유 시간
    yield();
}