/*
 * 시리얼 플로터용 데이터 출력
 * VS Code: Ctrl+Alt+S → Serial Plotter 탭 선택
 */

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("시간,CPU온도,메모리사용률,가상센서값");
}

void loop() {
    float temperature = temperatureRead();
    float memoryUsage = (float)(ESP.getHeapSize() - ESP.getFreeHeap()) / ESP.getHeapSize() * 100;
    float virtualSensor = 50 + 20 * sin(millis() * 0.001); // 가상 센서 데이터
    
    // CSV 형태로 출력 (플로터에서 그래프로 표시)
    Serial.printf("CPU온도: %.1f, 메모리사용률: %.1f, 가상센서값: %.1f\n", temperature, memoryUsage, virtualSensor);
    
    delay(1000);
}