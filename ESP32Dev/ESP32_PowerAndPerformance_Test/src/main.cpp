/*
 * Arduino Nano ESP32 전력 및 성능 테스트
 * 환경: VS Code + PlatformIO
 */

#include <Arduino.h>
#include <WiFi.h>
// #include <driver/adc.h>

// 테스트 설정
const int TEST_DURATION = 30000;  // 각 테스트 30초
const int SAMPLE_INTERVAL = 1000; // 1초마다 샘플링

// 성능 카운터
struct PerformanceCounter {
    unsigned long loops;
    unsigned long maxLoopTime;
    unsigned long minLoopTime;
    unsigned long totalLoopTime;
};

PerformanceCounter perfCounter = {0, 0, ULONG_MAX, 0};

// 함수 선언 (setup보다 앞에 위치)
void printSystemInfo();
void testNormalOperation();
void testWiFiOperation();
void testLightSleepMode();
void updatePerformanceCounter(unsigned long loopTime);  // 매개변수 추가
void printPerformanceReport();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);
    
    Serial.println("\n====================================");
    Serial.println("Arduino Nano ESP32 전력/성능 테스트");
    Serial.println("====================================");
    
    // 시스템 정보
    printSystemInfo();
    
    // 테스트 시퀀스 실행
    testNormalOperation();
    testWiFiOperation();
    testLightSleepMode();
    
    Serial.println("모든 테스트 완료!");
}

void loop() {
    // 성능 측정을 위한 루프
    unsigned long loopStart = micros();
    
    // 간단한 작업 시뮬레이션
    for (int i = 0; i < 100; i++) {
        float dummy = sin(i * 0.1) + cos(i * 0.1);
        (void)dummy; // 컴파일러 경고 방지
    }
    
    unsigned long loopTime = micros() - loopStart;
    updatePerformanceCounter(loopTime);
    
    // 1초마다 성능 리포트
    static unsigned long lastReport = 0;
    if (millis() - lastReport >= 1000) {
        lastReport = millis();
        printPerformanceReport();
    }
    
    yield();
}

void printSystemInfo() {
    Serial.println("--- 시스템 정보 ---");
    Serial.printf("칩: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("코어 수: %d\n", ESP.getChipCores());
    Serial.printf("CPU 주파수: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("Flash: %d MB (%d MHz)\n", 
                 ESP.getFlashChipSize() / 1024 / 1024,
                 ESP.getFlashChipSpeed() / 1000000);
    Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
    Serial.printf("여유 메모리: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("최대 할당 가능: %d bytes\n", ESP.getMaxAllocHeap());
    Serial.printf("내부 온도: %.1f°C\n", temperatureRead());
    Serial.println("--------------------\n");
}

void testNormalOperation() {
    Serial.println("📊 일반 동작 모드 테스트 시작");
    Serial.println("멀티미터로 전류를 측정하세요 (예상: 80-120mA)");
    
    unsigned long startTime = millis();
    unsigned long nextSample = startTime;
    
    while (millis() - startTime < TEST_DURATION) {
        // CPU 부하 생성
        for (int i = 0; i < 1000; i++) {
            float calculation = sqrt(i) * sin(i * 0.01);
            (void)calculation;
        }
        
        // 샘플링
        if (millis() >= nextSample) {
            Serial.printf("메모리: %d bytes, 온도: %.1f°C\n", 
                         ESP.getFreeHeap(), temperatureRead());
            nextSample += SAMPLE_INTERVAL;
        }
        
        yield();
    }
    
    Serial.println("✅ 일반 동작 테스트 완료\n");
}

void testWiFiOperation() {
    Serial.println("📊 WiFi 활성 모드 테스트 시작");
    Serial.println("예상 전류: 120-200mA");
    
    WiFi.begin("TestNetwork", "password"); // 가상 네트워크
    
    unsigned long startTime = millis();
    unsigned long nextSample = startTime;
    
    while (millis() - startTime < TEST_DURATION) {
        // WiFi 스캔 활동 유지
        static unsigned long lastScan = 0;
        if (millis() - lastScan >= 5000) {
            lastScan = millis();
            WiFi.scanNetworks(true); // 비동기 스캔
        }
        
        // 샘플링
        if (millis() >= nextSample) {
            Serial.printf("WiFi 상태: %d, 메모리: %d bytes\n", 
                         WiFi.status(), ESP.getFreeHeap());
            nextSample += SAMPLE_INTERVAL;
        }
        
        yield();
    }
    
    WiFi.disconnect();
    Serial.println("✅ WiFi 모드 테스트 완료\n");
}

void testLightSleepMode() {
    Serial.println("📊 Light Sleep 모드 테스트 시작");
    Serial.println("예상 전류: 1-5mA");
    Serial.println("5초 후 절전모드 진입 (10초간)");
    
    delay(5000);
    Serial.println("절전모드 진입...");
    Serial.flush(); // 시리얼 버퍼 비우기
    
    // Light Sleep 설정
    esp_sleep_enable_timer_wakeup(10 * 1000000); // 10초
    esp_light_sleep_start();
    
    Serial.println("절전모드에서 깨어남!");
    Serial.println("✅ Light Sleep 테스트 완료\n");
}

void updatePerformanceCounter(unsigned long loopTime) {
    perfCounter.loops++;
    perfCounter.totalLoopTime += loopTime;
    
    if (loopTime > perfCounter.maxLoopTime) {
        perfCounter.maxLoopTime = loopTime;
    }
    
    if (loopTime < perfCounter.minLoopTime) {
        perfCounter.minLoopTime = loopTime;
    }
}

void printPerformanceReport() {
    if (perfCounter.loops > 0) {
        unsigned long avgLoopTime = perfCounter.totalLoopTime / perfCounter.loops;
        
        Serial.printf("성능: 평균 %lu μs, 최대 %lu μs, 최소 %lu μs, 총 %lu 루프\n",
                     avgLoopTime, perfCounter.maxLoopTime, 
                     perfCounter.minLoopTime, perfCounter.loops);
        
        // 카운터 리셋
        perfCounter = {0, 0, ULONG_MAX, 0};
    }
}