/*
 * Arduino Nano ESP32 WiFi 연결 테스트
 * 환경: VS Code + PlatformIO
 */

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

// WiFi 설정 (본인의 정보로 변경!)
const char* WIFI_SSID = "KT_GiGA_0C1C";
const char* WIFI_PASSWORD = "3dc60ch699";

// 연결 타임아웃 (밀리초)
const unsigned long WIFI_TIMEOUT = 20000;

// 함수 선언 (setup보다 앞에 위치)
void printWiFiInfo();
void printWiFiStatus();
void printCurrentTime();

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);
    
    Serial.println("\n====================================");
    Serial.println("Arduino Nano ESP32 WiFi 테스트");
    Serial.println("PlatformIO 환경에서 실행");
    Serial.println("====================================");
    
    // WiFi 모듈 정보
    Serial.printf("WiFi MAC 주소: %s\n", WiFi.macAddress().c_str());
    
    // WiFi 연결 시작
    Serial.printf("WiFi 연결 시도: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // 연결 대기
    unsigned long startTime = millis();
    Serial.print("연결 중");
    
    while (WiFi.status() != WL_CONNECTED && 
           (millis() - startTime) < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    
    // 연결 결과 확인
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ WiFi 연결 성공!");
        printWiFiInfo();
    } else {
        Serial.println("❌ WiFi 연결 실패!");
        Serial.println("설정을 확인하고 재시도하세요.");
        printWiFiStatus();
    }

    // NTP 서버 설정
  configTime(9 * 3600, 0, "pool.ntp.org");  // 한국 시간 (UTC+9)

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.printf("현재시각: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900,
                  timeinfo.tm_mon + 1,
                  timeinfo.tm_mday,
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec);
  } else {
    Serial.println("시간 정보를 가져오지 못했습니다.");
  }
}

void loop() {
    // WiFi 상태 모니터링 (10초마다)
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck >= 10000) {
        lastCheck = millis();
        
        if (WiFi.status() == WL_CONNECTED) {
            printWiFiInfo();
            
            Serial.printf("[%lu] WiFi 연결됨 - RSSI: %d dBm\n", 
                         millis(), WiFi.RSSI());
        } else {
            Serial.printf("[%lu] WiFi 연결 끊어짐 - 재연결 시도\n", millis());
            WiFi.reconnect();
        }
    }
    
    yield();
}

void printWiFiInfo() {
    Serial.println("--- WiFi 연결 정보 ---");
    printCurrentTime();
    Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("IP 주소: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("서브넷 마스크: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("게이트웨이: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("신호 강도: %d dBm\n", WiFi.RSSI());
    Serial.printf("채널: %d\n", WiFi.channel());
    Serial.println("------------------------");
}

void printWiFiStatus() {
    Serial.print("WiFi 상태: ");
    switch (WiFi.status()) {
        case WL_IDLE_STATUS:
            Serial.println("대기 중");
            break;
        case WL_NO_SSID_AVAIL:
            Serial.println("SSID를 찾을 수 없음");
            break;
        case WL_SCAN_COMPLETED:
            Serial.println("스캔 완료");
            break;
        case WL_CONNECTED:
            Serial.println("연결됨");
            break;
        case WL_CONNECT_FAILED:
            Serial.println("연결 실패");
            break;
        case WL_CONNECTION_LOST:
            Serial.println("연결 끊어짐");
            break;
        case WL_DISCONNECTED:
            Serial.println("연결 해제됨");
            break;
        default:
            Serial.println("알 수 없는 상태");
            break;
    }
}

void printCurrentTime() {
  // 5초마다 현재 시각 출력
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.printf("현재시각: %04d-%02d-%02d %02d:%02d:%02d\n",
                  timeinfo.tm_year + 1900,
                  timeinfo.tm_mon + 1,
                  timeinfo.tm_mday,
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec);
  }
  // delay(5000);
}