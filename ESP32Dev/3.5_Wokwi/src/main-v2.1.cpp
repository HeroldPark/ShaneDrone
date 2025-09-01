// /*
//  * ESP32 드론 컨트롤러 v2.1
//  * GPIO 핀 수정 및 Watchdog 안전 버전
//  */

// #include <Arduino.h>

// // ESP32에서 안전하게 사용 가능한 GPIO 핀
// #define LED_BUILTIN 2      // 내장 LED (안전)
// #define STATUS_LED_PIN 2   // 상태 LED (내장 LED 공유)

// // 외부 LED 핀 정의
// const int LED_PIN = 13;

// // 모터 제어 핀 - ESP32 안전한 GPIO 사용
// #define MOTOR_FL_PIN 25    // 전면 좌측 모터 (GPIO25 안전)
// #define MOTOR_FR_PIN 26    // 전면 우측 모터 (GPIO26 안전)
// #define MOTOR_RL_PIN 27    // 후면 좌측 모터 (GPIO27 안전)
// #define MOTOR_RR_PIN 33    // 후면 우측 모터 (GPIO33 안전)

// // 드론 설정
// #define FRAME_SIZE 65
// #define MOTOR_KV 14000
// #define BATTERY_CELLS 1

// // 디버그 모드
// #define DEBUG_MODE true

// // 구조체 정의
// struct SensorData {
//   float accelX, accelY, accelZ;
//   float gyroX, gyroY, gyroZ;
//   float roll, pitch, yaw;
//   bool sensorHealthy;
//   unsigned long timestamp;
// };

// struct DroneState {
//   bool armed;
//   bool calibrated;
//   uint8_t flightMode;
//   float batteryVoltage;
//   uint16_t flightTime;
// };

// // 전역 변수
// SensorData sensorData;
// DroneState droneState;

// // 타이밍 변수
// unsigned long previousLedMillis = 0;
// unsigned long previousStatusMillis = 0;
// unsigned long previousSensorMillis = 0;
// unsigned long loopStartTime = 0;
// bool ledState = false;
// int loopCounter = 0;

// // 함수 선언
// void initializeSystem();
// void initializeData();
// void updateLED();
// void updateSensors();
// void printStatus();
// void checkSystem();

// void setup() {
//   // 시리얼 통신 초기화 (가장 먼저)
//   Serial.begin(115200);
//   while (!Serial && millis() < 3000) {
//     delay(10);  // Serial 연결 대기 (최대 3초)
//   }
  
//   delay(500);  // 안정화 대기
  
//   Serial.println("\n\n========================================");
//   Serial.println("    ESP32 드론 컨트롤러 v2.1");
//   Serial.println("    GPIO 핀 수정 버전");
//   Serial.println("========================================\n");
  
//   // 시스템 초기화
//   initializeSystem();
  
//   Serial.println("\n>>> 메인 루프 시작 <<<\n");
//   loopStartTime = millis();
// }

// void loop() {
//   unsigned long currentMillis = millis();
  
//   // LED 업데이트 (500ms)
//   if (currentMillis - previousLedMillis >= 500) {
//     previousLedMillis = currentMillis;
//     updateLED();
//   }
  
//   // 센서 업데이트 (100ms)
//   if (currentMillis - previousSensorMillis >= 100) {
//     previousSensorMillis = currentMillis;
//     updateSensors();
//   }
  
//   // 상태 출력 (1초)
//   if (currentMillis - previousStatusMillis >= 1000) {
//     previousStatusMillis = currentMillis;
//     printStatus();
//   }
  
//   // 시스템 체크
//   checkSystem();
  
//   // CPU 부하 감소
//   delay(1);
//   loopCounter++;
// }

// void initializeSystem() {
//   Serial.println("[INIT] 시스템 초기화 시작");
  
//   // GPIO 핀 안전 체크
//   Serial.println("[GPIO] 핀 설정 시작");
  
//   // LED 핀만 먼저 설정
//   Serial.print("  - LED 핀 (GPIO");
//   Serial.print(LED_BUILTIN);
//   Serial.print(") 설정...");
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, LOW);
//   Serial.println(" OK");

//   // 외부 LED 핀 설정
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);
  
//   // 모터 핀 개별 설정 (안전하게)
//   if (DEBUG_MODE) {
//     Serial.println("[GPIO] 모터 핀 설정:");
    
//     Serial.print("  - FL 모터 (GPIO");
//     Serial.print(MOTOR_FL_PIN);
//     Serial.print(")...");
//     pinMode(MOTOR_FL_PIN, OUTPUT);
//     digitalWrite(MOTOR_FL_PIN, LOW);
//     Serial.println(" OK");
//     delay(10);
    
//     Serial.print("  - FR 모터 (GPIO");
//     Serial.print(MOTOR_FR_PIN);
//     Serial.print(")...");
//     pinMode(MOTOR_FR_PIN, OUTPUT);
//     digitalWrite(MOTOR_FR_PIN, LOW);
//     Serial.println(" OK");
//     delay(10);
    
//     Serial.print("  - RL 모터 (GPIO");
//     Serial.print(MOTOR_RL_PIN);
//     Serial.print(")...");
//     pinMode(MOTOR_RL_PIN, OUTPUT);
//     digitalWrite(MOTOR_RL_PIN, LOW);
//     Serial.println(" OK");
//     delay(10);
    
//     Serial.print("  - RR 모터 (GPIO");
//     Serial.print(MOTOR_RR_PIN);
//     Serial.print(")...");
//     pinMode(MOTOR_RR_PIN, OUTPUT);
//     digitalWrite(MOTOR_RR_PIN, LOW);
//     Serial.println(" OK");
//   }
  
//   // 데이터 구조체 초기화
//   Serial.print("[DATA] 구조체 초기화...");
//   initializeData();
//   Serial.println(" OK");
  
//   // LED 테스트
//   Serial.print("[TEST] LED 테스트");
//   for(int i = 0; i < 3; i++) {
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(100);
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(100);
//     Serial.print(".");
//   }
//   Serial.println(" OK");
  
//   // 시스템 정보 출력
//   Serial.println("\n[INFO] 시스템 정보:");
//   Serial.print("  - Chip Model: ");
//   Serial.println(ESP.getChipModel());
//   Serial.print("  - CPU Freq: ");
//   Serial.print(ESP.getCpuFreqMHz());
//   Serial.println(" MHz");
//   Serial.print("  - Free Heap: ");
//   Serial.print(ESP.getFreeHeap());
//   Serial.println(" bytes");
//   Serial.print("  - Flash Size: ");
//   Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
//   Serial.println(" MB");
  
//   Serial.println("\n[INIT] 초기화 완료!");
// }

// void initializeData() {
//   // SensorData 초기화
//   memset(&sensorData, 0, sizeof(SensorData));
//   sensorData.accelZ = 9.81;
//   sensorData.sensorHealthy = true;
  
//   // DroneState 초기화
//   memset(&droneState, 0, sizeof(DroneState));
//   droneState.calibrated = true;
//   droneState.batteryVoltage = 4.1;
// }

// void updateLED() {
//   ledState = !ledState;
//   digitalWrite(LED_BUILTIN, ledState);
  
//   if (DEBUG_MODE && loopCounter % 10 == 0) {
//     Serial.print("[LED] 상태: ");
//     Serial.println(ledState ? "ON" : "OFF");
//   }

//   // 외부 LED 상태 토글
//   digitalWrite(LED_PIN, ledState);
// }

// void updateSensors() {
//   float time = millis() / 1000.0;
  
//   // 간단한 센서 시뮬레이션
//   sensorData.roll = sin(time * 0.5) * 10.0;
//   sensorData.pitch = cos(time * 0.3) * 8.0;
//   sensorData.yaw += 0.1;
  
//   if (sensorData.yaw > 180.0) sensorData.yaw -= 360.0;
  
//   sensorData.timestamp = micros();
// }

// void printStatus() {
//   Serial.println("\n╔════════════════════════════════════╗");
//   Serial.println("║      드론 상태 모니터              ║");
//   Serial.println("╠════════════════════════════════════╣");
  
//   // 자세 정보
//   Serial.print("║ 자세: R=");
//   Serial.print(sensorData.roll, 1);
//   Serial.print("° P=");
//   Serial.print(sensorData.pitch, 1);
//   Serial.print("° Y=");
//   Serial.print(sensorData.yaw, 1);
//   Serial.println("°");
  
//   // 시스템 상태
//   Serial.print("║ 배터리: ");
//   Serial.print(droneState.batteryVoltage);
//   Serial.println(" V");
  
//   Serial.print("║ 비행시간: ");
//   Serial.print(droneState.flightTime++);
//   Serial.println(" 초");
  
//   Serial.print("║ 메모리: ");
//   Serial.print(ESP.getFreeHeap());
//   Serial.println(" bytes");
  
//   Serial.print("║ 루프: ");
//   Serial.print(loopCounter);
//   Serial.print(" (");
//   Serial.print((millis() - loopStartTime) / 1000);
//   Serial.println("초)");
  
//   Serial.println("╚════════════════════════════════════╝");
// }

// void checkSystem() {
//   // Watchdog 리셋
//   yield();
  
//   // 스택 오버플로우 체크
//   if (ESP.getFreeHeap() < 10000) {
//     Serial.println("[WARNING] Low memory!");
//   }
  
//   // 온도 체크 (ESP32 내부 온도 센서)
//   // temperatureRead() 함수는 일부 보드에서 지원
//   #ifdef CONFIG_IDF_TARGET_ESP32
//   float temp = temperatureRead();
//   if (temp > 70.0) {
//     Serial.print("[WARNING] High temperature: ");
//     Serial.println(temp);
//     delay(1000);
//   }
//   #endif
// }

// /*
//  * ESP32 GPIO 핀 맵 참고:
//  * 
//  * 사용 가능한 안전한 GPIO:
//  * - GPIO 2: 내장 LED (대부분 보드)
//  * - GPIO 4, 5: 일반 I/O (단, 5번은 부팅 시 HIGH)
//  * - GPIO 12-15: 일반 I/O (12번은 부팅 관련 주의)
//  * - GPIO 16-19: 일반 I/O
//  * - GPIO 21-23: 일반 I/O
//  * - GPIO 25-27: 일반 I/O
//  * - GPIO 32-33: 일반 I/O (입력 전용 ADC)
//  * 
//  * 사용 불가 또는 주의 필요:
//  * - GPIO 0: 부팅 모드 선택
//  * - GPIO 1, 3: UART0 (시리얼 통신)
//  * - GPIO 6-11: SPI 플래시 (절대 사용 금지!)
//  * - GPIO 34-39: 입력 전용
//  */