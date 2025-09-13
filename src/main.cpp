/*
 * DIY Shane Drone - Arduino Nano ESP32 + MPU9250
 * BETAFPV 1102 14000KV 모터 + Walksnail Avatar HD V2
 *
 * 개발자: DIY 드론 제작자
 * 버전: 1.0
 * 날짜: 2025.08
 */

#include "config.h"
#include "control.h"
#include "sensors.h"
#include "communication.h"
#include <cmath>

// 전역 변수
DroneState droneState;
SensorData sensorData;
ControllerInput controllerInput;
MotorOutputs motorOutputs;

// 타이밍 변수
unsigned long lastLoopTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastControlUpdate = 0;
unsigned long lastTelemetryUpdate = 0;

// 시스템 상태 (communication.cpp에서 정의됨)
extern bool systemArmed;
bool systemReady = false;
float batteryVoltage = 0.0;

// 함수 선언들
void checkArmStatus(ControllerInput *input);
void checkBatteryStatus(float voltage);
void updateSystemStatus();
void performSafetyChecks();
void activateFailsafeMode();
void debugSystemStatus();
void communicationLoop();

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("DIY Shane Drone v1.0");
  Serial.println("Arduino Nano ESP32 + MPU9250");
  Serial.println("=================================");

   Serial.printf("시작 시 Free heap: %d bytes\n", ESP.getFreeHeap());

  // LED 초기화 (상태 표시용)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 하드웨어 초기화
  // MPU9250 센서 있을 떄
  // if (!initializeSensors())
  // {
  //   Serial.println("ERROR: 센서 초기화 실패!");
  //   while (1)
  //   {
  //     delay(1000);
  //     Serial.println("시스템 중단 - 센서 확인 필요");
  //   }
  // }

  // 센서 초기화 시도
  bool sensorSuccess = initializeSensors();
  if (!sensorSuccess) {
    Serial.println("WARNING: 센서 초기화 실패 - 시뮬레이션 모드로 전환");
    // 센서 없이도 계속 진행
  }

  // 모터 초기화 (void 타입)
  initializeMotors();
  Serial.println("모터 초기화 완료");

  if (!initializeCommunication())
  {
    Serial.println("WARNING: 통신 초기화 실패 - 수동 모드로 전환");
  }

  // 제어 시스템 초기화
  initializeControl();

  // 시스템 준비 완료
  systemReady = true;
  Serial.println("시스템 준비 완료!");
  Serial.println("ARM 신호 대기 중...");

  Serial.println("시스템 준비 완료! (센서 상태: " + String(sensorSuccess ? "정상" : "시뮬레이션") + ")");

  lastLoopTime = micros();

  Serial.printf("초기화 후 Free heap: %d bytes\n", ESP.getFreeHeap());
  
  if (ESP.getFreeHeap() < 50000) {
    Serial.println("WARNING: 메모리 부족!");
  }
}

void loop()
{
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastLoopTime) / 1000000.0f;
  lastLoopTime = currentTime;

  // 메인 루프 주파수 제한 (1000Hz)
  if (deltaTime < 0.001f) {
    return;
  }

  // 1. 센서 데이터 읽기 (1000Hz)
  if (currentTime - lastSensorRead >= SENSOR_UPDATE_INTERVAL) {
    readSensorData(&sensorData);
    lastSensorRead = currentTime;

    // 배터리 전압 모니터링
    batteryVoltage = readBatteryVoltage();
    checkBatteryStatus(batteryVoltage);
  }

  // 2. 통신 데이터 처리 (500Hz)
  if (currentTime - lastControlUpdate >= CONTROL_UPDATE_INTERVAL) {
    // 리시버 데이터 읽기
    readReceiverData(&controllerInput);

    // ARM/DISARM 체크
    checkArmStatus(&controllerInput);

    lastControlUpdate = currentTime;
  }

  // 3. 제어 알고리즘 실행 (수정됨)
  static bool lastArmedState = false;

  if (systemArmed && systemReady) {
    if (!lastArmedState) {
      Serial.println("모터 제어 시작");
      lastArmedState = true;
    }
    calculateControl(&sensorData, &controllerInput, &motorOutputs, deltaTime);
    updateMotorOutputs(&motorOutputs);
  } else {
    if (lastArmedState) {
      stopAllMotors();  // ARM 상태 변경될 때만 호출
      lastArmedState = false;
    }
  }

  // 4. 텔레메트리 전송 (50Hz)
  if (currentTime - lastTelemetryUpdate >= TELEMETRY_UPDATE_INTERVAL) {
    sendTelemetryData(&sensorData, &controllerInput, batteryVoltage);
    lastTelemetryUpdate = currentTime;
  }

  // 5. 시스템 모니터링
  updateSystemStatus();

  // 6. 안전 체크
  performSafetyChecks();

  debugSystemStatus();  // 추가
}

void debugSystemStatus() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {  // 5초마다
    Serial.println("\n=== 시스템 상태 ===");
    Serial.printf("systemArmed: %s\n", systemArmed ? "true" : "false");
    Serial.printf("systemReady: %s\n", systemReady ? "true" : "false");
    Serial.printf("WiFi 클라이언트: %d\n", WiFi.softAPgetStationNum());
    Serial.printf("WiFi 상태: %s\n", WiFi.status() == WL_CONNECTED ? "연결됨" : "AP 모드");
    Serial.printf("WiFi SSID: %s\n", WiFi.softAPSSID().c_str());
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("배터리: %.2fV\n", batteryVoltage);
    
    // 클라이언트 연결 시도 감지
    static int lastClientCount = -1;
    int currentClients = WiFi.softAPgetStationNum();
    if (currentClients != lastClientCount) {
      Serial.printf(">>> 클라이언트 수 변경: %d -> %d\n", lastClientCount, currentClients);
      lastClientCount = currentClients;
    }
    
    lastDebug = millis();
  }
}

// ARM/DISARM 체크
void checkArmStatus(ControllerInput *input)
{
  static bool lastArmState = false;
  bool currentArmSwitch = (input->aux1 > ARM_THRESHOLD);

  // ARM 조건 체크
  bool canArm = (input->throttle < ARM_THROTTLE_THRESHOLD) &&
                (abs((int)input->roll - 1500) < ARM_STICK_THRESHOLD) &&
                (abs((int)input->pitch - 1500) < ARM_STICK_THRESHOLD) &&
                (abs((int)input->yaw - 1500) < ARM_STICK_THRESHOLD) &&
                (batteryVoltage > MIN_BATTERY_VOLTAGE) &&
                systemReady;

  if (currentArmSwitch && !lastArmState && canArm)
  {
    // ARM 시퀀스
    systemArmed = true;
    Serial.println("*** SYSTEM ARMED ***");
    calibrateGyroscope();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (!currentArmSwitch && lastArmState)
  {
    // DISARM 시퀀스
    systemArmed = false;
    Serial.println("*** SYSTEM DISARMED ***");
    stopAllMotors();
    digitalWrite(LED_BUILTIN, LOW);
  }

  lastArmState = currentArmSwitch;
}

// 배터리 상태 체크
void checkBatteryStatus(float voltage)
{
  static unsigned long lastBatteryWarning = 0;
  unsigned long currentTime = millis();

  if (voltage < CRITICAL_BATTERY_VOLTAGE)
  {
    if (systemArmed)
    {
      Serial.println("!!! CRITICAL BATTERY - EMERGENCY LANDING !!!");
      systemArmed = false;
      stopAllMotors();
    }
  }
  else if (voltage < LOW_BATTERY_VOLTAGE)
  {
    if (currentTime - lastBatteryWarning > 10000)
    {
      Serial.printf("WARNING: Low Battery - %.2fV\n", voltage);
      lastBatteryWarning = currentTime;
    }
  }
}

// 시스템 상태 업데이트
void updateSystemStatus()
{
  static unsigned long lastStatusUpdate = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastStatusUpdate > 1000)
  {
    if (!systemArmed)
    {
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
    lastStatusUpdate = currentTime;
  }
}

// 안전 체크
void performSafetyChecks()
{
  static unsigned long lastSensorCheck = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSensorCheck > 5000)
  {
    if (!checkSensorHealth())
    {
      Serial.println("WARNING: 센서 오류 감지");
      if (systemArmed)
      {
        systemArmed = false;
        stopAllMotors();
        Serial.println("센서 오류로 인한 자동 DISARM");
      }
    }
    lastSensorCheck = currentTime;
  }

  if (isReceiverTimeout() && systemArmed)
  {
    Serial.println("통신 타임아웃 - 안전 모드 활성화");
    activateFailsafeMode();
  }
}

// Failsafe 모드
void activateFailsafeMode()
{
  static unsigned long failsafeStartTime = 0;
  static bool failsafeActive = false;

  Serial.println("=== FAILSAFE MODE ACTIVATED ===");

  if (!failsafeActive)
  {
    failsafeStartTime = millis();
    failsafeActive = true;
  }

  // 모든 스틱을 중앙으로 설정하고 천천히 하강
  controllerInput.roll = 1500;
  controllerInput.pitch = 1500;
  controllerInput.yaw = 1500;
  controllerInput.throttle = FAILSAFE_THROTTLE; // 천천히 하강

  // 10초 후 자동 DISARM
  if (millis() - failsafeStartTime > 10000)
  {
    systemArmed = false;
    stopAllMotors();
    failsafeActive = false;
    Serial.println("Failsafe 타임아웃 - 자동 DISARM");
  }
}

// main.cpp에 추가
void debugPrintStatus() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) { // 10Hz
        Serial.printf("R:%.1f P:%.1f Y:%.1f T:%.2f FL:%d FR:%d RL:%d RR:%d\n",
            sensorData.roll, sensorData.pitch, sensorData.yaw,
            controllerInput.throttleNorm,
            motorOutputs.motor_fl, motorOutputs.motor_fr,
            motorOutputs.motor_rl, motorOutputs.motor_rr);
        lastPrint = millis();
    }
}