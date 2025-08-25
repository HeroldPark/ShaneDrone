/*
 * Wokwi용 드론 컴파일 테스트
 * https://wokwi.com에서 테스트 가능
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// 기본 설정 (config.h 내용 간소화)
#define STATUS_LED_PIN 2
#define MOTOR_FL_PIN 5
#define MOTOR_FR_PIN 6
#define MOTOR_RL_PIN 7
#define MOTOR_RR_PIN 8

#define FRAME_SIZE 65
#define MOTOR_KV 14000
#define BATTERY_CELLS 1

// 구조체 정의
struct SensorData
{
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float roll, pitch, yaw;
  bool sensorHealthy;
  unsigned long timestamp;
};

struct DroneState
{
  bool armed;
  bool calibrated;
  uint8_t flightMode;
  float batteryVoltage;
  uint16_t flightTime;
};

// 전역 변수
SensorData sensorData;
DroneState droneState;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== Wokwi ESP32 드론 테스트 ===");

  // 핀 설정
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(MOTOR_FL_PIN, OUTPUT);
  pinMode(MOTOR_FR_PIN, OUTPUT);
  pinMode(MOTOR_RL_PIN, OUTPUT);
  pinMode(MOTOR_RR_PIN, OUTPUT);

  Serial.println("핀 설정 완료");

  // 구조체 초기화
  initializeData();

  // 기본 테스트
  testBasicFunctions();

  Serial.println("=== 초기화 완료 ===");
}

void loop()
{
  static unsigned long lastUpdate = 0;
  static bool ledState = false;

  // 1초마다 상태 업데이트
  if (millis() - lastUpdate > 1000)
  {
    // LED 토글
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);

    // 상태 출력
    printStatus();

    // 가상 센서 데이터 업데이트
    updateVirtualSensors();

    lastUpdate = millis();
  }

  // 빠른 루프 테스트
  testHighFrequencyLoop();

  delay(10);
}

void initializeData()
{
  Serial.println("데이터 구조체 초기화...");

  // SensorData 초기화
  sensorData.accelX = 0.0;
  sensorData.accelY = 0.0;
  sensorData.accelZ = 9.81;
  sensorData.gyroX = 0.0;
  sensorData.gyroY = 0.0;
  sensorData.gyroZ = 0.0;
  sensorData.roll = 0.0;
  sensorData.pitch = 0.0;
  sensorData.yaw = 0.0;
  sensorData.sensorHealthy = true;
  sensorData.timestamp = 0;

  // DroneState 초기화
  droneState.armed = false;
  droneState.calibrated = true;
  droneState.flightMode = 0;
  droneState.batteryVoltage = 4.1;
  droneState.flightTime = 0;

  Serial.println("초기화 완료");
}

void testBasicFunctions()
{
  Serial.println("기본 함수 테스트:");

  // 수학 함수 테스트
  float angle = 45.0;
  float radians = angle * DEG_TO_RAD;
  float backToDegrees = radians * RAD_TO_DEG;

  Serial.print("각도 변환: ");
  Serial.print(angle);
  Serial.print("° → ");
  Serial.print(radians);
  Serial.print(" rad → ");
  Serial.print(backToDegrees);
  Serial.println("°");

  // 상수 출력
  Serial.print("프레임 크기: ");
  Serial.print(FRAME_SIZE);
  Serial.println("mm");

  Serial.print("모터 KV: ");
  Serial.println(MOTOR_KV);

  // PWM 테스트
  Serial.println("PWM 출력 테스트...");
  for (int i = 0; i < 255; i += 50)
  {
    analogWrite(MOTOR_FL_PIN, i);
    Serial.print("PWM: ");
    Serial.println(i);
    delay(100);
  }
  analogWrite(MOTOR_FL_PIN, 0);
}

void updateVirtualSensors()
{
  // 가상 센서 데이터 생성 (시뮬레이션용)
  float time = millis() / 1000.0;

  sensorData.roll = sin(time * 0.5) * 10.0;      // ±10도 roll
  sensorData.pitch = cos(time * 0.3) * 8.0;      // ±8도 pitch
  sensorData.yaw += (random(-100, 100) / 100.0); // 랜덤 yaw 변화

  // 각도 정규화
  if (sensorData.yaw > 180.0)
    sensorData.yaw -= 360.0;
  if (sensorData.yaw < -180.0)
    sensorData.yaw += 360.0;

  sensorData.timestamp = micros();
}

void printStatus()
{
  Serial.println("=== 드론 상태 ===");
  Serial.print("자세 - Roll: ");
  Serial.print(sensorData.roll, 1);
  Serial.print("°, Pitch: ");
  Serial.print(sensorData.pitch, 1);
  Serial.print("°, Yaw: ");
  Serial.print(sensorData.yaw, 1);
  Serial.println("°");

  Serial.print("배터리: ");
  Serial.print(droneState.batteryVoltage);
  Serial.println("V");

  Serial.print("비행시간: ");
  Serial.print(droneState.flightTime++);
  Serial.println("초");

  Serial.print("상태: ");
  Serial.print(droneState.armed ? "ARMED" : "DISARMED");
  Serial.print(", 모드: ");
  Serial.println(droneState.flightMode);

  Serial.print("메모리 사용량: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");

  Serial.println("==================");
}

void testHighFrequencyLoop()
{
  static unsigned long loopCount = 0;
  static unsigned long lastFreqCheck = 0;

  loopCount++;

  // 1초마다 루프 주파수 계산
  if (millis() - lastFreqCheck >= 1000)
  {
    float frequency = (float)loopCount / ((millis() - lastFreqCheck) / 1000.0);
    Serial.print("루프 주파수: ");
    Serial.print(frequency, 0);
    Serial.println(" Hz");

    loopCount = 0;
    lastFreqCheck = millis();
  }
}

// 추가 테스트 함수들
void testWiFi()
{
  Serial.println("WiFi 기능 테스트...");
  WiFi.mode(WIFI_STA);
  Serial.print("MAC 주소: ");
  Serial.println(WiFi.macAddress());
}

void testI2C()
{
  Serial.println("I2C 스캔...");
  Wire.begin();

  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C 장치 발견: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}