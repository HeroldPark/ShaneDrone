/*
 * config.h - DIY Shane Drone 설정 파일
 * 모든 하드웨어 설정 및 상수 정의
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// =================================
// 하드웨어 핀 설정 (Arduino Nano ESP32)
// =================================

// I2C 핀 (MPU9250용)
#define SDA_PIN 21
#define SCL_PIN 22

// ESC PWM 핀 (4-in-1 ESC 20x20mm)
#define MOTOR_FL_PIN 5 // Front Left
#define MOTOR_FR_PIN 6 // Front Right
#define MOTOR_RL_PIN 7 // Rear Left
#define MOTOR_RR_PIN 8 // Rear Right

// 리시버 핀 (SBUS/PPM)
#define RECEIVER_PIN 4

// 배터리 전압 측정 핀 (분압저항 사용)
#define BATTERY_PIN A0

// LED 상태 표시
#define STATUS_LED_PIN 2

// Walksnail Avatar HD V2 연결
#define VTX_TX_PIN 17
#define VTX_RX_PIN 16

// =================================
// 드론 물리적 특성
// =================================

// 프레임: Shane Brushless Whoop
#define FRAME_SIZE 65        // mm (대각선 모터 거리)
#define MOTOR_DISTANCE 0.046 // m (모터 중심간 거리)

// 모터: BETAFPV 1102 14000KV
#define MOTOR_KV 14000
#define MAX_THROTTLE_VALUE 2000
#define MIN_THROTTLE_VALUE 1000
#define MOTOR_IDLE_SPEED 1100

// 배터리: BETAFPV LAVA 550mAh 1S
#define BATTERY_CELLS 1
#define BATTERY_CAPACITY 0.55 // Ah
#define MAX_BATTERY_VOLTAGE 4.2
#define MIN_BATTERY_VOLTAGE 3.3
#define LOW_BATTERY_VOLTAGE 3.5
#define CRITICAL_BATTERY_VOLTAGE 3.4

// 프로펠러: GF 45mm 2블레이드
#define PROP_DIAMETER 0.045 // m
#define PROP_PITCH 0.025    // m

// =================================
// 센서 설정 (MPU9250)
// =================================

// MPU9250 I2C 주소
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C

// 센서 범위 설정
#define GYRO_SCALE 2000 // ±2000 dps
#define ACCEL_SCALE 8   // ±8g
#define MAG_SCALE 4800  // ±4800 µT

// 센서 필터 설정
#define GYRO_DLPF_CFG 3  // 41Hz
#define ACCEL_DLPF_CFG 3 // 41Hz

// 센서 캘리브레이션 샘플 수
#define CALIBRATION_SAMPLES 1000

// =================================
// 제어 시스템 설정
// =================================

// PID 게인 - BETAFPV 1102 14000KV 모터용 튜닝값
// Roll PID
#define ROLL_P_GAIN 1.2
#define ROLL_I_GAIN 0.8
#define ROLL_D_GAIN 0.15
#define ROLL_MAX_I 100.0

// Pitch PID
#define PITCH_P_GAIN 1.2
#define PITCH_I_GAIN 0.8
#define PITCH_D_GAIN 0.15
#define PITCH_MAX_I 100.0

// Yaw PID
#define YAW_P_GAIN 1.5
#define YAW_I_GAIN 0.5
#define YAW_D_GAIN 0.05
#define YAW_MAX_I 50.0

// 각도 제한 (도 단위)
#define MAX_ANGLE_ROLL 30.0
#define MAX_ANGLE_PITCH 30.0
#define MAX_YAW_RATE 200.0 // deg/s

// 제어 데드밴드
#define STICK_DEADBAND 10 // ±10 마이크로초

// =================================
// 통신 설정
// =================================

// SBUS 설정
#define SBUS_BAUDRATE 100000
#define SBUS_INVERT true

// WiFi 텔레메트리 (선택사항)
#define WIFI_SSID "Shane_Drone"
#define WIFI_PASSWORD "drone123"
#define TELEMETRY_PORT 8888

// 컨트롤 채널 매핑
#define CH_THROTTLE 0
#define CH_ROLL 1
#define CH_PITCH 2
#define CH_YAW 3
#define CH_AUX1 4 // ARM/DISARM
#define CH_AUX2 5 // 비행 모드
#define CH_AUX3 6 // 예비

// =================================
// 시스템 타이밍
// =================================

#define MAIN_LOOP_FREQ 1000             // Hz
#define SENSOR_UPDATE_INTERVAL 1000     // 마이크로초 (1000Hz)
#define CONTROL_UPDATE_INTERVAL 2000    // 마이크로초 (500Hz)
#define TELEMETRY_UPDATE_INTERVAL 20000 // 마이크로초 (50Hz)

// =================================
// 안전 설정
// =================================

// ARM/DISARM 임계값
#define ARM_THRESHOLD 1700          // 마이크로초
#define ARM_THROTTLE_THRESHOLD 1100 // 스로틀이 이 값보다 낮아야 ARM 가능
#define ARM_STICK_THRESHOLD 50      // 스틱이 중앙에서 이 범위 안에 있어야 ARM 가능

// Failsafe 설정
#define RECEIVER_TIMEOUT 1000  // ms (1초)
#define FAILSAFE_THROTTLE 1200 // 천천히 하강

// 배터리 보호
#define VOLTAGE_DIVIDER_RATIO 3.0 // 분압비 (3.3V → 11.1V 측정용)

// config.h에 추가할 설정들
#define WEB_SERVER_PORT 80
#define WEBSOCKET_PORT 81
#define MAX_WEBSOCKET_CLIENTS 5

// JSON 버퍼 크기
#define TELEMETRY_JSON_SIZE 1024
#define COMMAND_JSON_SIZE 256

// =================================
// 데이터 구조체
// =================================

struct SensorData
{
  // 가속도계 (m/s²)
  float accelX, accelY, accelZ;

  // 자이로스코프 (deg/s)
  float gyroX, gyroY, gyroZ;

  // 자기계 (µT)
  float magX, magY, magZ;

  // 융합된 자세 (도)
  float roll, pitch, yaw;

  // 센서 상태
  bool sensorHealthy;

  // 타임스탬프
  unsigned long timestamp;
};

struct ControllerInput
{
  // 조종기 입력 (1000-2000 마이크로초)
  uint16_t throttle;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint16_t aux1; // ARM/DISARM
  uint16_t aux2; // 비행 모드
  uint16_t aux3; // 예비

  // 정규화된 값 (-1.0 ~ 1.0)
  float throttleNorm;
  float rollNorm;
  float pitchNorm;
  float yawNorm;

  // 연결 상태
  bool receiverConnected;
  unsigned long lastUpdate;
};

struct MotorOutputs
{
  uint16_t motor_fl; // Front Left
  uint16_t motor_fr; // Front Right
  uint16_t motor_rl; // Rear Left
  uint16_t motor_rr; // Rear Right
};

struct DroneState
{
  bool armed;
  bool calibrated;
  uint8_t flightMode; // 0: STABILIZE, 1: ALTITUDE_HOLD, 2: ACRO
  float batteryVoltage;
  uint16_t flightTime; // 초 단위
};

// =================================
// 유틸리티 매크로
// =================================

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAP(x, in_min, in_max, out_min, out_max) \
  ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// 수학 상수 정의
#ifndef PI
#define PI 3.14159265358979323846
#endif

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

// =================================
// 디버그 설정
// =================================

#define DEBUG_ENABLED 1
#define DEBUG_SENSORS 0
#define DEBUG_CONTROL 0
#define DEBUG_MOTORS 0

#if DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif // CONFIG_H