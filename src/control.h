/*
 * control.h - 제어 시스템 관련 함수 선언
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "config.h"

// PID 제어기 구조체 전방 선언
struct PIDController;

// 제어 시스템 초기화
void initializeControl();
bool initializeMotors();

// 메인 제어 함수
void calculateControl(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime);

// 비행 모드별 제어
void calculateStabilizeMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime);
void calculateAcroMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime);
void calculateAltitudeHoldMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime);

// PID 계산
float calculatePID(PIDController* pid, float currentValue, float deltaTime);

// 입력 처리
void normalizeControllerInput(ControllerInput* input);
float applyDeadband(float input, float deadband);

// 모터 제어
void updateMotorOutputs(MotorOutputs* outputs);
void stopAllMotors();
void constrainMotorOutputs(MotorOutputs* outputs);

// 비행 모드 관리
void setFlightMode(uint8_t mode);
uint8_t getFlightMode();

// PID 튜닝
void tuneRollPID(float kp, float ki, float kd);
void tunePitchPID(float kp, float ki, float kd);
void tuneYawPID(float kp, float ki, float kd);
void resetPIDIntegrals();

// 안전 기능
void emergencyStabilize(SensorData* sensorData, MotorOutputs* outputs);

// 디버그 및 모니터링
void debugPrintControl(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs);

// 고급 제어 기능
void implementAntiWindup(PIDController* pid);
void adaptivePIDGains(SensorData* sensorData);
float calculateMotorMixing(float throttle, float roll, float pitch, float yaw, uint8_t motorIndex);

// 서보 객체 선언 제거
// static Servo motorFL, motorFR, motorRL, motorRR;

// 대신 채널 번호만 정의
#define MOTOR_FL_CHANNEL 0
#define MOTOR_FR_CHANNEL 1
#define MOTOR_RL_CHANNEL 2
#define MOTOR_RR_CHANNEL 3

#endif // CONTROL_H