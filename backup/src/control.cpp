/*
 * control.cpp - 드론 제어 알고리즘 (PID 제어)
 * BETAFPV 1102 14000KV 모터에 최적화된 제어 로직
 */

#include "../include/control.h"
#include "../include/config.h"
#include <cmath>

// PID 제어기 구조체
struct PIDController
{
  float kp, ki, kd;
  float setpoint;
  float lastError;
  float integral;
  float maxIntegral;
  float output;
  unsigned long lastTime;
};

// 전역 PID 제어기
static PIDController rollPID;
static PIDController pitchPID;
static PIDController yawPID;

// 제어 상태 변수
static bool controlInitialized = false;
static uint8_t currentFlightMode = 0; // 0: STABILIZE, 1: ALTITUDE_HOLD, 2: ACRO

void initializeControl()
{
  Serial.println("제어 시스템 초기화 중...");

  // Roll PID 초기화
  rollPID.kp = ROLL_P_GAIN;
  rollPID.ki = ROLL_I_GAIN;
  rollPID.kd = ROLL_D_GAIN;
  rollPID.maxIntegral = ROLL_MAX_I;
  rollPID.setpoint = 0.0;
  rollPID.lastError = 0.0;
  rollPID.integral = 0.0;
  rollPID.lastTime = micros();

  // Pitch PID 초기화
  pitchPID.kp = PITCH_P_GAIN;
  pitchPID.ki = PITCH_I_GAIN;
  pitchPID.kd = PITCH_D_GAIN;
  pitchPID.maxIntegral = PITCH_MAX_I;
  pitchPID.setpoint = 0.0;
  pitchPID.lastError = 0.0;
  pitchPID.integral = 0.0;
  pitchPID.lastTime = micros();

  // Yaw PID 초기화
  yawPID.kp = YAW_P_GAIN;
  yawPID.ki = YAW_I_GAIN;
  yawPID.kd = YAW_D_GAIN;
  yawPID.maxIntegral = YAW_MAX_I;
  yawPID.setpoint = 0.0;
  yawPID.lastError = 0.0;
  yawPID.integral = 0.0;
  yawPID.lastTime = micros();

  controlInitialized = true;
  Serial.println("제어 시스템 초기화 완료");
}

void initializeMotors()
{
  Serial.println("모터/ESC 초기화 중...");

  // LEDC 채널 설정
  ledcSetup(0, 50, 16); // 채널 0, 50Hz, 16비트 해상도
  ledcSetup(1, 50, 16); // 채널 1
  ledcSetup(2, 50, 16); // 채널 2
  ledcSetup(3, 50, 16); // 채널 3

  // 핀에 채널 연결
  ledcAttachPin(MOTOR_FL_PIN, 0); // Pin 5
  ledcAttachPin(MOTOR_FR_PIN, 1); // Pin 6
  ledcAttachPin(MOTOR_RL_PIN, 2); // Pin 7
  ledcAttachPin(MOTOR_RR_PIN, 3); // Pin 8

  // 초기값 설정 (1000μs = 최소 스로틀)
  int minPulse = map(1000, 1000, 2000, 1638, 6553);
  ledcWrite(0, minPulse);
  ledcWrite(1, minPulse);
  ledcWrite(2, minPulse);
  ledcWrite(3, minPulse);

  Serial.println("모터 초기화 완료");
}

void updateMotorOutputs(MotorOutputs *outputs)
{
  // μs를 LEDC 값으로 변환 (16비트, 50Hz)
  int fl = map(outputs->motor_fl, 1000, 2000, 1638, 6553);
  int fr = map(outputs->motor_fr, 1000, 2000, 1638, 6553);
  int rl = map(outputs->motor_rl, 1000, 2000, 1638, 6553);
  int rr = map(outputs->motor_rr, 1000, 2000, 1638, 6553);

  ledcWrite(0, fl); // Motor FL
  ledcWrite(1, fr); // Motor FR
  ledcWrite(2, rl); // Motor RL
  ledcWrite(3, rr); // Motor RR

#if DEBUG_MOTORS
  Serial.print("Motors: FL=");
  Serial.print(outputs->motor_fl);
  Serial.print(", FR=");
  Serial.print(outputs->motor_fr);
  Serial.print(", RL=");
  Serial.print(outputs->motor_rl);
  Serial.print(", RR=");
  Serial.println(outputs->motor_rr);
#endif
}

void stopAllMotors()
{
  static bool lastStopState = false;

  if (!lastStopState) {  // 처음 호출될 때만 메시지 출력
    Serial.println("모든 모터 정지");
    lastStopState = true;
  }

  int stopValue = map(1000, 1000, 2000, 1638, 6553);
  ledcWrite(0, stopValue);
  ledcWrite(1, stopValue);
  ledcWrite(2, stopValue);
  ledcWrite(3, stopValue);

  // PID 적분항 리셋
  rollPID.integral = 0.0;
  pitchPID.integral = 0.0;
  yawPID.integral = 0.0;

  // Serial.println("모든 모터 정지");
}

void calculateControl(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs, float deltaTime)
{
  if (!controlInitialized)
  {
    return;
  }

  // 조종기 입력 정규화 및 데드밴드 적용
  normalizeControllerInput(input);

  // 비행 모드에 따른 제어 로직
  switch (currentFlightMode)
  {
  case 0: // STABILIZE 모드
    calculateStabilizeMode(sensorData, input, outputs, deltaTime);
    break;

  case 1: // ALTITUDE_HOLD 모드 (추후 구현)
    calculateAltitudeHoldMode(sensorData, input, outputs, deltaTime);
    break;

  case 2: // ACRO 모드
    calculateAcroMode(sensorData, input, outputs, deltaTime);
    break;

  default:
    calculateStabilizeMode(sensorData, input, outputs, deltaTime);
    break;
  }

  // 모터 출력 제한 및 안전 체크
  constrainMotorOutputs(outputs);

#if DEBUG_CONTROL
  debugPrintControl(sensorData, input, outputs);
#endif
}

void calculateStabilizeMode(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs, float deltaTime)
{
  // 목표 각도 계산 (조종기 입력에 비례)
  float targetRoll = input->rollNorm * MAX_ANGLE_ROLL;
  float targetPitch = input->pitchNorm * MAX_ANGLE_PITCH;
  float targetYawRate = input->yawNorm * MAX_YAW_RATE;

  // Roll PID 계산
  rollPID.setpoint = targetRoll;
  float rollOutput = calculatePID(&rollPID, sensorData->roll, deltaTime);

  // Pitch PID 계산
  pitchPID.setpoint = targetPitch;
  float pitchOutput = calculatePID(&pitchPID, sensorData->pitch, deltaTime);

  // Yaw Rate PID 계산 (각속도 제어)
  yawPID.setpoint = targetYawRate;
  float yawOutput = calculatePID(&yawPID, sensorData->gyroZ, deltaTime);

  // 기본 스로틀
  float baseThrottle = input->throttleNorm * (MAX_THROTTLE_VALUE - MOTOR_IDLE_SPEED) + MOTOR_IDLE_SPEED;

  // 모터 믹싱 (쿼드콥터 X 형태)
  outputs->motor_fl = baseThrottle - rollOutput + pitchOutput - yawOutput; // Front Left
  outputs->motor_fr = baseThrottle + rollOutput + pitchOutput + yawOutput; // Front Right
  outputs->motor_rl = baseThrottle - rollOutput - pitchOutput + yawOutput; // Rear Left
  outputs->motor_rr = baseThrottle + rollOutput - pitchOutput - yawOutput; // Rear Right
}

void calculateAcroMode(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs, float deltaTime)
{
  // ACRO 모드: 직접 각속도 제어
  float targetRollRate = input->rollNorm * MAX_YAW_RATE * 2;
  float targetPitchRate = input->pitchNorm * MAX_YAW_RATE * 2;
  float targetYawRate = input->yawNorm * MAX_YAW_RATE;

  // 각속도 PID 제어
  rollPID.setpoint = targetRollRate;
  float rollOutput = calculatePID(&rollPID, sensorData->gyroX, deltaTime);

  pitchPID.setpoint = targetPitchRate;
  float pitchOutput = calculatePID(&pitchPID, sensorData->gyroY, deltaTime);

  yawPID.setpoint = targetYawRate;
  float yawOutput = calculatePID(&yawPID, sensorData->gyroZ, deltaTime);

  // 기본 스로틀
  float baseThrottle = input->throttleNorm * (MAX_THROTTLE_VALUE - MOTOR_IDLE_SPEED) + MOTOR_IDLE_SPEED;

  // 모터 믹싱
  outputs->motor_fl = baseThrottle - rollOutput + pitchOutput - yawOutput;
  outputs->motor_fr = baseThrottle + rollOutput + pitchOutput + yawOutput;
  outputs->motor_rl = baseThrottle - rollOutput - pitchOutput + yawOutput;
  outputs->motor_rr = baseThrottle + rollOutput - pitchOutput - yawOutput;
}

void calculateAltitudeHoldMode(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs, float deltaTime)
{
  // 고도 유지 모드 (추후 바로미터 센서 추가 시 구현)
  // 현재는 STABILIZE 모드와 동일하게 동작
  calculateStabilizeMode(sensorData, input, outputs, deltaTime);
}

float calculatePID(PIDController *pid, float currentValue, float deltaTime)
{
  unsigned long currentTime = micros();

  // 오차 계산
  float error = pid->setpoint - currentValue;

  // Proportional 항
  float pTerm = pid->kp * error;

  // Integral 항
  pid->integral += error * deltaTime;
  pid->integral = CONSTRAIN(pid->integral, -pid->maxIntegral, pid->maxIntegral);
  float iTerm = pid->ki * pid->integral;

  // Derivative 항
  float dTerm = 0.0;
  if (deltaTime > 0)
  {
    dTerm = pid->kd * (error - pid->lastError) / deltaTime;
  }

  // PID 출력
  pid->output = pTerm + iTerm + dTerm;

  // 다음 계산을 위해 저장
  pid->lastError = error;
  pid->lastTime = currentTime;

  return pid->output;
}

void normalizeControllerInput(ControllerInput *input)
{
  // 스로틀 정규화 (0.0 ~ 1.0)
  input->throttleNorm = (float)(input->throttle - MIN_THROTTLE_VALUE) / (MAX_THROTTLE_VALUE - MIN_THROTTLE_VALUE);
  input->throttleNorm = CONSTRAIN(input->throttleNorm, 0.0, 1.0);

  // Roll, Pitch, Yaw 정규화 (-1.0 ~ 1.0)
  float center = (MAX_THROTTLE_VALUE + MIN_THROTTLE_VALUE) / 2.0;
  float range = (MAX_THROTTLE_VALUE - MIN_THROTTLE_VALUE) / 2.0;

  input->rollNorm = (float)(input->roll - center) / range;
  input->pitchNorm = (float)(input->pitch - center) / range;
  input->yawNorm = (float)(input->yaw - center) / range;

  // 데드밴드 적용
  input->rollNorm = applyDeadband(input->rollNorm, STICK_DEADBAND / range);
  input->pitchNorm = applyDeadband(input->pitchNorm, STICK_DEADBAND / range);
  input->yawNorm = applyDeadband(input->yawNorm, STICK_DEADBAND / range);

  // 범위 제한
  input->rollNorm = CONSTRAIN(input->rollNorm, -1.0, 1.0);
  input->pitchNorm = CONSTRAIN(input->pitchNorm, -1.0, 1.0);
  input->yawNorm = CONSTRAIN(input->yawNorm, -1.0, 1.0);
}

float applyDeadband(float input, float deadband)
{
  if (fabs(input) < deadband)
  {
    return 0.0;
  }
  else if (input > 0)
  {
    return (input - deadband) / (1.0 - deadband);
  }
  else
  {
    return (input + deadband) / (1.0 - deadband);
  }
}

void constrainMotorOutputs(MotorOutputs *outputs)
{
  // 모터 출력 범위 제한
  outputs->motor_fl = CONSTRAIN(outputs->motor_fl, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_fr = CONSTRAIN(outputs->motor_fr, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_rl = CONSTRAIN(outputs->motor_rl, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_rr = CONSTRAIN(outputs->motor_rr, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);

  // 스로틀이 낮을 때 모터 정지
  if (outputs->motor_fl < MOTOR_IDLE_SPEED)
    outputs->motor_fl = MIN_THROTTLE_VALUE;
  if (outputs->motor_fr < MOTOR_IDLE_SPEED)
    outputs->motor_fr = MIN_THROTTLE_VALUE;
  if (outputs->motor_rl < MOTOR_IDLE_SPEED)
    outputs->motor_rl = MIN_THROTTLE_VALUE;
  if (outputs->motor_rr < MOTOR_IDLE_SPEED)
    outputs->motor_rr = MIN_THROTTLE_VALUE;
}

void setFlightMode(uint8_t mode)
{
  if (mode <= 2)
  {
    currentFlightMode = mode;

    // 모드 변경 시 PID 적분항 리셋
    rollPID.integral = 0.0;
    pitchPID.integral = 0.0;
    yawPID.integral = 0.0;

    Serial.print("비행 모드 변경: ");
    switch (mode)
    {
    case 0:
      Serial.println("STABILIZE");
      break;
    case 1:
      Serial.println("ALTITUDE_HOLD");
      break;
    case 2:
      Serial.println("ACRO");
      break;
    }
  }
}

uint8_t getFlightMode()
{
  return currentFlightMode;
}

void tuneRollPID(float kp, float ki, float kd)
{
  rollPID.kp = kp;
  rollPID.ki = ki;
  rollPID.kd = kd;
  rollPID.integral = 0.0; // 적분항 리셋

  Serial.print("Roll PID 조정: P=");
  Serial.print(kp);
  Serial.print(", I=");
  Serial.print(ki);
  Serial.print(", D=");
  Serial.println(kd);
}

void tunePitchPID(float kp, float ki, float kd)
{
  pitchPID.kp = kp;
  pitchPID.ki = ki;
  pitchPID.kd = kd;
  pitchPID.integral = 0.0; // 적분항 리셋

  Serial.print("Pitch PID 조정: P=");
  Serial.print(kp);
  Serial.print(", I=");
  Serial.print(ki);
  Serial.print(", D=");
  Serial.println(kd);
}

void tuneYawPID(float kp, float ki, float kd)
{
  yawPID.kp = kp;
  yawPID.ki = ki;
  yawPID.kd = kd;
  yawPID.integral = 0.0; // 적분항 리셋

  Serial.print("Yaw PID 조정: P=");
  Serial.print(kp);
  Serial.print(", I=");
  Serial.print(ki);
  Serial.print(", D=");
  Serial.println(kd);
}

void resetPIDIntegrals()
{
  rollPID.integral = 0.0;
  pitchPID.integral = 0.0;
  yawPID.integral = 0.0;
  Serial.println("PID 적분항 리셋");
}

void emergencyStabilize(SensorData *sensorData, MotorOutputs *outputs)
{
  // 긴급 안정화 모드 (통신 두절 시)
  Serial.println("긴급 안정화 모드 활성화");

  // 목표: 수평 유지, 천천히 하강
  rollPID.setpoint = 0.0;
  pitchPID.setpoint = 0.0;
  yawPID.setpoint = 0.0;

  float deltaTime = 0.001; // 1ms 가정

  float rollOutput = calculatePID(&rollPID, sensorData->roll, deltaTime);
  float pitchOutput = calculatePID(&pitchPID, sensorData->pitch, deltaTime);
  float yawOutput = 0; // Yaw 제어 중단

  // 천천히 하강하는 기본 스로틀
  float baseThrottle = FAILSAFE_THROTTLE;

  // 모터 믹싱
  outputs->motor_fl = baseThrottle - rollOutput + pitchOutput - yawOutput;
  outputs->motor_fr = baseThrottle + rollOutput + pitchOutput + yawOutput;
  outputs->motor_rl = baseThrottle - rollOutput - pitchOutput + yawOutput;
  outputs->motor_rr = baseThrottle + rollOutput - pitchOutput - yawOutput;

  // 안전 제한
  constrainMotorOutputs(outputs);
}