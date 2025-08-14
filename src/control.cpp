/*
 * control.cpp - 드론 제어 알고리즘 (PID 제어)
 * BETAFPV 1102 14000KV 모터에 최적화된 제어 로직
 */

#include "control.h"
#include "config.h"

// PID 제어기 구조체
struct PIDController {
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

// 모터 출력 객체
static Servo motorFL, motorFR, motorRL, motorRR;

// 제어 상태 변수
static bool controlInitialized = false;
static uint8_t currentFlightMode = 0; // 0: STABILIZE, 1: ALTITUDE_HOLD, 2: ACRO

void initializeControl() {
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

// bool initializeMotors() {
//   Serial.println("모터/ESC 초기화 중...");
  
//   // Servo 라이브러리로 ESC 제어
//   motorFL.attach(MOTOR_FL_PIN, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
//   motorFR.attach(MOTOR_FR_PIN, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
//   motorRL.attach(MOTOR_RL_PIN, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
//   motorRR.attach(MOTOR_RR_PIN, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  
//   delay(100);
  
//   // ESC 캘리브레이션 시퀀스
//   Serial.println("ESC 캘리브레이션...");
  
//   // 최대 스로틀 신호 전송 (2초)
//   motorFL.writeMicroseconds(MAX_THROTTLE_VALUE);
//   motorFR.writeMicroseconds(MAX_THROTTLE_VALUE);
//   motorRL.writeMicroseconds(MAX_THROTTLE_VALUE);
//   motorRR.writeMicroseconds(MAX_THROTTLE_VALUE);
//   delay(2000);
  
//   // 최소 스로틀 신호 전송 (2초)
//   motorFL.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorFR.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorRL.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorRR.writeMicroseconds(MIN_THROTTLE_VALUE);
//   delay(2000);
  
//   Serial.println("ESC 캘리브레이션 완료");
//   Serial.println("모터 초기화 완료");
  
//   return true;
// }

// void updateMotorOutputs(MotorOutputs *outputs)
// {
//   // 모터 PWM 신호 출력
//   motorFL.writeMicroseconds(outputs->motor_fl);
//   motorFR.writeMicroseconds(outputs->motor_fr);
//   motorRL.writeMicroseconds(outputs->motor_rl);
//   motorRR.writeMicroseconds(outputs->motor_rr);

// #if DEBUG_MOTORS
//   Serial.print("Motors: FL=");
//   Serial.print(outputs->motor_fl);
//   Serial.print(", FR=");
//   Serial.print(outputs->motor_fr);
//   Serial.print(", RL=");
//   Serial.print(outputs->motor_rl);
//   Serial.print(", RR=");
//   Serial.println(outputs->motor_rr);
// #endif
// }

// void stopAllMotors()
// {
//   motorFL.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorFR.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorRL.writeMicroseconds(MIN_THROTTLE_VALUE);
//   motorRR.writeMicroseconds(MIN_THROTTLE_VALUE);

//   // PID 적분항 리셋
//   rollPID.integral = 0.0;
//   pitchPID.integral = 0.0;
//   yawPID.integral = 0.0;

//   Serial.println("모든 모터 정지");
// }

// 서보 객체 대신 직접 PWM 제어
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

  // 초기값 설정 (1500μs = 중립)
  int neutralPulse = map(1500, 1000, 2000, 1638, 6553);
  ledcWrite(0, neutralPulse);
  ledcWrite(1, neutralPulse);
  ledcWrite(2, neutralPulse);
  ledcWrite(3, neutralPulse);

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
}

void stopAllMotors()
{
  int stopValue = map(1000, 1000, 2000, 1638, 6553);
  ledcWrite(0, stopValue);
  ledcWrite(1, stopValue);
  ledcWrite(2, stopValue);
  ledcWrite(3, stopValue);
}

void calculateControl(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime) {
  if (!controlInitialized) {
    return;
  }
  
  // 조종기 입력 정규화 및 데드밴드 적용
  normalizeControllerInput(input);
  
  // 비행 모드에 따른 제어 로직
  switch (currentFlightMode) {
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

void calculateStabilizeMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime) {
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
  outputs->motor_fl = baseThrottle - rollOutput + pitchOutput - yawOutput;  // Front Left
  outputs->motor_fr = baseThrottle + rollOutput + pitchOutput + yawOutput;  // Front Right
  outputs->motor_rl = baseThrottle - rollOutput - pitchOutput + yawOutput;  // Rear Left
  outputs->motor_rr = baseThrottle + rollOutput - pitchOutput - yawOutput;  // Rear Right
}

void calculateAcroMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime) {
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

void calculateAltitudeHoldMode(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs, float deltaTime) {
  // 고도 유지 모드 (추후 바로미터 센서 추가 시 구현)
  // 현재는 STABILIZE 모드와 동일하게 동작
  calculateStabilizeMode(sensorData, input, outputs, deltaTime);
}

float calculatePID(PIDController* pid, float currentValue, float deltaTime) {
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
  if (deltaTime > 0) {
    dTerm = pid->kd * (error - pid->lastError) / deltaTime;
  }
  
  // PID 출력
  pid->output = pTerm + iTerm + dTerm;
  
  // 다음 계산을 위해 저장
  pid->lastError = error;
  pid->lastTime = currentTime;
  
  return pid->output;
}

void normalizeControllerInput(ControllerInput* input) {
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

float applyDeadband(float input, float deadband) {
  if (abs(input) < deadband) {
    return 0.0;
  } else if (input > 0) {
    return (input - deadband) / (1.0 - deadband);
  } else {
    return (input + deadband) / (1.0 - deadband);
  }
}

void constrainMotorOutputs(MotorOutputs* outputs) {
  // 모터 출력 범위 제한
  outputs->motor_fl = CONSTRAIN(outputs->motor_fl, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_fr = CONSTRAIN(outputs->motor_fr, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_rl = CONSTRAIN(outputs->motor_rl, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  outputs->motor_rr = CONSTRAIN(outputs->motor_rr, MIN_THROTTLE_VALUE, MAX_THROTTLE_VALUE);
  
  // 스로틀이 낮을 때 모터 정지
  if (outputs->motor_fl < MOTOR_IDLE_SPEED) outputs->motor_fl = MIN_THROTTLE_VALUE;
  if (outputs->motor_fr < MOTOR_IDLE_SPEED) outputs->motor_fr = MIN_THROTTLE_VALUE;
  if (outputs->motor_rl < MOTOR_IDLE_SPEED) outputs->motor_rl = MIN_THROTTLE_VALUE;
  if (outputs->motor_rr < MOTOR_IDLE_SPEED) outputs->motor_rr = MIN_THROTTLE_VALUE;
}

void setFlightMode(uint8_t mode) {
  if (mode <= 2) {
    currentFlightMode = mode;
    
    // 모드 변경 시 PID 적분항 리셋
    rollPID.integral = 0.0;
    pitchPID.integral = 0.0;
    yawPID.integral = 0.0;
    
    Serial.print("비행 모드 변경: ");
    switch (mode) {
      case 0: Serial.println("STABILIZE"); break;
      case 1: Serial.println("ALTITUDE_HOLD"); break;
      case 2: Serial.println("ACRO"); break;
    }
  }
}

uint8_t getFlightMode() {
  return currentFlightMode;
}

void tuneRollPID(float kp, float ki, float kd) {
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

void tunePitchPID(float kp, float ki, float kd) {
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

void tuneYawPID(float kp, float ki, float kd) {
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

void resetPIDIntegrals() {
  rollPID.integral = 0.0;
  pitchPID.integral = 0.0;
  yawPID.integral = 0.0;
  Serial.println("PID 적분항 리셋");
}

void emergencyStabilize(SensorData* sensorData, MotorOutputs* outputs) {
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

void debugPrintControl(SensorData* sensorData, ControllerInput* input, MotorOutputs* outputs) {
  static unsigned long lastDebugPrint = 0;
  unsigned long currentTime = millis();
  
  // 100ms마다 디버그 출력
  if (currentTime - lastDebugPrint > 100) {
    Serial.println("=== 제어 디버그 ===");
    
    // 센서 데이터
    Serial.print("자세: R=");
    Serial.print(sensorData->roll, 1);
    Serial.print("°, P=");
    Serial.print(sensorData->pitch, 1);
    Serial.print("°, Y=");
    Serial.print(sensorData->yaw, 1);
    Serial.println("°");
    
    // 조종기 입력
    Serial.print("입력: T=");
    Serial.print(input->throttleNorm, 2);
    Serial.print(", R=");
    Serial.print(input->rollNorm, 2);
    Serial.print(", P=");
    Serial.print(input->pitchNorm, 2);
    Serial.print(", Y=");
    Serial.print(input->yawNorm, 2);
    Serial.println();
    
    // PID 출력
    Serial.print("PID: R=");
    Serial.print(rollPID.output, 1);
    Serial.print(", P=");
    Serial.print(pitchPID.output, 1);
    Serial.print(", Y=");
    Serial.print(yawPID.output, 1);
    Serial.println();
    
    // 모터 출력
    Serial.print("모터: FL=");
    Serial.print(outputs->motor_fl);
    Serial.print(", FR=");
    Serial.print(outputs->motor_fr);
    Serial.print(", RL=");
    Serial.print(outputs->motor_rl);
    Serial.print(", RR=");
    Serial.println(outputs->motor_rr);
    
    Serial.println();
    
    lastDebugPrint = currentTime;
  }
}

// 고급 제어 기능들

void implementAntiWindup(PIDController* pid) {
  // Anti-windup: 적분항이 포화되지 않도록 제한
  if (pid->output > MAX_THROTTLE_VALUE || pid->output < MIN_THROTTLE_VALUE) {
    // 출력이 포화되면 적분항을 줄임
    if (pid->integral > 0 && pid->output > MAX_THROTTLE_VALUE) {
      pid->integral *= 0.9;
    } else if (pid->integral < 0 && pid->output < MIN_THROTTLE_VALUE) {
      pid->integral *= 0.9;
    }
  }
}

void adaptivePIDGains(SensorData* sensorData) {
  // 적응형 PID 게인 (추후 구현)
  // 비행 상태에 따라 PID 게인을 동적으로 조정
  
  static float baseRollP = ROLL_P_GAIN;
  static float basePitchP = PITCH_P_GAIN;
  
  // 각속도가 클 때 P 게인을 줄여 오버슈트 방지
  float gyroMagnitude = sqrt(sensorData->gyroX * sensorData->gyroX + 
                            sensorData->gyroY * sensorData->gyroY);
  
  if (gyroMagnitude > 100.0) { // 100 deg/s 이상
    rollPID.kp = baseRollP * 0.8;
    pitchPID.kp = basePitchP * 0.8;
  } else {
    rollPID.kp = baseRollP;
    pitchPID.kp = basePitchP;
  }
}

float calculateMotorMixing(float throttle, float roll, float pitch, float yaw, uint8_t motorIndex) {
  // 모터별 믹싱 계수
  float mixingMatrix[4][4] = {
    // throttle, roll, pitch, yaw
    {1.0, -1.0,  1.0, -1.0}, // Front Left
    {1.0,  1.0,  1.0,  1.0}, // Front Right
    {1.0, -1.0, -1.0,  1.0}, // Rear Left
    {1.0,  1.0, -1.0, -1.0}  // Rear Right
  };
  
  if (motorIndex > 3) return MIN_THROTTLE_VALUE;
  
  return throttle * mixingMatrix[motorIndex][0] +
         roll * mixingMatrix[motorIndex][1] +
         pitch * mixingMatrix[motorIndex][2] +
         yaw * mixingMatrix[motorIndex][3];
}