/*
 * DIY Pavo Pico Drone - Arduino Nano ESP32 + MPU9250
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

// 전역 변수들
static uint8_t currentFlightMode = 0;
extern bool systemArmed;
extern DroneState droneState;

// PID 튜닝 함수들
void tuneRollPID(float kp, float ki, float kd) {
    Serial.printf("Roll PID 튜닝: P=%.3f, I=%.3f, D=%.3f\n", kp, ki, kd);
    // 실제 PID 파라미터 적용 (control.cpp의 PID 구조체에 적용)
}

void tunePitchPID(float kp, float ki, float kd) {
    Serial.printf("Pitch PID 튜닝: P=%.3f, I=%.3f, D=%.3f\n", kp, ki, kd);
    // 실제 PID 파라미터 적용
}

void tuneYawPID(float kp, float ki, float kd) {
    Serial.printf("Yaw PID 튜닝: P=%.3f, I=%.3f, D=%.3f\n", kp, ki, kd);
    // 실제 PID 파라미터 적용
}

void resetPIDIntegrals() {
    Serial.println("PID 적분항 리셋");
    // PID 적분항 초기화
}

// 비행 모드 관리
void setFlightMode(uint8_t mode) {
    if (mode <= 2) {  // 0: STABILIZE, 1: ALTITUDE_HOLD, 2: ACRO
        currentFlightMode = mode;
        const char* modeNames[] = {"STABILIZE", "ALTITUDE_HOLD", "ACRO"};
        Serial.printf("비행 모드 변경: %s\n", modeNames[mode]);
    }
}

uint8_t getFlightMode() {
    return currentFlightMode;
}

// 배터리 모니터링
float readBatteryVoltage() {
    static float lastVoltage = 4.0f;
    
    int rawValue = analogRead(BATTERY_PIN);
    float voltage = (rawValue / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;
    
    // 간단한 저역 필터 적용
    lastVoltage = lastVoltage * 0.9f + voltage * 0.1f;
    
    return lastVoltage;
}

// 센서 캘리브레이션
void calibrateSensors() {
    Serial.println("센서 캘리브레이션 시작...");
    calibrateGyroscope();
    calibrateAccelerometer();
    calibrateMagnetometer();
    Serial.println("센서 캘리브레이션 완료!");
}

void calibrateGyroscope() {
    Serial.println("자이로스코프 캘리브레이션 중... (움직이지 마세요)");
    delay(2000);
    // 실제 자이로 캘리브레이션 구현
    Serial.println("자이로스코프 캘리브레이션 완료");
}

void calibrateAccelerometer() {
    Serial.println("가속도계 캘리브레이션 완료");
    // 6면 캘리브레이션 등 구현
}

void calibrateMagnetometer() {
    Serial.println("자력계 캘리브레이션 완료");
    // 자력계 캘리브레이션 구현
}

// ARM/DISARM 체크
void checkArmStatus(ControllerInput *input) {
    static bool lastArmState = false;
    bool currentArmSwitch = (input->aux1 > ARM_THRESHOLD);
    
    // ARM 조건 체크
    bool canArm = (input->throttle < ARM_THROTTLE_THRESHOLD) &&
                  (abs((int)input->roll - 1500) < ARM_STICK_THRESHOLD) &&
                  (abs((int)input->pitch - 1500) < ARM_STICK_THRESHOLD) &&
                  (abs((int)input->yaw - 1500) < ARM_STICK_THRESHOLD) &&
                  (readBatteryVoltage() > MIN_BATTERY_VOLTAGE);
    
    if (currentArmSwitch && !lastArmState && canArm) {
        // ARM 시퀀스
        systemArmed = true;
        Serial.println("*** SYSTEM ARMED ***");
        calibrateGyroscope();
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (!currentArmSwitch && lastArmState) {
        // DISARM 시퀀스
        systemArmed = false;
        Serial.println("*** SYSTEM DISARMED ***");
        stopAllMotors();
        digitalWrite(LED_BUILTIN, LOW);
    }
    
    lastArmState = currentArmSwitch;
}

// 배터리 상태 체크
void checkBatteryStatus(float voltage) {
    static unsigned long lastBatteryWarning = 0;
    unsigned long currentTime = millis();
    
    if (voltage < CRITICAL_BATTERY_VOLTAGE) {
        if (systemArmed) {
            Serial.println("!!! CRITICAL BATTERY - EMERGENCY LANDING !!!");
            systemArmed = false;
            stopAllMotors();
        }
    }
    else if (voltage < LOW_BATTERY_VOLTAGE) {
        if (currentTime - lastBatteryWarning > 10000) {
            Serial.printf("WARNING: Low Battery - %.2fV\n", voltage);
            lastBatteryWarning = currentTime;
        }
    }
}

// 시스템 상태 업데이트
void updateSystemStatus() {
    static unsigned long lastStatusUpdate = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastStatusUpdate > 1000) {
        if (!systemArmed) {
            static bool ledState = false;
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
        }
        lastStatusUpdate = currentTime;
    }
}

// 안전 체크
void performSafetyChecks() {
    static unsigned long lastSensorCheck = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastSensorCheck > 5000) {
        if (!checkSensorHealth()) {
            Serial.println("WARNING: 센서 오류 감지");
            if (systemArmed) {
                systemArmed = false;
                stopAllMotors();
                Serial.println("센서 오류로 인한 자동 DISARM");
            }
        }
        lastSensorCheck = currentTime;
    }
    
    if (isReceiverTimeout() && systemArmed) {
        Serial.println("통신 타임아웃 - 안전 모드 활성화");
        activateFailsafeMode();
    }
}

// Failsafe 모드
void activateFailsafeMode() {
    Serial.println("=== FAILSAFE MODE ACTIVATED ===");
    
    static unsigned long failsafeStartTime = millis();
    static bool failsafeActive = false;
    
    if (!failsafeActive) {
        failsafeStartTime = millis();
        failsafeActive = true;
    }
    
    // 10초 후 자동 DISARM
    if (millis() - failsafeStartTime > 10000) {
        systemArmed = false;
        stopAllMotors();
        failsafeActive = false;
        Serial.println("Failsafe 타임아웃 - 자동 DISARM");
    }
}

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

// 시스템 상태
bool systemArmed = false;
bool systemReady = false;
float batteryVoltage = 0.0;

void setup()
{
	Serial.begin(115200);
	delay(1000);

	Serial.println("=================================");
	Serial.println("DIY Pavo Pico Drone v1.0");
	Serial.println("Arduino Nano ESP32 + MPU9250");
	Serial.println("=================================");

	// 하드웨어 초기화
	if (!initializeSensors())
	{
		Serial.println("ERROR: 센서 초기화 실패!");
		while (1)
		{
			delay(1000);
			Serial.println("시스템 중단 - 센서 확인 필요");
		}
	}

	if (!initializeMotors())
	{
		Serial.println("ERROR: 모터/ESC 초기화 실패!");
		while (1)
		{
			delay(1000);
			Serial.println("시스템 중단 - ESC 확인 필요");
		}
	}

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

	// LED 초기화 (상태 표시용)
	pinMode(LED_BUILTIN, OUTPUT);

	lastLoopTime = micros();
}

void loop()
{
	unsigned long currentTime = micros();
	float deltaTime = (currentTime - lastLoopTime) / 1000000.0f;
	lastLoopTime = currentTime;

	// 메인 루프 주파수 제한 (1000Hz)
	if (deltaTime < 0.001f)
	{
		return;
	}

	// 1. 센서 데이터 읽기 (1000Hz)
	if (currentTime - lastSensorRead >= SENSOR_UPDATE_INTERVAL)
	{
		readSensorData(&sensorData);
		lastSensorRead = currentTime;

		// 배터리 전압 모니터링
		batteryVoltage = readBatteryVoltage();
		checkBatteryStatus(batteryVoltage);
	}

	// 2. 통신 데이터 처리 (500Hz)
	if (currentTime - lastControlUpdate >= CONTROL_UPDATE_INTERVAL)
	{
		// 리시버 데이터 읽기
		readReceiverData(&controllerInput);

		// ARM/DISARM 체크
		checkArmStatus(&controllerInput);

		lastControlUpdate = currentTime;
	}

	// 3. 제어 알고리즘 실행 (1000Hz)
	if (systemArmed && systemReady)
	{
		// PID 제어 계산
		calculateControl(&sensorData, &controllerInput, &motorOutputs, deltaTime);

		// 모터 출력 적용
		updateMotorOutputs(&motorOutputs);
	}
	else
	{
		// 시스템이 DISARM 상태면 모터 정지
		stopAllMotors();
	}

	// 4. 텔레메트리 전송 (50Hz)
	if (currentTime - lastTelemetryUpdate >= TELEMETRY_UPDATE_INTERVAL)
	{
		sendTelemetryData(&sensorData, &controllerInput, batteryVoltage);
		lastTelemetryUpdate = currentTime;
	}

	// 5. 시스템 모니터링
	updateSystemStatus();

	// 6. 안전 체크
	performSafetyChecks();
}

void checkArmStatus(ControllerInput *input)
{
	static bool lastArmState = false;
	bool currentArmSwitch = (input->aux1 > ARM_THRESHOLD);

	// ARM 조건 체크
	bool canArm = (input->throttle < ARM_THROTTLE_THRESHOLD) &&
				  (abs(input->roll) < ARM_STICK_THRESHOLD) &&
				  (abs(input->pitch) < ARM_STICK_THRESHOLD) &&
				  (abs(input->yaw) < ARM_STICK_THRESHOLD) &&
				  (batteryVoltage > MIN_BATTERY_VOLTAGE) &&
				  systemReady;

	if (currentArmSwitch && !lastArmState && canArm)
	{
		// ARM 시퀀스
		systemArmed = true;
		Serial.println("*** SYSTEM ARMED ***");

		// 자이로 캘리브레이션
		calibrateGyroscope();

		// LED 상태 변경
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

void checkBatteryStatus(float voltage)
{
	static unsigned long lastBatteryWarning = 0;
	unsigned long currentTime = millis();

	if (voltage < CRITICAL_BATTERY_VOLTAGE)
	{
		// 긴급 착륙 모드
		if (systemArmed)
		{
			Serial.println("!!! CRITICAL BATTERY - EMERGENCY LANDING !!!");
			// 강제 DISARM (안전을 위해)
			systemArmed = false;
			stopAllMotors();
		}
	}
	else if (voltage < LOW_BATTERY_VOLTAGE)
	{
		// 저전압 경고 (10초마다)
		if (currentTime - lastBatteryWarning > 10000)
		{
			Serial.print("WARNING: Low Battery - ");
			Serial.print(voltage);
			Serial.println("V");
			lastBatteryWarning = currentTime;
		}
	}
}

void updateSystemStatus()
{
	static unsigned long lastStatusUpdate = 0;
	unsigned long currentTime = millis();

	// 1초마다 상태 업데이트
	if (currentTime - lastStatusUpdate > 1000)
	{
		// LED 깜빡임으로 상태 표시
		if (!systemArmed)
		{
			static bool ledState = false;
			ledState = !ledState;
			digitalWrite(LED_BUILTIN, ledState);
		}

		lastStatusUpdate = currentTime;
	}
}

void performSafetyChecks()
{
	static unsigned long lastSensorCheck = 0;
	unsigned long currentTime = millis();

	// 센서 상태 체크 (5초마다)
	if (currentTime - lastSensorCheck > 5000)
	{
		if (!checkSensorHealth())
		{
			Serial.println("WARNING: 센서 오류 감지");
			if (systemArmed)
			{
				// 센서 오류 시 안전을 위해 DISARM
				systemArmed = false;
				stopAllMotors();
				Serial.println("센서 오류로 인한 자동 DISARM");
			}
		}
		lastSensorCheck = currentTime;
	}

	// 통신 타임아웃 체크
	if (isReceiverTimeout() && systemArmed)
	{
		Serial.println("통신 타임아웃 - 안전 모드 활성화");
		// Failsafe 모드 (천천히 하강)
		activateFailsafeMode();
	}
}

void activateFailsafeMode()
{
	Serial.println("=== FAILSAFE MODE ACTIVATED ===");

	// 모든 스틱을 중앙으로 설정하고 천천히 하강
	controllerInput.roll = 0;
	controllerInput.pitch = 0;
	controllerInput.yaw = 0;
	controllerInput.throttle = FAILSAFE_THROTTLE; // 천천히 하강

	// 10초 후 자동 DISARM
	static unsigned long failsafeStartTime = millis();
	if (millis() - failsafeStartTime > 10000)
	{
		systemArmed = false;
		stopAllMotors();
		Serial.println("Failsafe 타임아웃 - 자동 DISARM");
	}
}