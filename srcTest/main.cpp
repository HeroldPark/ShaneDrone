/*
 * main.cpp - PlatformIO 컴파일 테스트
 * 헤더 파일들과 기본 함수들이 제대로 컴파일되는지 확인
 */

#include <Arduino.h>
#include "config.h"	 // 먼저 config.h 포함
#include "sensors.h" // 그 다음 sensors.h 포함

// 컴파일 타임 체크
#ifndef CONFIG_H
#error "config.h is not properly included"
#endif

// 함수 전방 선언
void initializeStructures();
void testControlFunctions();
void testMathFunctions();
void testTimingFunctions();

// 전역 변수 테스트
SensorData sensorData;
ControllerInput controllerInput;
MotorOutputs motorOutputs;
DroneState droneState;

void setup()
{
	Serial.begin(115200);
	delay(1000);

	Serial.println("=== PlatformIO 컴파일 테스트 ===");
	Serial.println("컴파일 성공!");

	// 핀 모드 설정 테스트
	pinMode(STATUS_LED_PIN, OUTPUT);
	pinMode(MOTOR_FL_PIN, OUTPUT);
	pinMode(MOTOR_FR_PIN, OUTPUT);
	pinMode(MOTOR_RL_PIN, OUTPUT);
	pinMode(MOTOR_RR_PIN, OUTPUT);

	Serial.println("핀 설정 완료");

	// 상수 정의 테스트
	Serial.print("프레임 크기: ");
	Serial.print(FRAME_SIZE);
	Serial.println("mm");

	Serial.print("모터 KV: ");
	Serial.println(MOTOR_KV);

	Serial.print("배터리 셀: ");
	Serial.println(BATTERY_CELLS);

	// 매크로 테스트
	float testAngle = 45.0;
	// float radians = DEG_TO_RAD(testAngle);
	// float degrees = RAD_TO_DEG(radians);
	float radians = testAngle * DEG_TO_RAD; // 곱셈 사용
	float degrees = radians * RAD_TO_DEG;	// 곱셈 사용

	Serial.print("각도 변환 테스트: ");
	Serial.print(testAngle);
	Serial.print("° → ");
	Serial.print(radians);
	Serial.print(" rad → ");
	Serial.print(degrees);
	Serial.println("°");

	// 구조체 초기화 테스트
	initializeStructures();

	// 센서 초기화 시도 (실제 하드웨어 없어도 컴파일 확인)
	Serial.println("센서 초기화 시도...");
	bool sensorResult = initializeSensors();
	Serial.print("센서 초기화 결과: ");
	Serial.println(sensorResult ? "성공" : "실패 (정상 - 하드웨어 없음)");

	Serial.println("=== 컴파일 테스트 완료 ===");
}

void loop()
{
	// LED 깜빡이기
	static unsigned long lastBlink = 0;
	static bool ledState = false;

	if (millis() - lastBlink > 1000)
	{
		ledState = !ledState;
		digitalWrite(STATUS_LED_PIN, ledState);
		lastBlink = millis();

		// 상태 출력
		Serial.print("상태 LED: ");
		Serial.println(ledState ? "ON" : "OFF");

		// 배터리 전압 읽기 시도
		float voltage = readBatteryVoltage();
		Serial.print("배터리 전압: ");
		Serial.print(voltage);
		Serial.println("V");

		// 센서 상태 확인
		bool sensorHealthy = checkSensorHealth();
		Serial.print("센서 상태: ");
		Serial.println(sensorHealthy ? "정상" : "오류");
	}

	// 센서 데이터 읽기 시도
	static unsigned long lastSensorRead = 0;
	if (micros() - lastSensorRead > 10000)
	{ // 10ms마다
		bool result = readSensorData(&sensorData);
		if (result)
		{
			// 센서 데이터가 성공적으로 읽혔을 때만 출력
			Serial.print("센서 데이터 - Roll: ");
			Serial.print(sensorData.roll);
			Serial.print(", Pitch: ");
			Serial.print(sensorData.pitch);
			Serial.print(", Yaw: ");
			Serial.println(sensorData.yaw);
		}
		lastSensorRead = micros();
	}

	// 컨트롤 함수들 테스트 (실제 하드웨어 없이도 컴파일 확인)
	testControlFunctions();

	delay(10);
}

void initializeStructures()
{
	Serial.println("구조체 초기화...");

	// SensorData 초기화
	sensorData.accelX = 0.0;
	sensorData.accelY = 0.0;
	sensorData.accelZ = 9.81; // 중력
	sensorData.gyroX = 0.0;
	sensorData.gyroY = 0.0;
	sensorData.gyroZ = 0.0;
	sensorData.magX = 0.0;
	sensorData.magY = 0.0;
	sensorData.magZ = 0.0;
	sensorData.roll = 0.0;
	sensorData.pitch = 0.0;
	sensorData.yaw = 0.0;
	sensorData.sensorHealthy = false;
	sensorData.timestamp = 0;

	// ControllerInput 초기화
	controllerInput.throttle = MIN_THROTTLE_VALUE;
	controllerInput.roll = 1500;
	controllerInput.pitch = 1500;
	controllerInput.yaw = 1500;
	controllerInput.aux1 = MIN_THROTTLE_VALUE;
	controllerInput.aux2 = MIN_THROTTLE_VALUE;
	controllerInput.aux3 = MIN_THROTTLE_VALUE;
	controllerInput.throttleNorm = 0.0;
	controllerInput.rollNorm = 0.0;
	controllerInput.pitchNorm = 0.0;
	controllerInput.yawNorm = 0.0;
	controllerInput.receiverConnected = false;
	controllerInput.lastUpdate = 0;

	// MotorOutputs 초기화
	motorOutputs.motor_fl = MIN_THROTTLE_VALUE;
	motorOutputs.motor_fr = MIN_THROTTLE_VALUE;
	motorOutputs.motor_rl = MIN_THROTTLE_VALUE;
	motorOutputs.motor_rr = MIN_THROTTLE_VALUE;

	// DroneState 초기화
	droneState.armed = false;
	droneState.calibrated = false;
	droneState.flightMode = 0; // STABILIZE
	droneState.batteryVoltage = 0.0;
	droneState.flightTime = 0;

	Serial.println("구조체 초기화 완료");
}

void testControlFunctions()
{
	static unsigned long lastTest = 0;

	if (millis() - lastTest > 5000)
	{ // 5초마다 테스트
		Serial.println("제어 함수 테스트...");

		// CONSTRAIN 매크로 테스트
		float testValue = 150.0;
		float constrained = CONSTRAIN(testValue, 0.0, 100.0);
		Serial.print("CONSTRAIN(150, 0, 100) = ");
		Serial.println(constrained);

		// MAP 매크로 테스트
		float mapped = MAP(1500, 1000, 2000, -1.0, 1.0);
		Serial.print("MAP(1500, 1000, 2000, -1, 1) = ");
		Serial.println(mapped);

		// 각도 정규화 테스트
		float angle = 370.0;
		float normalized = normalizeAngle(angle);
		Serial.print("normalizeAngle(370°) = ");
		Serial.print(normalized);
		Serial.println("°");

		// PID 상수 출력
		Serial.println("PID 상수:");
		Serial.print("Roll P/I/D: ");
		Serial.print(ROLL_P_GAIN);
		Serial.print("/");
		Serial.print(ROLL_I_GAIN);
		Serial.print("/");
		Serial.println(ROLL_D_GAIN);

		lastTest = millis();
	}
}

// 추가적인 테스트 함수들
void testMathFunctions()
{
	Serial.println("수학 함수 테스트:");

	// 삼각함수 테스트
	float angle = 30.0;
	// float rad = DEG_TO_RAD(angle);
	float rad = angle * RAD_TO_DEG;
	float sinValue = sin(rad);
	float cosValue = cos(rad);

	Serial.print("sin(30°) = ");
	Serial.println(sinValue);
	Serial.print("cos(30°) = ");
	Serial.println(cosValue);

	// 제곱근 테스트
	float sqrtValue = sqrt(16.0);
	Serial.print("sqrt(16) = ");
	Serial.println(sqrtValue);

	// atan2 테스트
	float atanValue = atan2(1.0, 1.0) * RAD_TO_DEG;
	Serial.print("atan2(1,1) = ");
	Serial.print(atanValue);
	Serial.println("°");
}

void testTimingFunctions()
{
	static unsigned long lastTiming = 0;

	if (millis() - lastTiming > 2000)
	{
		Serial.println("타이밍 함수 테스트:");

		unsigned long currentMillis = millis();
		unsigned long currentMicros = micros();

		Serial.print("millis(): ");
		Serial.println(currentMillis);
		Serial.print("micros(): ");
		Serial.println(currentMicros);

		// 루프 주파수 계산
		static unsigned long loopCount = 0;
		static unsigned long startTime = currentMillis;
		loopCount++;

		if (currentMillis - startTime >= 1000)
		{
			float frequency = (float)loopCount / ((currentMillis - startTime) / 1000.0);
			Serial.print("루프 주파수: ");
			Serial.print(frequency);
			Serial.println(" Hz");

			loopCount = 0;
			startTime = currentMillis;
		}

		lastTiming = currentMillis;
	}
}