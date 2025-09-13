/*
 * communication.cpp - 드론 통신 처리
 * SBUS 수신기, WiFi 텔레메트리, Walksnail Avatar HD V2 연동
 */

#include "communication.h"
#include "config.h"
#include "control.h"
#include "sensors.h"
#include "web.h"              // ✅ 추가: 분리된 웹 모듈
#include <cmath>

// SBUS 관련 변수
static HardwareSerial *sbusSerial;
static uint8_t sbusPacket[25];
static uint16_t sbusChannels[18];
static bool sbusFailsafe = false;
static bool sbusLostFrame = false;
static unsigned long lastSbusReceived = 0;

// WiFi 관련 변수
static WiFiServer telemetryServer(TELEMETRY_PORT);
static WiFiClient telemetryClient;
static bool wifiEnabled = false;

// Walksnail Avatar HD V2 관련
static HardwareSerial *vtxSerial;
static bool vtxConnected = false;

// 전역 변수 선언 (extern으로 선언되어 있으므로 정의 필요)
bool systemArmed = false;

bool initializeCommunication()
{
	Serial.println("통신 시스템 초기화 중...");

	// SBUS 수신기 초기화
	if (!initializeSBUS())
	{
		Serial.println("WARNING: SBUS 초기화 실패");
	}

	// WiFi 텔레메트리 초기화 (선택사항)
	if (initializeWiFi())
	{
		Serial.println("WiFi 텔레메트리 활성화됨");
		wifiEnabled = true;
	}
	else
	{
		Serial.println("WiFi 비활성화됨");
	}

	// Walksnail Avatar HD V2 초기화
	if (initializeVTX())
	{
		Serial.println("Walksnail Avatar HD V2 연결됨");
		vtxConnected = true;
	}
	else
	{
		Serial.println("VTX 연결 실패");
	}

	// ✅ 웹 서버 & WebSocket 초기화 (분리된 모듈)
  	web::begin();

	Serial.println("통신 시스템 초기화 완료");
	return true;
}

bool initializeSBUS()
{
	// SBUS는 Serial2 사용 (핀 16, 17)
	sbusSerial = &Serial2;
	sbusSerial->begin(SBUS_BAUDRATE, SERIAL_8E2, RECEIVER_PIN, -1, SBUS_INVERT);

	// SBUS 채널 초기화 (중앙값으로 설정)
	for (int i = 0; i < 18; i++)
	{
		sbusChannels[i] = 1500; // 중앙값
	}

	Serial.println("SBUS 수신기 초기화됨");
	return true;
}

bool initializeWiFi()
{
	// WiFi AP 모드로 설정
	WiFi.mode(WIFI_AP);

	if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD))
	{
		Serial.print("WiFi AP 생성됨: ");
		Serial.println(WIFI_SSID);
		Serial.print("IP 주소: ");
		Serial.println(WiFi.softAPIP());

		// 텔레메트리 서버 시작
		telemetryServer.begin();

		return true;
	}
	else
	{
		Serial.println("WiFi AP 생성 실패");
		return false;
	}
}

bool initializeVTX()
{
	// Walksnail Avatar HD V2 통신 초기화
	vtxSerial = &Serial1;
	vtxSerial->begin(115200, SERIAL_8N1, VTX_RX_PIN, VTX_TX_PIN);

	delay(100);

	// VTX 연결 테스트
	vtxSerial->print("AT\r\n");
	delay(50);

	if (vtxSerial->available())
	{
		String response = vtxSerial->readString();
		if (response.indexOf("OK") >= 0)
		{
			Serial.println("Walksnail Avatar HD V2 응답 확인");

			// VTX 설정 명령
			vtxSerial->print("AT+POWER=2\r\n"); // 파워 레벨 설정
			delay(10);
			vtxSerial->print("AT+FREQ=5800\r\n"); // 주파수 설정 (5.8GHz)
			delay(10);

			return true;
		}
	}

	Serial.println("VTX 응답 없음");
	return false;
}

void readReceiverData(ControllerInput *input)
{
	if (!readSBUSData())
	{
		// SBUS 데이터가 없으면 Failsafe 상태로 설정
		input->receiverConnected = false;
		setFailsafeValues(input);
		return;
	}

	// SBUS 채널을 ControllerInput으로 변환
	input->throttle = sbusChannels[CH_THROTTLE];
	input->roll = sbusChannels[CH_ROLL];
	input->pitch = sbusChannels[CH_PITCH];
	input->yaw = sbusChannels[CH_YAW];
	input->aux1 = sbusChannels[CH_AUX1];
	input->aux2 = sbusChannels[CH_AUX2];
	input->aux3 = sbusChannels[CH_AUX3];

	input->receiverConnected = !sbusFailsafe;
	input->lastUpdate = millis();

	// 비행 모드 체크 (AUX2 스위치)
	checkFlightModeSwitch(input->aux2);
}

bool readSBUSData()
{
	static uint8_t sbusIndex = 0;
	static unsigned long lastByteTime = 0;

	unsigned long currentTime = millis();

	// SBUS 패킷 타임아웃 체크 (3ms)
	if (currentTime - lastByteTime > 3 && sbusIndex > 0)
	{
		sbusIndex = 0; // 패킷 리셋
	}

	// SBUS 데이터 읽기
	while (sbusSerial->available())
	{
		uint8_t inByte = sbusSerial->read();
		lastByteTime = currentTime;

		if (sbusIndex == 0 && inByte != 0x0F)
		{
			// SBUS 헤더 바이트가 아니면 무시
			continue;
		}

		sbusPacket[sbusIndex] = inByte;
		sbusIndex++;

		if (sbusIndex >= 25)
		{
			// 완전한 SBUS 패킷 수신됨
			sbusIndex = 0;

			if (sbusPacket[24] == 0x00)
			{ // 올바른 엔드 바이트
				parseSBUSPacket();
				lastSbusReceived = currentTime;
				return true;
			}
		}
	}

	// 수신 타임아웃 체크 (100ms)
	if (currentTime - lastSbusReceived > 100)
	{
		sbusFailsafe = true;
		return false;
	}

	return false;
}

void parseSBUSPacket()
{
	// SBUS 패킷 파싱 (11비트 해상도, 16채널)
	sbusChannels[0] = ((sbusPacket[1] | sbusPacket[2] << 8) & 0x07FF);
	sbusChannels[1] = ((sbusPacket[2] >> 3 | sbusPacket[3] << 5) & 0x07FF);
	sbusChannels[2] = ((sbusPacket[3] >> 6 | sbusPacket[4] << 2 | sbusPacket[5] << 10) & 0x07FF);
	sbusChannels[3] = ((sbusPacket[5] >> 1 | sbusPacket[6] << 7) & 0x07FF);
	sbusChannels[4] = ((sbusPacket[6] >> 4 | sbusPacket[7] << 4) & 0x07FF);
	sbusChannels[5] = ((sbusPacket[7] >> 7 | sbusPacket[8] << 1 | sbusPacket[9] << 9) & 0x07FF);
	sbusChannels[6] = ((sbusPacket[9] >> 2 | sbusPacket[10] << 6) & 0x07FF);
	sbusChannels[7] = ((sbusPacket[10] >> 5 | sbusPacket[11] << 3) & 0x07FF);
	sbusChannels[8] = ((sbusPacket[12] | sbusPacket[13] << 8) & 0x07FF);
	sbusChannels[9] = ((sbusPacket[13] >> 3 | sbusPacket[14] << 5) & 0x07FF);
	sbusChannels[10] = ((sbusPacket[14] >> 6 | sbusPacket[15] << 2 | sbusPacket[16] << 10) & 0x07FF);
	sbusChannels[11] = ((sbusPacket[16] >> 1 | sbusPacket[17] << 7) & 0x07FF);
	sbusChannels[12] = ((sbusPacket[17] >> 4 | sbusPacket[18] << 4) & 0x07FF);
	sbusChannels[13] = ((sbusPacket[18] >> 7 | sbusPacket[19] << 1 | sbusPacket[20] << 9) & 0x07FF);
	sbusChannels[14] = ((sbusPacket[20] >> 2 | sbusPacket[21] << 6) & 0x07FF);
	sbusChannels[15] = ((sbusPacket[21] >> 5 | sbusPacket[22] << 3) & 0x07FF);

	// SBUS 값을 마이크로초로 변환 (172~1811 → 1000~2000)
	for (int i = 0; i < 16; i++)
	{
		sbusChannels[i] = MAP(sbusChannels[i], 172, 1811, 1000, 2000);
		sbusChannels[i] = CONSTRAIN(sbusChannels[i], 1000, 2000);
	}

	// Failsafe 및 Frame Lost 상태 체크
	sbusFailsafe = (sbusPacket[23] & 0x08) != 0;
	sbusLostFrame = (sbusPacket[23] & 0x04) != 0;
}

void setFailsafeValues(ControllerInput *input)
{
	// Failsafe 시 안전한 기본값 설정
	input->throttle = 1000; // 최소 스로틀
	input->roll = 1500;		// 중앙
	input->pitch = 1500;	// 중앙
	input->yaw = 1500;		// 중앙
	input->aux1 = 1000;		// DISARM
	input->aux2 = 1000;		// STABILIZE 모드
	input->aux3 = 1000;

	input->throttleNorm = 0.0;
	input->rollNorm = 0.0;
	input->pitchNorm = 0.0;
	input->yawNorm = 0.0;
}

void checkFlightModeSwitch(uint16_t aux2Value)
{
	static uint8_t lastFlightMode = 0;
	uint8_t newFlightMode = 0;

	// AUX2 스위치 위치에 따른 비행 모드 결정
	if (aux2Value < 1300)
	{
		newFlightMode = 0; // STABILIZE
	}
	else if (aux2Value < 1700)
	{
		newFlightMode = 1; // ALTITUDE_HOLD
	}
	else
	{
		newFlightMode = 2; // ACRO
	}

	if (newFlightMode != lastFlightMode)
	{
		setFlightMode(newFlightMode);
		lastFlightMode = newFlightMode;
	}
}

bool isReceiverTimeout()
{
	return (millis() - lastSbusReceived > RECEIVER_TIMEOUT);
}

// 메인 루프에서 주기적으로 호출 필요
void communicationLoop()
{
  // 기존 루프 처리들 ...
  handleWiFiCommands();

  // ✅ 웹 서버 요청/웹소켓 이벤트 처리 + 주기 브로드캐스트
  web::loop();
}

void sendTelemetryData(SensorData *sensorData, ControllerInput *input, float batteryVoltage)
{
	// WiFi 텔레메트리 전송
	if (wifiEnabled)
	{
		sendWiFiTelemetry(sensorData, input, batteryVoltage);
	}

	// VTX OSD 데이터 전송
	if (vtxConnected)
	{
		sendVTXTelemetry(sensorData, batteryVoltage);
	}
}

void sendWiFiTelemetry(SensorData *sensorData, ControllerInput *input, float batteryVoltage)
{
	// 새로운 클라이언트 연결 확인
	if (!telemetryClient.connected())
	{
		telemetryClient = telemetryServer.available();
	}

	if (telemetryClient.connected())
	{
		// JSON 형태로 텔레메트리 데이터 전송
		String telemetryJson = "{";
		telemetryJson += "\"timestamp\":" + String(millis()) + ",";
		telemetryJson += "\"attitude\":{";
		telemetryJson += "\"roll\":" + String(sensorData->roll, 2) + ",";
		telemetryJson += "\"pitch\":" + String(sensorData->pitch, 2) + ",";
		telemetryJson += "\"yaw\":" + String(sensorData->yaw, 2);
		telemetryJson += "},";
		telemetryJson += "\"gyro\":{";
		telemetryJson += "\"x\":" + String(sensorData->gyroX, 1) + ",";
		telemetryJson += "\"y\":" + String(sensorData->gyroY, 1) + ",";
		telemetryJson += "\"z\":" + String(sensorData->gyroZ, 1);
		telemetryJson += "},";
		telemetryJson += "\"input\":{";
		telemetryJson += "\"throttle\":" + String(input->throttleNorm, 2) + ",";
		telemetryJson += "\"roll\":" + String(input->rollNorm, 2) + ",";
		telemetryJson += "\"pitch\":" + String(input->pitchNorm, 2) + ",";
		telemetryJson += "\"yaw\":" + String(input->yawNorm, 2);
		telemetryJson += "},";
		telemetryJson += "\"battery\":" + String(batteryVoltage, 2) + ",";
		telemetryJson += "\"armed\":" + String(systemArmed ? "true" : "false") + ",";
		telemetryJson += "\"mode\":" + String(getFlightMode());
		telemetryJson += "}\n";

		telemetryClient.print(telemetryJson);
	}
}

void sendVTXTelemetry(SensorData *sensorData, float batteryVoltage)
{
	if (!vtxConnected)
		return;

	// Walksnail Avatar HD V2 OSD 데이터 전송
	// MSP (MultiWii Serial Protocol) 형태로 전송

	// 자세 데이터 전송
	sendMSPAttitude(sensorData->roll, sensorData->pitch, sensorData->yaw);

	// 배터리 데이터 전송
	sendMSPBattery(batteryVoltage);

	// GPS 데이터 (더미 - GPS 없음)
	sendMSPGPS();
}

void sendMSPAttitude(float roll, float pitch, float yaw)
{
	uint8_t msp_attitude[12];

	// MSP_ATTITUDE 메시지 구성
	msp_attitude[0] = '$'; // 헤더
	msp_attitude[1] = 'M';
	msp_attitude[2] = '<';
	msp_attitude[3] = 6;   // 데이터 길이
	msp_attitude[4] = 108; // MSP_ATTITUDE 명령

	// 각도 데이터 (0.1도 단위)
	int16_t rollInt = (int16_t)(roll * 10);
	int16_t pitchInt = (int16_t)(pitch * 10);
	int16_t yawInt = (int16_t)(yaw * 10);

	msp_attitude[5] = rollInt & 0xFF;
	msp_attitude[6] = (rollInt >> 8) & 0xFF;
	msp_attitude[7] = pitchInt & 0xFF;
	msp_attitude[8] = (pitchInt >> 8) & 0xFF;
	msp_attitude[9] = yawInt & 0xFF;
	msp_attitude[10] = (yawInt >> 8) & 0xFF;

	// 체크섬 계산
	uint8_t checksum = 0;
	for (int i = 3; i < 11; i++)
	{
		checksum ^= msp_attitude[i];
	}
	msp_attitude[11] = checksum;

	// VTX로 전송
	vtxSerial->write(msp_attitude, 12);
}

void sendMSPBattery(float voltage)
{
	uint8_t msp_battery[10];

	// MSP_ANALOG 메시지 구성
	msp_battery[0] = '$';
	msp_battery[1] = 'M';
	msp_battery[2] = '<';
	msp_battery[3] = 4;	  // 데이터 길이
	msp_battery[4] = 110; // MSP_ANALOG 명령

	// 배터리 전압 (0.1V 단위)
	uint16_t voltageInt = (uint16_t)(voltage * 10);

	msp_battery[5] = voltageInt & 0xFF;
	msp_battery[6] = (voltageInt >> 8) & 0xFF;
	msp_battery[7] = 0; // 전류 (사용 안함)
	msp_battery[8] = 0;

	// 체크섬 계산
	uint8_t checksum = 0;
	for (int i = 3; i < 9; i++)
	{
		checksum ^= msp_battery[i];
	}
	msp_battery[9] = checksum;

	vtxSerial->write(msp_battery, 10);
}

void sendMSPGPS()
{
	uint8_t msp_gps[21];

	// MSP_RAW_GPS 메시지 구성 (더미 데이터)
	msp_gps[0] = '$';
	msp_gps[1] = 'M';
	msp_gps[2] = '<';
	msp_gps[3] = 16;  // 데이터 길이
	msp_gps[4] = 106; // MSP_RAW_GPS 명령

	// GPS 상태 (0 = GPS 없음)
	msp_gps[5] = 0; // GPS fix
	msp_gps[6] = 0; // 위성 수

	// 위도/경도 (0으로 설정)
	for (int i = 7; i < 19; i++)
	{
		msp_gps[i] = 0;
	}

	// 체크섬
	uint8_t checksum = 0;
	for (int i = 3; i < 20; i++)
	{
		checksum ^= msp_gps[i];
	}
	msp_gps[20] = checksum;

	vtxSerial->write(msp_gps, 21);
}

void handleWiFiCommands()
{
	if (!wifiEnabled || !telemetryClient.connected())
	{
		return;
	}

	// WiFi로부터 명령 수신 처리
	if (telemetryClient.available())
	{
		String command = telemetryClient.readStringUntil('\n');
		command.trim();

		if (command.startsWith("PID_ROLL"))
		{
			// Roll PID 튜닝 명령: PID_ROLL,1.2,0.8,0.15
			int firstComma = command.indexOf(',');
			int secondComma = command.indexOf(',', firstComma + 1);
			int thirdComma = command.indexOf(',', secondComma + 1);

			if (firstComma > 0 && secondComma > 0 && thirdComma > 0)
			{
				float kp = command.substring(firstComma + 1, secondComma).toFloat();
				float ki = command.substring(secondComma + 1, thirdComma).toFloat();
				float kd = command.substring(thirdComma + 1).toFloat();

				tuneRollPID(kp, ki, kd);
				telemetryClient.println("Roll PID updated");
			}
		}
		else if (command.startsWith("PID_PITCH"))
		{
			// Pitch PID 튜닝
			int firstComma = command.indexOf(',');
			int secondComma = command.indexOf(',', firstComma + 1);
			int thirdComma = command.indexOf(',', secondComma + 1);

			if (firstComma > 0 && secondComma > 0 && thirdComma > 0)
			{
				float kp = command.substring(firstComma + 1, secondComma).toFloat();
				float ki = command.substring(secondComma + 1, thirdComma).toFloat();
				float kd = command.substring(thirdComma + 1).toFloat();

				tunePitchPID(kp, ki, kd);
				telemetryClient.println("Pitch PID updated");
			}
		}
		else if (command.startsWith("PID_YAW"))
		{
			// Yaw PID 튜닝
			int firstComma = command.indexOf(',');
			int secondComma = command.indexOf(',', firstComma + 1);
			int thirdComma = command.indexOf(',', secondComma + 1);

			if (firstComma > 0 && secondComma > 0 && thirdComma > 0)
			{
				float kp = command.substring(firstComma + 1, secondComma).toFloat();
				float ki = command.substring(secondComma + 1, thirdComma).toFloat();
				float kd = command.substring(thirdComma + 1).toFloat();

				tuneYawPID(kp, ki, kd);
				telemetryClient.println("Yaw PID updated");
			}
		}
		else if (command == "CALIBRATE")
		{
			// 센서 캘리브레이션 명령
			telemetryClient.println("Starting calibration...");
			calibrateSensors();
			telemetryClient.println("Calibration complete");
		}
		else if (command == "RESET_PID")
		{
			// PID 적분항 리셋
			resetPIDIntegrals();
			telemetryClient.println("PID integrals reset");
		}
		else if (command == "STATUS")
		{
			// 시스템 상태 요청
			String status = "ARMED:" + String(systemArmed ? "1" : "0");
			status += ",MODE:" + String(getFlightMode());
			status += ",BATTERY:" + String(readBatteryVoltage(), 2);
			telemetryClient.println(status);
		}
	}
}

void debugPrintSBUS()
{
	static unsigned long lastDebugPrint = 0;
	unsigned long currentTime = millis();

	if (currentTime - lastDebugPrint > 1000)
	{ // 1초마다
		Serial.println("=== SBUS 디버그 ===");
		Serial.print("채널: ");
		for (int i = 0; i < 8; i++)
		{
			Serial.print("CH");
			Serial.print(i + 1);
			Serial.print("=");
			Serial.print(sbusChannels[i]);
			Serial.print(" ");
		}
		Serial.println();
		Serial.print("Failsafe: ");
		Serial.print(sbusFailsafe ? "YES" : "NO");
		Serial.print(", Lost Frame: ");
		Serial.print(sbusLostFrame ? "YES" : "NO");
		Serial.print(", Last RX: ");
		Serial.print(millis() - lastSbusReceived);
		Serial.println("ms ago");
		Serial.println();

		lastDebugPrint = currentTime;
	}
}

// 고급 통신 기능

void setupBlackboxLogging()
{
	// 블랙박스 로깅 설정 (추후 구현)
	// SD카드나 플래시 메모리에 비행 데이터 저장
}

void logFlightData(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs)
{
	// 비행 데이터 로깅 (추후 구현)
	static unsigned long lastLogTime = 0;
	unsigned long currentTime = millis();

	if (currentTime - lastLogTime > 10)
	{ // 100Hz 로깅
		// CSV 형태로 데이터 구성
		String logData = String(currentTime) + ",";
		logData += String(sensorData->roll, 2) + ",";
		logData += String(sensorData->pitch, 2) + ",";
		logData += String(sensorData->yaw, 2) + ",";
		logData += String(sensorData->gyroX, 1) + ",";
		logData += String(sensorData->gyroY, 1) + ",";
		logData += String(sensorData->gyroZ, 1) + ",";
		logData += String(input->throttleNorm, 3) + ",";
		logData += String(input->rollNorm, 3) + ",";
		logData += String(input->pitchNorm, 3) + ",";
		logData += String(input->yawNorm, 3) + ",";
		logData += String(outputs->motor_fl) + ",";
		logData += String(outputs->motor_fr) + ",";
		logData += String(outputs->motor_rl) + ",";
		logData += String(outputs->motor_rr);

		// SD카드나 플래시에 저장 (추후 구현)
		// writeToBlackbox(logData);

		lastLogTime = currentTime;
	}
}

void handleSerialCommands()
{
	// 시리얼 포트를 통한 CLI 명령 처리
	if (Serial.available())
	{
		String command = Serial.readStringUntil('\n');
		command.trim();
		command.toLowerCase();

		if (command == "help")
		{
			Serial.println("=== DIY Shane 드론 CLI ===");
			Serial.println("사용 가능한 명령:");
			Serial.println("  status - 시스템 상태");
			Serial.println("  calibrate - 센서 캘리브레이션");
			Serial.println("  pid_roll <p> <i> <d> - Roll PID 설정");
			Serial.println("  pid_pitch <p> <i> <d> - Pitch PID 설정");
			Serial.println("  pid_yaw <p> <i> <d> - Yaw PID 설정");
			Serial.println("  reset_pid - PID 적분항 리셋");
			Serial.println("  sbus_debug - SBUS 디버그 출력");
			Serial.println("  motor_test - 모터 테스트 (주의!)");
		}
		else if (command == "status")
		{
			Serial.println("=== 시스템 상태 ===");
			Serial.print("ARM 상태: ");
			Serial.println(systemArmed ? "ARMED" : "DISARMED");
			Serial.print("비행 모드: ");
			switch (getFlightMode())
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
			Serial.print("배터리 전압: ");
			Serial.print(readBatteryVoltage(), 2);
			Serial.println("V");
			Serial.print("SBUS 연결: ");
			Serial.println(sbusFailsafe ? "실패" : "정상");
			Serial.print("WiFi 상태: ");
			Serial.println(wifiEnabled ? "활성화" : "비활성화");
		}
		else if (command == "calibrate")
		{
			Serial.println("센서 캘리브레이션 시작...");
			calibrateSensors();
		}
		else if (command == "reset_pid")
		{
			resetPIDIntegrals();
		}
		else if (command == "sbus_debug")
		{
			debugPrintSBUS();
		}
		else if (command.startsWith("pid_roll"))
		{
			// PID 설정 파싱
			int space1 = command.indexOf(' ');
			int space2 = command.indexOf(' ', space1 + 1);
			int space3 = command.indexOf(' ', space2 + 1);

			if (space1 > 0 && space2 > 0 && space3 > 0)
			{
				float kp = command.substring(space1 + 1, space2).toFloat();
				float ki = command.substring(space2 + 1, space3).toFloat();
				float kd = command.substring(space3 + 1).toFloat();
				tuneRollPID(kp, ki, kd);
			}
			else
			{
				Serial.println("사용법: pid_roll <p> <i> <d>");
			}
		}
		else if (command.startsWith("pid_pitch"))
		{
			int space1 = command.indexOf(' ');
			int space2 = command.indexOf(' ', space1 + 1);
			int space3 = command.indexOf(' ', space2 + 1);

			if (space1 > 0 && space2 > 0 && space3 > 0)
			{
				float kp = command.substring(space1 + 1, space2).toFloat();
				float ki = command.substring(space2 + 1, space3).toFloat();
				float kd = command.substring(space3 + 1).toFloat();
				tunePitchPID(kp, ki, kd);
			}
			else
			{
				Serial.println("사용법: pid_pitch <p> <i> <d>");
			}
		}
		else if (command.startsWith("pid_yaw"))
		{
			int space1 = command.indexOf(' ');
			int space2 = command.indexOf(' ', space1 + 1);
			int space3 = command.indexOf(' ', space2 + 1);

			if (space1 > 0 && space2 > 0 && space3 > 0)
			{
				float kp = command.substring(space1 + 1, space2).toFloat();
				float ki = command.substring(space2 + 1, space3).toFloat();
				float kd = command.substring(space3 + 1).toFloat();
				tuneYawPID(kp, ki, kd);
			}
			else
			{
				Serial.println("사용법: pid_yaw <p> <i> <d>");
			}
		}
		else
		{
			Serial.println("알 수 없는 명령. 'help' 입력하여 도움말 확인");
		}
	}
}