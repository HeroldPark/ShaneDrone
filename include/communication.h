/*
 * communication.h - 통신 관련 함수 선언
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
// #include <WebSocketsServer.h>
#include <ArduinoJson.h>

// 통신 시스템 초기화
bool initializeCommunication();
bool initializeSBUS();
bool initializeWiFi();
bool initializeVTX();

// 웹 서버 관련 함수 추가
void setupWebServer();
void handleRoot();
void handleTelemetryData();
void handleCommand();
void handleNotFound();
void broadcastTelemetry();
// void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// 수신기 데이터 처리
void readReceiverData(ControllerInput *input);
bool readSBUSData();
void parseSBUSPacket();
void setFailsafeValues(ControllerInput *input);
void checkFlightModeSwitch(uint16_t aux2Value);
bool isReceiverTimeout();

// 텔레메트리 전송
void sendTelemetryData(SensorData *sensorData, ControllerInput *input, float batteryVoltage);
void sendWiFiTelemetry(SensorData *sensorData, ControllerInput *input, float batteryVoltage);
void sendVTXTelemetry(SensorData *sensorData, float batteryVoltage);

// MSP 프로토콜 (VTX용)
void sendMSPAttitude(float roll, float pitch, float yaw);
void sendMSPBattery(float voltage);
void sendMSPGPS();

// 명령 처리
void handleWiFiCommands();
void handleSerialCommands();

// 디버그 및 로깅
void debugPrintSBUS();
void setupBlackboxLogging();
void logFlightData(SensorData *sensorData, ControllerInput *input, MotorOutputs *outputs);

// 전역 변수 선언 (extern)
extern bool systemArmed;
extern WebServer webServer;
// extern WebSocketsServer webSocket;

// 전역 데이터 변수들 추가
extern SensorData sensorData;
extern ControllerInput controllerInput;
extern MotorOutputs motorOutputs;
extern float batteryVoltage;

#endif // COMMUNICATION_H