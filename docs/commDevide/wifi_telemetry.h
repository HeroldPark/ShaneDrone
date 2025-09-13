// ============== wifi_telemetry.h ==============
#ifndef WIFI_TELEMETRY_H
#define WIFI_TELEMETRY_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "config.h"
#include "sensors.h"
#include "control.h"

// WiFi 설정
#define TELEMETRY_PORT 8080
#define TELEMETRY_UPDATE_RATE 50 // ms

// 텔레메트리 데이터 구조체
struct TelemetryPacket {
    // 자세 데이터
    float roll;
    float pitch;
    float yaw;
    
    // 자이로 데이터
    float gyroX;
    float gyroY;
    float gyroZ;
    
    // 가속도 데이터
    float accX;
    float accY;
    float accZ;
    
    // 시스템 상태
    bool armed;
    uint8_t flightMode;
    float batteryVoltage;
    
    // 조종 입력
    float throttle;
    float rollInput;
    float pitchInput;
    float yawInput;
    
    // 모터 출력
    uint16_t motorFL;
    uint16_t motorFR;
    uint16_t motorRL;
    uint16_t motorRR;
    
    // 타임스탬프
    unsigned long timestamp;
};

class WiFiTelemetry {
private:
    WiFiServer* server;
    WiFiClient client;
    bool enabled;
    unsigned long lastSendTime;
    IPAddress apIP;
    
    String createJsonPacket(const TelemetryPacket& packet);
    bool parseCommand(const String& command);
    
public:
    WiFiTelemetry();
    bool begin(const char* ssid, const char* password);
    void stop();
    bool isEnabled() const { return enabled; }
    bool isClientConnected() const { return client.connected(); }
    
    void sendTelemetry(const TelemetryPacket& packet);
    void sendTelemetry(const SensorData* sensors, const ControllerInput* input, 
                      const MotorOutputs* motors, float battery, bool armed, uint8_t mode);
    
    bool receiveCommands();
    String getLastCommand();
    
    IPAddress getIP() const { return apIP; }
    int getConnectedClients() const;
};

#endif // WIFI_TELEMETRY_H