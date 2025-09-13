// ============== web_interface.h ==============
#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "sensors.h"
#include "control.h"

// 웹 서버 설정
#define WEB_SERVER_PORT 80
#define WEBSOCKET_PORT 81
#define BROADCAST_INTERVAL 100 // ms

// 명령 콜백 타입
typedef void (*CommandCallback)(const String& command, const JsonDocument& params);

class WebInterface {
private:
    WebServer* webServer;
    WebSocketsServer* webSocket;
    bool running;
    unsigned long lastBroadcast;
    
    // 콜백 함수들
    CommandCallback commandCallback;
    
    // HTML 페이지 생성
    String generateDashboardHTML();
    String generateConfigHTML();
    
    // 핸들러 함수들 (static으로 선언)
    static WebInterface* instance;
    static void handleRoot();
    static void handleTelemetry();
    static void handleCommand();
    static void handleConfig();
    static void handleNotFound();
    static void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    
    // 데이터 포인터 (broadcastTelemetry용)
    const SensorData* sensorDataPtr;
    const ControllerInput* inputDataPtr;
    const MotorOutputs* motorDataPtr;
    float* batteryPtr;
    bool* armedPtr;
    uint8_t* flightModePtr;
    
public:
    WebInterface();
    ~WebInterface();
    
    bool begin();
    void stop();
    void loop();
    
    // 데이터 포인터 설정
    void setDataPointers(const SensorData* sensors, const ControllerInput* input,
                        const MotorOutputs* motors, float* battery, 
                        bool* armed, uint8_t* mode);
    
    // 콜백 설정
    void setCommandCallback(CommandCallback callback) { commandCallback = callback; }
    
    // 텔레메트리 브로드캐스트
    void broadcastTelemetry();
    void broadcastMessage(const String& message);
    
    bool isRunning() const { return running; }
    int getConnectedClients() const;
};

#endif // WEB_INTERFACE_H