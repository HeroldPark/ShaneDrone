// ============== communication.cpp ==============
#include "communication.h"

// Static 멤버 초기화
CommunicationSystem* CommunicationSystem::instance = nullptr;

// 전역 인스턴스
CommunicationSystem commSystem;

CommunicationSystem::CommunicationSystem() :
    sbus(nullptr),
    wifi(nullptr),
    web(nullptr),
    vtx(nullptr),
    cli(nullptr),
    blackbox(nullptr),
    initialized(false),
    lastTelemetryTime(0),
    sensorDataPtr(nullptr),
    inputDataPtr(nullptr),
    motorDataPtr(nullptr),
    batteryPtr(nullptr),
    armedPtr(nullptr),
    flightModePtr(nullptr),
    calibrationCallback(nullptr),
    pidCallback(nullptr),
    motorTestCallback(nullptr) {
    
    instance = this;
    memset(&status, 0, sizeof(status));
}

CommunicationSystem::~CommunicationSystem() {
    // 모듈 정리
    if (sbus) delete sbus;
    if (wifi) delete wifi;
    if (web) delete web;
    if (vtx) delete vtx;
    if (cli) delete cli;
    if (blackbox) delete blackbox;
    
    instance = nullptr;
}

bool CommunicationSystem::begin() {
    Serial.println("[COMM] 통신 시스템 초기화 시작");
    
    // SBUS 수신기 초기화
    Serial.println("[COMM] SBUS 수신기 초기화...");
    sbus = new SBUSReceiver();
    if (sbus->begin(&Serial2, RECEIVER_PIN)) {
        status.sbusConnected = true;
        Serial.println("[COMM] SBUS 초기화 성공");
    } else {
        Serial.println("[COMM] SBUS 초기화 실패");
    }
    
    // WiFi 텔레메트리 초기화 (선택적)
    #ifdef ENABLE_WIFI_TELEMETRY
    Serial.println("[COMM] WiFi 텔레메트리 초기화...");
    wifi = new WiFiTelemetry();
    if (wifi->begin(WIFI_SSID, WIFI_PASSWORD)) {
        status.wifiEnabled = true;
        Serial.println("[COMM] WiFi 초기화 성공");
    }
    #endif
    
    // 웹 인터페이스 초기화 (선택적)
    #ifdef ENABLE_WEB_INTERFACE
    Serial.println("[COMM] 웹 인터페이스 초기화...");
    web = new WebInterface();
    if (web->begin()) {
        web->setCommandCallback(handleWebCommand);
        status.webServerRunning = true;
        Serial.println("[COMM] 웹 서버 시작됨");
    }
    #endif
    
    // VTX OSD 초기화 (선택적)
    #ifdef ENABLE_VTX_OSD
    Serial.println("[COMM] VTX OSD 초기화...");
    vtx = new VTXOSD();
    if (vtx->begin(&Serial1, VTX_RX_PIN, VTX_TX_PIN)) {
        status.vtxConnected = true;
        Serial.println("[COMM] VTX 연결됨");
    }
    #endif
    
    // Serial CLI 초기화
    Serial.println("[COMM] Serial CLI 초기화...");
    cli = new SerialCLI();
    cli->begin(&Serial, 115200);
    
    // CLI 콜백 설정
    cli->getStatusCallback = handleCLIStatus;
    cli->calibrateCallback = handleCLICalibrate;
    cli->setPIDCallback = handleCLIPID;
    cli->motorTestCallback = handleCLIMotorTest;
    
    // Blackbox 로거 초기화 (선택적)
    #ifdef ENABLE_BLACKBOX
    Serial.println("[COMM] Blackbox 로거 초기화...");
    blackbox = new BlackboxLogger();
    if (blackbox->begin(SD_CS_PIN)) {
        Serial.println("[COMM] Blackbox 초기화 성공");
    }
    #endif
    
    initialized = true;
    status.lastUpdate = millis();
    
    Serial.println("[COMM] 통신 시스템 초기화 완료");
    return true;
}

void CommunicationSystem::setDataPointers(SensorData* sensors, ControllerInput* input,
                                         MotorOutputs* motors, float* battery,
                                         bool* armed, uint8_t* mode) {
    sensorDataPtr = sensors;
    inputDataPtr = input;
    motorDataPtr = motors;
    batteryPtr = battery;
    armedPtr = armed;
    flightModePtr = mode;
    
    // 웹 인터페이스에도 포인터 전달
    if (web) {
        web->setDataPointers(sensors, input, motors, battery, armed, mode);
    }
}

void CommunicationSystem::loop() {
    if (!initialized) return;
    
    unsigned long currentTime = millis();
    
    // SBUS 데이터 읽기
    if (sbus) {
        sbus->read();
        status.sbusConnected = sbus->isConnected();
    }
    
    // Serial CLI 처리
    if (cli) {
        cli->loop();
    }
    
    // 웹 서버 처리
    if (web && status.webServerRunning) {
        web->loop();
    }
    
    // 텔레메트리 업데이트 (50Hz)
    if (currentTime - lastTelemetryTime > 20) {
        updateTelemetry();
        lastTelemetryTime = currentTime;
    }
    
    // Failsafe 체크
    checkFailsafe();
    
    status.lastUpdate = currentTime;
}

void CommunicationSystem::readReceiver(ControllerInput* input) {
    if (!sbus || !input) return;
    
    const SBUSData& sbusData = sbus->getData();
    
    if (sbusData.connected && !sbusData.failsafe) {
        // SBUS 채널을 ControllerInput으로 변환
        input->throttle = sbusData.channels[CH_THROTTLE - 1];
        input->roll = sbusData.channels[CH_ROLL - 1];
        input->pitch = sbusData.channels[CH_PITCH - 1];
        input->yaw = sbusData.channels[CH_YAW - 1];
        input->aux1 = sbusData.channels[CH_AUX1 - 1];
        input->aux2 = sbusData.channels[CH_AUX2 - 1];
        input->aux3 = sbusData.channels[CH_AUX3 - 1];
        
        // 정규화된 값 계산 (-1 ~ 1)
        input->throttleNorm = (input->throttle - 1000) / 1000.0f;
        input->rollNorm = (input->roll - 1500) / 500.0f;
        input->pitchNorm = (input->pitch - 1500) / 500.0f;
        input->yawNorm = (input->yaw - 1500) / 500.0f;
        
        input->receiverConnected = true;
        input->lastUpdate = millis();
    } else {
        // Failsafe 값 설정
        input->throttle = 1000;
        input->roll = 1500;
        input->pitch = 1500;
        input->yaw = 1500;
        input->aux1 = 1000;
        input->aux2 = 1000;
        input->aux3 = 1000;
        
        input->throttleNorm = 0.0f;
        input->rollNorm = 0.0f;
        input->pitchNorm = 0.0f;
        input->yawNorm = 0.0f;
        
        input->receiverConnected = false;
    }
}

bool CommunicationSystem::isReceiverConnected() const {
    return sbus ? sbus->isConnected() : false;
}

void CommunicationSystem::updateTelemetry() {
    if (!sensorDataPtr || !inputDataPtr || !motorDataPtr) return;
    
    // WiFi 텔레메트리
    if (wifi && status.wifiEnabled) {
        wifi->sendTelemetry(sensorDataPtr, inputDataPtr, motorDataPtr,
                           batteryPtr ? *batteryPtr : 0,
                           armedPtr ? *armedPtr : false,
                           flightModePtr ? *flightModePtr : 0);
    }
    
    // VTX OSD
    if (vtx && status.vtxConnected) {
        vtx->updateTelemetry(sensorDataPtr,
                            batteryPtr ? *batteryPtr : 0,
                            armedPtr ? *armedPtr : false,
                            flightModePtr ? *flightModePtr : 0);
    }
    
    // Blackbox 로깅
    if (blackbox && status.blackboxLogging) {
        blackbox->logData(sensorDataPtr, inputDataPtr, motorDataPtr,
                         batteryPtr ? *batteryPtr : 0,
                         armedPtr ? *armedPtr : false,
                         flightModePtr ? *flightModePtr : 0);
    }
}

void CommunicationSystem::sendTelemetryData() {
    updateTelemetry();
}

void CommunicationSystem::checkFailsafe() {
    if (!sbus) return;
    
    if (!sbus->isConnected()) {
        // 수신기 연결 끊김 - Failsafe 동작
        if (inputDataPtr) {
            inputDataPtr->receiverConnected = false;
        }
    }
}

void CommunicationSystem::enableWiFi(bool enable) {
    if (enable && !status.wifiEnabled) {
        if (!wifi) {
            wifi = new WiFiTelemetry();
        }
        if (wifi->begin(WIFI_SSID, WIFI_PASSWORD)) {
            status.wifiEnabled = true;
        }
    } else if (!enable && status.wifiEnabled) {
        if (wifi) {
            wifi->stop();
            status.wifiEnabled = false;
        }
    }
}

void CommunicationSystem::enableWebServer(bool enable) {
    if (enable && !status.webServerRunning) {
        if (!web) {
            web = new WebInterface();
        }
        if (web->begin()) {
            web->setCommandCallback(handleWebCommand);
            web->setDataPointers(sensorDataPtr, inputDataPtr, motorDataPtr,
                               batteryPtr, armedPtr, flightModePtr);
            status.webServerRunning = true;
        }
    } else if (!enable && status.webServerRunning) {
        if (web) {
            web->stop();
            status.webServerRunning = false;
        }
    }
}

void CommunicationSystem::enableVTX(bool enable) {
    if (enable && !status.vtxConnected) {
        if (!vtx) {
            vtx = new VTXOSD();
        }
        if (vtx->begin(&Serial1, VTX_RX_PIN, VTX_TX_PIN)) {
            status.vtxConnected = true;
        }
    } else if (!enable && status.vtxConnected) {
        if (vtx) {
            vtx->disconnect();
            status.vtxConnected = false;
        }
    }
}

void CommunicationSystem::startBlackboxLogging() {
    if (blackbox && !status.blackboxLogging) {
        if (blackbox->startLogging()) {
            status.blackboxLogging = true;
            Serial.println("[COMM] Blackbox 로깅 시작");
        }
    }
}

void CommunicationSystem::stopBlackboxLogging() {
    if (blackbox && status.blackboxLogging) {
        blackbox->stopLogging();
        status.blackboxLogging = false;
        Serial.println("[COMM] Blackbox 로깅 중지");
    }
}

void CommunicationSystem::printStatus() {
    Serial.println("=== 통신 시스템 상태 ===");
    Serial.print("SBUS: ");
    Serial.println(status.sbusConnected ? "연결됨" : "연결 안됨");
    Serial.print("WiFi: ");
    Serial.println(status.wifiEnabled ? "활성화" : "비활성화");
    Serial.print("웹 서버: ");
    Serial.println(status.webServerRunning ? "실행중" : "중지");
    Serial.print("VTX: ");
    Serial.println(status.vtxConnected ? "연결됨" : "연결 안됨");
    Serial.print("Blackbox: ");
    Serial.println(status.blackboxLogging ? "로깅중" : "대기");
}

void CommunicationSystem::setCalibrationCallback(void (*callback)()) {
    calibrationCallback = callback;
}

void CommunicationSystem::setPIDCallback(void (*callback)(const char*, float, float, float)) {
    pidCallback = callback;
}

void CommunicationSystem::setMotorTestCallback(void (*callback)(uint8_t, uint16_t)) {
    motorTestCallback = callback;
}

// Static 콜백 함수들
void CommunicationSystem::handleWebCommand(const String& command, const JsonDocument& params) {
    if (!instance) return;
    
    if (command == "CALIBRATE") {
        if (instance->calibrationCallback) {
            instance->calibrationCallback();
        }
    } else if (command == "SET_PID") {
        if (instance->pidCallback) {
            const char* axis = params["axis"];
            float kp = params["kp"];
            float ki = params["ki"];
            float kd = params["kd"];
            instance->pidCallback(axis, kp, ki, kd);
        }
    } else if (command == "MOTOR_TEST") {
        if (instance->motorTestCallback) {
            uint8_t motor = params["motor"];
            uint16_t value = params["value"];
            instance->motorTestCallback(motor, value);
        }
    } else if (command == "START_LOG") {
        instance->startBlackboxLogging();
    } else if (command == "STOP_LOG") {
        instance->stopBlackboxLogging();
    }
}

void CommunicationSystem::handleCLIStatus() {
    if (instance) {
        instance->printStatus();
    }
}

void CommunicationSystem::handleCLICalibrate() {
    if (instance && instance->calibrationCallback) {
        instance->calibrationCallback();
    }
}

void CommunicationSystem::handleCLIPID(const char* axis, float kp, float ki, float kd) {
    if (instance && instance->pidCallback) {
        instance->pidCallback(axis, kp, ki, kd);
    }
}

void CommunicationSystem::handleCLIMotorTest(uint8_t motor, uint16_t value) {
    if (instance && instance->motorTestCallback) {
        instance->motorTestCallback(motor, value);
    }
}
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "control.h"

// 통신 모듈들
#include "sbus_receiver.h"
#include "wifi_telemetry.h"
#include "web_interface.h"
#include "vtx_osd.h"
#include "serial_cli.h"
#include "blackbox_logger.h"

// 통신 시스템 설정
#define COMM_UPDATE_RATE 50  // Hz
#define FAILSAFE_TIMEOUT 500  // ms

// 통신 시스템 상태
struct CommunicationStatus {
    bool sbusConnected;
    bool wifiEnabled;
    bool webServerRunning;
    bool vtxConnected;
    bool blackboxLogging;
    unsigned long lastUpdate;
};

class CommunicationSystem {
private:
    // 모듈 인스턴스
    SBUSReceiver* sbus;
    WiFiTelemetry* wifi;
    WebInterface* web;
    VTXOSD* vtx;
    SerialCLI* cli;
    BlackboxLogger* blackbox;
    
    // 상태
    CommunicationStatus status;
    bool initialized;
    unsigned long lastTelemetryTime;
    
    // 데이터 포인터 (외부 참조)
    SensorData* sensorDataPtr;
    ControllerInput* inputDataPtr;
    MotorOutputs* motorDataPtr;
    float* batteryPtr;
    bool* armedPtr;
    uint8_t* flightModePtr;
    
    // 내부 함수
    void processReceiverData();
    void updateTelemetry();
    void checkFailsafe();
    
    // 명령 콜백들
    static CommunicationSystem* instance;
    static void handleWebCommand(const String& command, const JsonDocument& params);
    static void handleCLIStatus();
    static void handleCLICalibrate();
    static void handleCLIPID(const char* axis, float kp, float ki, float kd);
    static void handleCLIMotorTest(uint8_t motor, uint16_t value);
    
public:
    CommunicationSystem();
    ~CommunicationSystem();
    
    // 초기화
    bool begin();
    void setDataPointers(SensorData* sensors, ControllerInput* input,
                        MotorOutputs* motors, float* battery,
                        bool* armed, uint8_t* mode);
    
    //