// ============== wifi_telemetry.cpp ==============
#include "wifi_telemetry.h"

WiFiTelemetry::WiFiTelemetry() : 
    server(nullptr), 
    enabled(false), 
    lastSendTime(0) {
}

bool WiFiTelemetry::begin(const char* ssid, const char* password) {
    Serial.println("[WiFi] 텔레메트리 초기화 시작");
    
    // WiFi AP 모드 설정
    WiFi.mode(WIFI_AP);
    
    if (!WiFi.softAP(ssid, password)) {
        Serial.println("[WiFi] AP 생성 실패");
        return false;
    }
    
    apIP = WiFi.softAPIP();
    Serial.print("[WiFi] AP 생성됨: ");
    Serial.println(ssid);
    Serial.print("[WiFi] IP 주소: ");
    Serial.println(apIP);
    
    // 텔레메트리 서버 시작
    server = new WiFiServer(TELEMETRY_PORT);
    server->begin();
    
    enabled = true;
    Serial.println("[WiFi] 텔레메트리 서버 시작됨");
    
    return true;
}

void WiFiTelemetry::stop() {
    if (server) {
        server->stop();
        delete server;
        server = nullptr;
    }
    
    WiFi.softAPdisconnect(true);
    enabled = false;
    
    Serial.println("[WiFi] 텔레메트리 중지됨");
}

void WiFiTelemetry::sendTelemetry(const TelemetryPacket& packet) {
    if (!enabled) return;
    
    // 새 클라이언트 확인
    if (!client.connected()) {
        client = server->available();
        if (client) {
            Serial.println("[WiFi] 새 클라이언트 연결됨");
        }
    }
    
    // 전송 주기 제한
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime < TELEMETRY_UPDATE_RATE) {
        return;
    }
    
    if (client.connected()) {
        String json = createJsonPacket(packet);
        client.println(json);
        lastSendTime = currentTime;
    }
}

void WiFiTelemetry::sendTelemetry(const SensorData* sensors, const ControllerInput* input,
                                  const MotorOutputs* motors, float battery, bool armed, uint8_t mode) {
    TelemetryPacket packet;
    
    // 센서 데이터 복사
    packet.roll = sensors->roll;
    packet.pitch = sensors->pitch;
    packet.yaw = sensors->yaw;
    packet.gyroX = sensors->gyroX;
    packet.gyroY = sensors->gyroY;
    packet.gyroZ = sensors->gyroZ;
    packet.accX = sensors->accX;
    packet.accY = sensors->accY;
    packet.accZ = sensors->accZ;
    
    // 입력 데이터
    packet.throttle = input->throttleNorm;
    packet.rollInput = input->rollNorm;
    packet.pitchInput = input->pitchNorm;
    packet.yawInput = input->yawNorm;
    
    // 모터 출력
    packet.motorFL = motors->motor_fl;
    packet.motorFR = motors->motor_fr;
    packet.motorRL = motors->motor_rl;
    packet.motorRR = motors->motor_rr;
    
    // 시스템 상태
    packet.armed = armed;
    packet.flightMode = mode;
    packet.batteryVoltage = battery;
    packet.timestamp = millis();
    
    sendTelemetry(packet);
}

String WiFiTelemetry::createJsonPacket(const TelemetryPacket& packet) {
    StaticJsonDocument<512> doc;
    
    // 자세
    JsonObject attitude = doc.createNestedObject("attitude");
    attitude["roll"] = packet.roll;
    attitude["pitch"] = packet.pitch;
    attitude["yaw"] = packet.yaw;
    
    // 자이로
    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = packet.gyroX;
    gyro["y"] = packet.gyroY;
    gyro["z"] = packet.gyroZ;
    
    // 가속도
    JsonObject acc = doc.createNestedObject("acc");
    acc["x"] = packet.accX;
    acc["y"] = packet.accY;
    acc["z"] = packet.accZ;
    
    // 입력
    JsonObject input = doc.createNestedObject("input");
    input["throttle"] = packet.throttle;
    input["roll"] = packet.rollInput;
    input["pitch"] = packet.pitchInput;
    input["yaw"] = packet.yawInput;
    
    // 모터
    JsonObject motors = doc.createNestedObject("motors");
    motors["fl"] = packet.motorFL;
    motors["fr"] = packet.motorFR;
    motors["rl"] = packet.motorRL;
    motors["rr"] = packet.motorRR;
    
    // 상태
    doc["armed"] = packet.armed;
    doc["mode"] = packet.flightMode;
    doc["battery"] = packet.batteryVoltage;
    doc["timestamp"] = packet.timestamp;
    
    String output;
    serializeJson(doc, output);
    return output;
}

bool WiFiTelemetry::receiveCommands() {
    if (!enabled || !client.connected()) return false;
    
    if (client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        return parseCommand(command);
    }
    
    return false;
}

bool WiFiTelemetry::parseCommand(const String& command) {
    // 명령 파싱 로직
    if (command.startsWith("PID_")) {
        // PID 튜닝 명령
        Serial.print("[WiFi] PID 명령 수신: ");
        Serial.println(command);
        
        // PID 값 추출 및 처리는 메인 모듈에서
        return true;
    }
    else if (command == "CALIBRATE") {
        Serial.println("[WiFi] 캘리브레이션 명령 수신");
        return true;
    }
    else if (command == "STATUS") {
        // 상태 요청
        client.println("OK");
        return true;
    }
    
    return false;
}

String WiFiTelemetry::getLastCommand() {
    if (!client.connected()) return "";
    
    if (client.available()) {
        return client.readStringUntil('\n');
    }
    
    return "";
}

int WiFiTelemetry::getConnectedClients() const {
    return WiFi.softAPgetStationNum();
}