// ============== web_interface.cpp ==============
#include "web_interface.h"

// Static 멤버 초기화
WebInterface* WebInterface::instance = nullptr;

WebInterface::WebInterface() : 
    webServer(nullptr),
    webSocket(nullptr),
    running(false),
    lastBroadcast(0),
    commandCallback(nullptr),
    sensorDataPtr(nullptr),
    inputDataPtr(nullptr),
    motorDataPtr(nullptr),
    batteryPtr(nullptr),
    armedPtr(nullptr),
    flightModePtr(nullptr) {
    instance = this;
}

WebInterface::~WebInterface() {
    stop();
    instance = nullptr;
}

bool WebInterface::begin() {
    Serial.println("[Web] 인터페이스 시작");
    
    // 웹 서버 초기화
    webServer = new WebServer(WEB_SERVER_PORT);
    
    // 라우트 설정
    webServer->on("/", handleRoot);
    webServer->on("/telemetry", handleTelemetry);
    webServer->on("/command", HTTP_POST, handleCommand);
    webServer->on("/config", handleConfig);
    webServer->onNotFound(handleNotFound);
    
    // CORS 활성화
    webServer->enableCORS(true);
    
    // 웹 서버 시작
    webServer->begin();
    Serial.print("[Web] HTTP 서버 시작 (포트 ");
    Serial.print(WEB_SERVER_PORT);
    Serial.println(")");
    
    // WebSocket 서버 시작
    webSocket = new WebSocketsServer(WEBSOCKET_PORT);
    webSocket->begin();
    webSocket->onEvent(onWebSocketEvent);
    Serial.print("[Web] WebSocket 서버 시작 (포트 ");
    Serial.print(WEBSOCKET_PORT);
    Serial.println(")");
    
    running = true;
    return true;
}

void WebInterface::stop() {
    if (webServer) {
        webServer->stop();
        delete webServer;
        webServer = nullptr;
    }
    
    if (webSocket) {
        webSocket->close();
        delete webSocket;
        webSocket = nullptr;
    }
    
    running = false;
    Serial.println("[Web] 인터페이스 중지됨");
}

void WebInterface::loop() {
    if (!running) return;
    
    webServer->handleClient();
    webSocket->loop();
    
    // 자동 브로드캐스트
    if (millis() - lastBroadcast > BROADCAST_INTERVAL) {
        broadcastTelemetry();
        lastBroadcast = millis();
    }
}

void WebInterface::setDataPointers(const SensorData* sensors, const ControllerInput* input,
                                   const MotorOutputs* motors, float* battery,
                                   bool* armed, uint8_t* mode) {
    sensorDataPtr = sensors;
    inputDataPtr = input;
    motorDataPtr = motors;
    batteryPtr = battery;
    armedPtr = armed;
    flightModePtr = mode;
}

void WebInterface::broadcastTelemetry() {
    if (!webSocket || webSocket->connectedClients() == 0) return;
    if (!sensorDataPtr || !inputDataPtr || !motorDataPtr) return;
    
    StaticJsonDocument<512> doc;
    
    // 자세 데이터
    JsonObject attitude = doc.createNestedObject("attitude");
    attitude["roll"] = sensorDataPtr->roll;
    attitude["pitch"] = sensorDataPtr->pitch;
    attitude["yaw"] = sensorDataPtr->yaw;
    
    // 자이로 데이터
    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = sensorDataPtr->gyroX;
    gyro["y"] = sensorDataPtr->gyroY;
    gyro["z"] = sensorDataPtr->gyroZ;
    
    // 모터 출력
    JsonObject motors = doc.createNestedObject("motors");
    motors["fl"] = motorDataPtr->motor_fl;
    motors["fr"] = motorDataPtr->motor_fr;
    motors["rl"] = motorDataPtr->motor_rl;
    motors["rr"] = motorDataPtr->motor_rr;
    
    // 입력값
    JsonObject input = doc.createNestedObject("input");
    input["throttle"] = inputDataPtr->throttleNorm;
    input["roll"] = inputDataPtr->rollNorm;
    input["pitch"] = inputDataPtr->pitchNorm;
    input["yaw"] = inputDataPtr->yawNorm;
    
    // 시스템 상태
    if (batteryPtr) doc["battery"] = *batteryPtr;
    if (armedPtr) doc["armed"] = *armedPtr;
    if (flightModePtr) doc["mode"] = *flightModePtr;
    
    doc["timestamp"] = millis();
    
    String output;
    serializeJson(doc, output);
    webSocket->broadcastTXT(output);
}

void WebInterface::broadcastMessage(const String& message) {
    if (webSocket) {
        webSocket->broadcastTXT(message);
    }
}

int WebInterface::getConnectedClients() const {
    return webSocket ? webSocket->connectedClients() : 0;
}

// Static 핸들러 함수들
void WebInterface::handleRoot() {
    if (!instance) return;
    instance->webServer->send(200, "text/html", instance->generateDashboardHTML());
}

void WebInterface::handleTelemetry() {
    if (!instance) return;
    
    StaticJsonDocument<512> doc;
    
    if (instance->sensorDataPtr) {
        doc["roll"] = instance->sensorDataPtr->roll;
        doc["pitch"] = instance->sensorDataPtr->pitch;
        doc["yaw"] = instance->sensorDataPtr->yaw;
    }
    
    if (instance->batteryPtr) {
        doc["battery"] = *instance->batteryPtr;
    }
    
    if (instance->armedPtr) {
        doc["armed"] = *instance->armedPtr;
    }
    
    doc["timestamp"] = millis();
    
    String response;
    serializeJson(doc, response);
    instance->webServer->send(200, "application/json", response);
}

void WebInterface::handleCommand() {
    if (!instance || !instance->webServer->hasArg("plain")) {
        instance->webServer->send(400, "text/plain", "잘못된 요청");
        return;
    }
    
    StaticJsonDocument<256> doc;
    deserializeJson(doc, instance->webServer->arg("plain"));
    
    String command = doc["command"];
    
    if (instance->commandCallback) {
        instance->commandCallback(command, doc);
        instance->webServer->send(200, "text/plain", "명령 처리됨");
    } else {
        instance->webServer->send(500, "text/plain", "명령 핸들러 없음");
    }
}

void WebInterface::handleConfig() {
    if (!instance) return;
    instance->webServer->send(200, "text/html", instance->generateConfigHTML());
}

void WebInterface::handleNotFound() {
    if (!instance) return;
    instance->webServer->send(404, "text/plain", "페이지를 찾을 수 없습니다");
}

String WebInterface::generateDashboardHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Shane 드론 대시보드</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e1e2e 0%, #2a2a3e 100%);
            color: white; padding: 20px; min-height: 100vh;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .status-grid { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); 
            gap: 20px; 
        }
        .card { 
            background: rgba(255,255,255,0.1); 
            border-radius: 15px; 
            padding: 20px;
            backdrop-filter: blur(10px); 
            border: 1px solid rgba(255,255,255,0.2);
        }
        .card h3 { color: #4fc3f7; margin-bottom: 15px; }
        .data-row { 
            display: flex; 
            justify-content: space-between; 
            margin: 8px 0; 
        }
        .status-indicator { 
            display: inline-block; 
            width: 12px; 
            height: 12px;
            border-radius: 50%; 
            margin-right: 8px;
        }
        .status-connected { background: #44ff44; }
        .status-disconnected { background: #ff4444; }
        .button { 
            background: #4fc3f7; 
            border: none; 
            padding: 10px 20px;
            border-radius: 8px; 
            color: white; 
            cursor: pointer; 
            margin: 5px;
            font-size: 14px;
        }
        .button:hover { background: #29b6f6; }
        .button.danger { background: #ff4444; }
        .button.danger:hover { background: #ff6666; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Shane 드론 대시보드</h1>
            <p>실시간 상태 모니터링</p>
            <div>
                <span class="status-indicator" id="wsStatus"></span>
                <span id="wsStatusText">연결 중...</span>
            </div>
        </div>
        
        <div class="status-grid">
            <div class="card">
                <h3>자세 정보</h3>
                <div class="data-row">
                    <span>Roll:</span>
                    <span id="roll">0.0°</span>
                </div>
                <div class="data-row">
                    <span>Pitch:</span>
                    <span id="pitch">0.0°</span>
                </div>
                <div class="data-row">
                    <span>Yaw:</span>
                    <span id="yaw">0.0°</span>
                </div>
            </div>
            
            <div class="card">
                <h3>시스템 상태</h3>
                <div class="data-row">
                    <span>ARM:</span>
                    <span id="armStatus">DISARMED</span>
                </div>
                <div class="data-row">
                    <span>비행모드:</span>
                    <span id="flightMode">STABILIZE</span>
                </div>
                <div class="data-row">
                    <span>배터리:</span>
                    <span id="battery">0.0V</span>
                </div>
            </div>
            
            <div class="card">
                <h3>모터 출력</h3>
                <div class="data-row">
                    <span>FL:</span>
                    <span id="motorFL">1000</span>
                </div>
                <div class="data-row">
                    <span>FR:</span>
                    <span id="motorFR">1000</span>
                </div>
                <div class="data-row">
                    <span>RL:</span>
                    <span id="motorRL">1000</span>
                </div>
                <div class="data-row">
                    <span>RR:</span>
                    <span id="motorRR">1000</span>
                </div>
            </div>
        </div>
        
        <div style="margin-top: 20px; text-align: center;">
            <button class="button" onclick="sendCommand('CALIBRATE')">센서 캘리브레이션</button>
            <button class="button" onclick="sendCommand('RESET_PID')">PID 리셋</button>
            <button class="button danger" onclick="sendCommand('EMERGENCY_STOP')">비상 정지</button>
        </div>
    </div>
    
    <script>
        let ws;
        
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            
            ws.onopen = function() {
                document.getElementById('wsStatus').className = 'status-indicator status-connected';
                document.getElementById('wsStatusText').textContent = '연결됨';
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateDisplay(data);
            };
            
            ws.onclose = function() {
                document.getElementById('wsStatus').className = 'status-indicator status-disconnected';
                document.getElementById('wsStatusText').textContent = '연결 끊김';
                setTimeout(connectWebSocket, 3000);
            };
        }
        
        function updateDisplay(data) {
            if (data.attitude) {
                document.getElementById('roll').textContent = data.attitude.roll.toFixed(1) + '°';
                document.getElementById('pitch').textContent = data.attitude.pitch.toFixed(1) + '°';
                document.getElementById('yaw').textContent = data.attitude.yaw.toFixed(1) + '°';
            }
            
            if (data.motors) {
                document.getElementById('motorFL').textContent = data.motors.fl;
                document.getElementById('motorFR').textContent = data.motors.fr;
                document.getElementById('motorRL').textContent = data.motors.rl;
                document.getElementById('motorRR').textContent = data.motors.rr;
            }
            
            if (data.battery !== undefined) {
                document.getElementById('battery').textContent = data.battery.toFixed(2) + 'V';
            }
            
            if (data.armed !== undefined) {
                document.getElementById('armStatus').textContent = data.armed ? 'ARMED' : 'DISARMED';
            }
            
            if (data.mode !== undefined) {
                const modes = ['STABILIZE', 'ALTITUDE_HOLD', 'ACRO'];
                document.getElementById('flightMode').textContent = modes[data.mode] || 'UNKNOWN';
            }
        }
        
        function sendCommand(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({command: cmd}));
            } else {
                alert('WebSocket이 연결되지 않았습니다');
            }
        }
        
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";
}

String WebInterface::generateConfigHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>드론 설정</title>
</head>
<body>
    <h1>PID 설정</h1>
    <p>구현 예정</p>
</body>
</html>
)rawliteral";
}

void WebInterface::onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WebSocket] 클라이언트 [%u] 연결 끊김\n", num);
            break;
            
        case WStype_CONNECTED: {
            IPAddress ip = instance->webSocket->remoteIP(num);
            Serial.printf("[WebSocket] 클라이언트 [%u] 연결됨: %s\n", num, ip.toString().c_str());
            break;
        }
        
        case WStype_TEXT: {
            Serial.printf("[WebSocket] 메시지 수신 [%u]: %s\n", num, payload);
            
            StaticJsonDocument<256> doc;
            deserializeJson(doc, payload);
            
            String command = doc["command"];
            if (instance->commandCallback) {
                instance->commandCallback(command, doc);
            }
            