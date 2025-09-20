// web_legacy.cpp - HTTP 서버 & WebSocket 전담
// PC 버전
#include "../include/web.h"
#include "../include/config.h"
#include "../include/control.h"
#include "../include/sensors.h"

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>


// ===== 내부 상태/참조 =====

// 원본 전역/외부 심볼 사용
extern bool systemArmed;
extern SensorData sensorData;
extern ControllerInput controllerInput;
extern MotorOutputs motorOutputs;
extern float batteryVoltage;
extern void stopAllMotors();
extern void calibrateSensors();
extern void resetPIDIntegrals();
extern uint8_t getFlightMode();

// 로컬 서버 인스턴스 (외부로 노출하지 않음)
static WebServer webServer(80);
static WebSocketsServer webSocket(81);

// ===== 내부 선언 =====
static void handleRoot();
static void handleTelemetryData();
static void handleCommand();
static void handleNotFound();
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
static void broadcastTelemetry();

// ===== 구현 =====

namespace web {

void begin() {
  Serial.println("웹 서버 설정 중...");

  // 라우트
  webServer.on("/", handleRoot);
  webServer.on("/telemetry", handleTelemetryData);
  webServer.on("/command", HTTP_POST, handleCommand);
  webServer.onNotFound(handleNotFound);

  // CORS
  webServer.enableCORS(true);

  // 시작
  webServer.begin();
  Serial.println("HTTP 서버 시작됨 (포트 80)");

  // WebSocket 시작
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket 서버 시작됨 (포트 81)");
}

void loop() {
  // HTTP 요청 처리
  webServer.handleClient();

  // WebSocket 이벤트 루프
  webSocket.loop();

  // 10Hz 브로드캐스트
  broadcastTelemetry();
}

} // namespace web

// ====== WebSocket 이벤트 ======
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] WebSocket 연결 끊김\n", num);
      break;

    case WStype_CONNECTED:
      // 클라이언트 IP는 핸드셰이크 시점에 접근 가능
      Serial.printf("[%u] WebSocket 연결됨\n", num);
      break;

    case WStype_TEXT: {
      Serial.printf("[%u] 명령 수신: %s\n", num, payload);
      DynamicJsonDocument doc(256);
      DeserializationError err = deserializeJson(doc, payload, length);
      if (err) return;

      String command = doc["command"];
      if (command == "CALIBRATE") {
        calibrateSensors();
      } else if (command == "RESET_PID") {
        resetPIDIntegrals();
      } else if (command == "EMERGENCY_STOP") {
        systemArmed = false;
        stopAllMotors();
      }
      break;
    }

    default:
      break;
  }
}

// ====== HTTP 핸들러 ======
static void handleRoot() {
  // 원본의 R"rawliteral(...)" HTML 그대로 이동
  String html = R"rawliteral(
  <!-- [원본과 동일, 생략 없이 그대로 복사: 대시보드 HTML/CSS/JS] -->
  <!DOCTYPE html>
  <html lang="ko">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Shane 드론 모니터</title>
    <style>
      * { margin: 0; padding: 0; box-sizing: border-box; }
      body {
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        background: linear-gradient(135deg, #1e1e2e 0%, #2a2a3e 100%);
        color: white; padding: 20px; min-height: 100vh;
      }
      .container { max-width: 1200px; margin: 0 auto; }
      .header { text-align: center; margin-bottom: 30px; }
      .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
      .card {
        background: rgba(255,255,255,0.1); border-radius: 15px; padding: 20px;
        backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2);
      }
      .card h3 { color: #4fc3f7; margin-bottom: 15px; }
      .data-row { display: flex; justify-content: space-between; margin: 8px 0; }
      .status-indicator {
        display: inline-block; width: 12px; height: 12px;
        border-radius: 50%; margin-right: 8px;
      }
      .status-connected { background: #44ff44; }
      .status-disconnected { background: #ff4444; }
      .motor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
      .motor { background: rgba(0,0,0,0.3); padding: 10px; border-radius: 8px; text-align: center; }
      .motor-bar {
        width: 100%; height: 8px; background: #333; border-radius: 4px;
        overflow: hidden; margin-top: 5px;
      }
      .motor-fill {
        height: 100%; background: linear-gradient(to right, #44ff44, #ffaa44, #ff4444);
        transition: width 0.3s ease;
      }
      .attitude-display {
        width: 150px; height: 150px; margin: 0 auto;
        background: #000; border-radius: 50%; position: relative;
        border: 2px solid #4fc3f7;
      }
      .attitude-line {
        position: absolute; top: 50%; left: 10%; right: 10%; height: 2px;
        background: #4fc3f7; transform-origin: center;
      }
      .controls { margin-top: 20px; }
      .button {
        background: #4fc3f7; border: none; padding: 10px 20px;
        border-radius: 8px; color: white; cursor: pointer; margin: 5px;
      }
      .button:hover { background: #29b6f6; }
      .button.danger { background: #ff4444; }
    </style>
  </head>
  <body>
    <div class="container">
      <div class="header">
        <h1>Shane 드론 모니터</h1>
        <p>실시간 상태 모니터링</p>
        <div>
          <span class="status-indicator" id="wsStatus"></span>
          <span id="wsStatusText">연결 중...</span>
        </div>
      </div>

      <div class="status-grid">
        <div class="card">
          <h3>자세 정보</h3>
          <div class="attitude-display">
            <div class="attitude-line" id="attitudeLine"></div>
          </div>
          <div class="data-row"><span>Roll:</span><span id="roll">0.0°</span></div>
          <div class="data-row"><span>Pitch:</span><span id="pitch">0.0°</span></div>
          <div class="data-row"><span>Yaw:</span><span id="yaw">0.0°</span></div>
        </div>

        <div class="card">
          <h3>모터 출력</h3>
          <div class="motor-grid">
            <div class="motor">
              FL <div class="motor-bar"><div class="motor-fill" id="motorFL"></div></div>
              <span id="motorFLVal">1000</span>
            </div>
            <div class="motor">
              FR <div class="motor-bar"><div class="motor-fill" id="motorFR"></div></div>
              <span id="motorFRVal">1000</span>
            </div>
            <div class="motor">
              RL <div class="motor-bar"><div class="motor-fill" id="motorRL"></div></div>
              <span id="motorRLVal">1000</span>
            </div>
            <div class="motor">
              RR <div class="motor-bar"><div class="motor-fill" id="motorRR"></div></div>
              <span id="motorRRVal">1000</span>
            </div>
          </div>
        </div>

        <div class="card">
          <h3>시스템 상태</h3>
          <div class="data-row">
            <span>ARM:</span>
            <span><span class="status-indicator" id="armStatus"></span><span id="armText">DISARMED</span></span>
          </div>
          <div class="data-row"><span>비행모드:</span><span id="flightMode">STABILIZE</span></div>
          <div class="data-row"><span>배터리:</span><span id="battery">0.0V</span></div>
          <div class="data-row"><span>수신기:</span><span id="receiver">연결됨</span></div>
        </div>

        <div class="card">
          <h3>센서 데이터</h3>
          <div class="data-row"><span>Gyro X:</span><span id="gyroX">0.0</span></div>
          <div class="data-row"><span>Gyro Y:</span><span id="gyroY">0.0</span></div>
          <div class="data-row"><span>Gyro Z:</span><span id="gyroZ">0.0</span></div>
          <div class="data-row"><span>스로틀:</span><span id="throttle">0%</span></div>
        </div>
      </div>

      <div class="controls">
        <button class="button" onclick="sendCommand('CALIBRATE')">센서 캘리브레이션</button>
        <button class="button" onclick="sendCommand('RESET_PID')">PID 리셋</button>
        <button class="button danger" onclick="sendCommand('EMERGENCY_STOP')">비상 정지</button>
      </div>
    </div>

    <script>
      let ws;
      let reconnectInterval;

      function connectWebSocket() {
        ws = new WebSocket('ws://' + window.location.hostname + ':81/');
        ws.onopen = function() {
          document.getElementById('wsStatus').className = 'status-indicator status-connected';
          document.getElementById('wsStatusText').textContent = '연결됨';
          if (reconnectInterval) { clearInterval(reconnectInterval); reconnectInterval = null; }
        };
        ws.onmessage = function(event) {
          const data = JSON.parse(event.data);
          updateDisplay(data);
        };
        ws.onclose = function() {
          document.getElementById('wsStatus').className = 'status-indicator status-disconnected';
          document.getElementById('wsStatusText').textContent = '연결 끊김';
          if (!reconnectInterval) { reconnectInterval = setInterval(connectWebSocket, 3000); }
        };
      }

      function updateDisplay(data) {
        if (data.attitude) {
          document.getElementById('roll').textContent = data.attitude.roll.toFixed(1) + '°';
          document.getElementById('pitch').textContent = data.attitude.pitch.toFixed(1) + '°';
          document.getElementById('yaw').textContent = data.attitude.yaw.toFixed(1) + '°';
          document.getElementById('attitudeLine').style.transform = `rotate(${data.attitude.roll}deg)`;
        }
        if (data.motors) {
          updateMotor('FL', data.motors.fl);
          updateMotor('FR', data.motors.fr);
          updateMotor('RL', data.motors.rl);
          updateMotor('RR', data.motors.rr);
        }
        if (data.gyro) {
          document.getElementById('gyroX').textContent = data.gyro.x.toFixed(1);
          document.getElementById('gyroY').textContent = data.gyro.y.toFixed(1);
          document.getElementById('gyroZ').textContent = data.gyro.z.toFixed(1);
        }
        if (typeof data.battery !== 'undefined') {
          document.getElementById('battery').textContent = data.battery.toFixed(2) + 'V';
        }
        if (typeof data.armed !== 'undefined') {
          const armStatus = document.getElementById('armStatus');
          const armText = document.getElementById('armText');
          if (data.armed) {
            armStatus.className = 'status-indicator status-disconnected';
            armText.textContent = 'ARMED';
          } else {
            armStatus.className = 'status-indicator status-connected';
            armText.textContent = 'DISARMED';
          }
        }
        if (data.input && data.input.throttle) {
          document.getElementById('throttle').textContent = (data.input.throttle * 100).toFixed(0) + '%';
        }
      }

      function updateMotor(motor, value) {
        const percentage = ((value - 1000) / 1000) * 100;
        document.getElementById('motor' + motor).style.width = Math.max(0, percentage) + '%';
        document.getElementById('motor' + motor + 'Val').textContent = value;
      }

      function sendCommand(cmd) {
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({command: cmd}));
        } else {
          alert('WebSocket이 연결되지 않았습니다');
        }
      }

      connectWebSocket();

      // 백업 폴링
      setInterval(function() {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
          fetch('/telemetry')
            .then(r => r.json())
            .then(updateDisplay)
            .catch(console.error);
        }
      }, 500);
    </script>
  </body>
  </html>
  )rawliteral";

  webServer.send(200, "text/html", html);
}

static void handleTelemetryData() {
  DynamicJsonDocument doc(1024);

  doc["attitude"]["roll"] = sensorData.roll;
  doc["attitude"]["pitch"] = sensorData.pitch;
  doc["attitude"]["yaw"] = sensorData.yaw;

  doc["gyro"]["x"] = sensorData.gyroX;
  doc["gyro"]["y"] = sensorData.gyroY;
  doc["gyro"]["z"] = sensorData.gyroZ;

  doc["motors"]["fl"] = motorOutputs.motor_fl;
  doc["motors"]["fr"] = motorOutputs.motor_fr;
  doc["motors"]["rl"] = motorOutputs.motor_rl;
  doc["motors"]["rr"] = motorOutputs.motor_rr;

  doc["armed"] = systemArmed;
  doc["battery"] = batteryVoltage;
  doc["mode"] = getFlightMode();

  doc["input"]["throttle"] = controllerInput.throttleNorm;
  doc["input"]["roll"] = controllerInput.rollNorm;
  doc["input"]["pitch"] = controllerInput.pitchNorm;
  doc["input"]["yaw"] = controllerInput.yawNorm;

  doc["timestamp"] = millis();

  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

static void handleCommand() {
  if (!webServer.hasArg("plain")) {
    webServer.send(400, "text/plain", "잘못된 요청");
    return;
  }

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, webServer.arg("plain"));
  if (err) {
    webServer.send(400, "text/plain", "JSON 파싱 오류");
    return;
  }

  String command = doc["command"];

  if (command == "CALIBRATE") {
    calibrateSensors();
    webServer.send(200, "text/plain", "센서 캘리브레이션 시작");
  }
  else if (command == "RESET_PID") {
    resetPIDIntegrals();
    webServer.send(200, "text/plain", "PID 적분항 리셋");
  }
  else if (command == "EMERGENCY_STOP") {
    systemArmed = false;
    stopAllMotors();
    webServer.send(200, "text/plain", "비상 정지 실행");
  }
  else {
    webServer.send(400, "text/plain", "알 수 없는 명령");
  }
}

static void handleNotFound() {
  webServer.send(404, "text/plain", "페이지를 찾을 수 없습니다");
}

// 10Hz 실시간 브로드캐스트
static void broadcastTelemetry() {
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast <= 100) return;

  DynamicJsonDocument doc(1024);

  doc["attitude"]["roll"] = sensorData.roll;
  doc["attitude"]["pitch"] = sensorData.pitch;
  doc["attitude"]["yaw"] = sensorData.yaw;

  doc["gyro"]["x"] = sensorData.gyroX;
  doc["gyro"]["y"] = sensorData.gyroY;
  doc["gyro"]["z"] = sensorData.gyroZ;

  doc["motors"]["fl"] = motorOutputs.motor_fl;
  doc["motors"]["fr"] = motorOutputs.motor_fr;
  doc["motors"]["rl"] = motorOutputs.motor_rl;
  doc["motors"]["rr"] = motorOutputs.motor_rr;

  doc["armed"] = systemArmed;
  doc["battery"] = batteryVoltage;
  doc["mode"] = getFlightMode();

  doc["input"]["throttle"] = controllerInput.throttleNorm;

  String message;
  serializeJson(doc, message);

  webSocket.broadcastTXT(message);
  lastBroadcast = millis();
}
