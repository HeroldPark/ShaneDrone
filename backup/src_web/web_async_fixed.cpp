// web_async_fixed.cpp - 수정된 모바일 웹 서버
#include "../include/web.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// External symbols
extern bool systemArmed;
extern float batteryVoltage;
extern uint8_t getFlightMode();

struct SensorData {
  float roll, pitch, yaw;
  float gyroX, gyroY, gyroZ;
};
extern SensorData sensorData;

struct ControllerInput {
  float throttleNorm, rollNorm, pitchNorm, yawNorm;
};
extern ControllerInput controllerInput;

struct MotorOutputs {
  int motor_fl, motor_fr, motor_rl, motor_rr;
};
extern MotorOutputs motorOutputs;

extern void stopAllMotors();
extern void calibrateSensors();
extern void resetPIDIntegrals();

// Static objects
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static DNSServer dns;

static bool g_webRcEnabled = false;
static unsigned long g_lastRcMs = 0;
static const unsigned long RC_TIMEOUT_MS = 500;

// Simple HTML page
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shane Drone</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font:16px/1.5 -apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:linear-gradient(135deg,#1e1e2e,#2a2a3e);color:#fff;padding:20px}
.container{max-width:800px;margin:0 auto}
h1{text-align:center;margin-bottom:20px}
.card{background:rgba(255,255,255,0.1);border-radius:10px;padding:15px;margin:10px 0;backdrop-filter:blur(10px)}
.status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.status-item{padding:10px;background:rgba(0,0,0,0.2);border-radius:5px}
.label{color:rgba(255,255,255,0.7);font-size:12px}
.value{font-size:18px;font-weight:bold;color:#4fc3f7}
.controls{display:flex;flex-wrap:wrap;gap:10px;margin-top:10px}
button{padding:10px 20px;border:none;border-radius:5px;background:#4fc3f7;color:#000;font-weight:bold;cursor:pointer}
button:hover{background:#29b6f6}
.emergency{background:#f44336}
.emergency:hover{background:#e53935}
.ws-status{display:inline-block;width:10px;height:10px;border-radius:50%;background:#f44336;margin-right:10px}
.ws-status.connected{background:#4caf50}
</style>
</head>
<body>
<div class="container">
<h1><span id="ws-status" class="ws-status"></span>Shane Drone Control</h1>

<div class="card">
<h2>센서 데이터</h2>
<div class="status-grid">
<div class="status-item">
<div class="label">Roll</div>
<div class="value" id="roll">0.0°</div>
</div>
<div class="status-item">
<div class="label">Pitch</div>
<div class="value" id="pitch">0.0°</div>
</div>
<div class="status-item">
<div class="label">Yaw</div>
<div class="value" id="yaw">0.0°</div>
</div>
<div class="status-item">
<div class="label">배터리</div>
<div class="value" id="battery">0.00V</div>
</div>
</div>
</div>

<div class="card">
<h2>모터 출력</h2>
<div class="status-grid">
<div class="status-item">
<div class="label">FL</div>
<div class="value" id="motor-fl">1000</div>
</div>
<div class="status-item">
<div class="label">FR</div>
<div class="value" id="motor-fr">1000</div>
</div>
<div class="status-item">
<div class="label">RL</div>
<div class="value" id="motor-rl">1000</div>
</div>
<div class="status-item">
<div class="label">RR</div>
<div class="value" id="motor-rr">1000</div>
</div>
</div>
</div>

<div class="card">
<h2>제어</h2>
<div class="controls">
<button onclick="sendCmd('CALIBRATE')">캘리브레이션</button>
<button onclick="sendCmd('RESET_PID')">PID 리셋</button>
<button class="emergency" onclick="sendCmd('EMERGENCY_STOP')">비상정지</button>
</div>
</div>
</div>

<script>
let ws;

function connectWS() {
  ws = new WebSocket('ws://' + window.location.host + '/ws');
  
  ws.onopen = function() {
    console.log('WebSocket connected');
    document.getElementById('ws-status').classList.add('connected');
  };
  
  ws.onclose = function() {
    console.log('WebSocket disconnected');
    document.getElementById('ws-status').classList.remove('connected');
    setTimeout(connectWS, 2000);
  };
  
  ws.onmessage = function(event) {
    try {
      const data = JSON.parse(event.data);
      updateDisplay(data);
    } catch(e) {
      console.error('Parse error:', e);
    }
  };
}

function updateDisplay(data) {
  if(data.attitude) {
    document.getElementById('roll').textContent = data.attitude.roll.toFixed(1) + '°';
    document.getElementById('pitch').textContent = data.attitude.pitch.toFixed(1) + '°';
    document.getElementById('yaw').textContent = data.attitude.yaw.toFixed(1) + '°';
  }
  if(data.battery !== undefined) {
    document.getElementById('battery').textContent = data.battery.toFixed(2) + 'V';
  }
  if(data.motors) {
    document.getElementById('motor-fl').textContent = data.motors.fl;
    document.getElementById('motor-fr').textContent = data.motors.fr;
    document.getElementById('motor-rl').textContent = data.motors.rl;
    document.getElementById('motor-rr').textContent = data.motors.rr;
  }
}

function sendCmd(cmd) {
  if(ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({command: cmd}));
  }
}

// Start connection
connectWS();

// Fallback polling
setInterval(function() {
  if(!ws || ws.readyState !== WebSocket.OPEN) {
    fetch('/telemetry')
      .then(r => r.json())
      .then(updateDisplay)
      .catch(e => console.error('Fetch error:', e));
  }
}, 1000);
</script>
</body>
</html>
)HTML";

// Handler functions
static void handleTelemetry(AsyncWebServerRequest* request) {
  JsonDocument doc;
  
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
  
  String output;
  serializeJson(doc, output);
  request->send(200, "application/json", output);
}

static void handleCommand(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
  if(index == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if(error) {
      request->send(400, "text/plain", "Invalid JSON");
      return;
    }
    
    const char* cmd = doc["command"] | "";
    
    if(strcmp(cmd, "CALIBRATE") == 0) {
      calibrateSensors();
      request->send(200, "text/plain", "OK");
    }
    else if(strcmp(cmd, "RESET_PID") == 0) {
      resetPIDIntegrals();
      request->send(200, "text/plain", "OK");
    }
    else if(strcmp(cmd, "EMERGENCY_STOP") == 0) {
      systemArmed = false;
      stopAllMotors();
      request->send(200, "text/plain", "OK");
    }
    else {
      request->send(400, "text/plain", "Unknown command");
    }
  }
}

static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, 
                     AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch(type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected\n", client->id());
      break;
      
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      break;
      
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        JsonDocument doc;
        if(deserializeJson(doc, data, len) == DeserializationError::Ok) {
          const char* cmd = doc["command"] | "";
          
          if(strcmp(cmd, "CALIBRATE") == 0) {
            calibrateSensors();
          }
          else if(strcmp(cmd, "RESET_PID") == 0) {
            resetPIDIntegrals();
          }
          else if(strcmp(cmd, "EMERGENCY_STOP") == 0) {
            systemArmed = false;
            stopAllMotors();
          }
        }
      }
      break;
    }
    
    default:
      break;
  }
}

// Main namespace implementation
namespace web {
  
  void begin() {
    // Remove strict CORS headers that might cause issues
    // DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    // Main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send_P(200, "text/html", INDEX_HTML);
    });
    
    // Telemetry endpoint
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    
    // Command endpoint
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, handleCommand);
    
    // WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    // Captive portal endpoints
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r) { r->send(204); });
    server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest* r) { r->send(204); });
    
    // 404 handler
    server.onNotFound([](AsyncWebServerRequest* request) {
      request->send(404, "text/plain", "Not Found");
    });
    
    // Start DNS
    dns.start(53, "*", WiFi.softAPIP());
    
    // Start server
    server.begin();
    
    Serial.println("Web server started on port 80");
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  }
  
  void loop() {
    dns.processNextRequest();
    ws.cleanupClients();
    
    // Broadcast telemetry to WebSocket clients
    static unsigned long lastBroadcast = 0;
    unsigned long now = millis();
    
    if(now - lastBroadcast >= 100) { // 10Hz
      lastBroadcast = now;
      
      if(ws.count() > 0) {
        JsonDocument doc;
        
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
        
        String output;
        serializeJson(doc, output);
        ws.textAll(output);
      }
    }
  }
  
  void setRcEnabled(bool enabled) {
    g_webRcEnabled = enabled;
  }
  
} // namespace web