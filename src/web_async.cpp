// src/web_async.cpp - LittleFS로 분리된 정적 리소스를 서빙
// (LittleFS + 정적 파일 서빙)
#include "../include/web.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <LittleFS.h>     // ← 코어 기본 LittleFS
#include <ESPAsyncWebServer.h>

// ===== 외부 심볼 (기존 코드와 동일) =====
extern bool systemArmed;
extern float batteryVoltage;
extern uint8_t getFlightMode();

struct SensorData { float roll, pitch, yaw; float gyroX, gyroY, gyroZ; };
extern SensorData sensorData;

struct ControllerInput { float throttleNorm, rollNorm, pitchNorm, yawNorm; };
extern ControllerInput controllerInput;

struct MotorOutputs { int motor_fl, motor_fr, motor_rl, motor_rr; };
extern MotorOutputs motorOutputs;

extern void stopAllMotors();
extern void calibrateSensors();
extern void resetPIDIntegrals();
// =====================================

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static DNSServer dns;

static bool g_webRcEnabled = false;
static unsigned long g_lastRcMs = 0;
static const unsigned long RC_TIMEOUT_MS = 500;

static inline String flightModeName(uint8_t m) {
  switch (m) { case 0: return "STABILIZE"; case 1: return "ANGLE"; case 2: return "ACRO"; default: return "UNKNOWN"; }
}

static void handleTelemetry(AsyncWebServerRequest* req);
static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

namespace web {

  void begin() {
    Serial.println("[WEB] Initializing server...");

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/style.css", "text/css");
    });
    server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/app.js", "application/javascript");
    });

    // 또는 전체 static 매핑
    server.serveStatic("/", LittleFS, "/");

    // 캡티브 포털 호환(안드로이드 등)
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });

    // API
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr, handleCommand);

    // WebSocket
    ws.onEvent(wsEvent);
    server.addHandler(&ws);

    // DNS 시작(캡티브 포털)
    dns.start(53, "*", WiFi.softAPIP());

    // 서버 기동
    server.begin();
    Serial.println("[WEB] Server started on :80");
    Serial.print("[WEB] AP IP: "); Serial.println(WiFi.softAPIP());
  }

  void loop() {
    dns.processNextRequest();

    // RC 입력 타임아웃
    if (g_webRcEnabled && (millis() - g_lastRcMs > RC_TIMEOUT_MS)) {
      controllerInput.rollNorm = 0.f;
      controllerInput.pitchNorm = 0.f;
      controllerInput.yawNorm = 0.f;
      controllerInput.throttleNorm = 0.f;
    }

    // 10Hz 브로드캐스트
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last >= 100) {
      last = now;
      if (ws.count() > 0) {
        JsonDocument doc;
        doc["attitude"]["roll"]  = sensorData.roll;
        doc["attitude"]["pitch"] = sensorData.pitch;
        doc["attitude"]["yaw"]   = sensorData.yaw;
        doc["gyro"]["x"] = sensorData.gyroX; doc["gyro"]["y"] = sensorData.gyroY; doc["gyro"]["z"] = sensorData.gyroZ;
        doc["motors"]["fl"] = motorOutputs.motor_fl; doc["motors"]["fr"] = motorOutputs.motor_fr;
        doc["motors"]["rl"] = motorOutputs.motor_rl; doc["motors"]["rr"] = motorOutputs.motor_rr;
        doc["armed"] = systemArmed;
        doc["battery"] = batteryVoltage;
        doc["mode"] = flightModeName(getFlightMode());
        doc["input"]["throttle"] = controllerInput.throttleNorm;
        doc["input"]["roll"] = controllerInput.rollNorm;
        doc["input"]["pitch"] = controllerInput.pitchNorm;
        doc["input"]["yaw"] = controllerInput.yawNorm;
        doc["rcEnabled"] = g_webRcEnabled;

        String out; serializeJson(doc, out);
        ws.textAll(out);
      }
    }
  }

  void setRcEnabled(bool on) {
    g_webRcEnabled = on;
    if (!on) {
      controllerInput.rollNorm = controllerInput.pitchNorm = controllerInput.yawNorm = 0.f;
      controllerInput.throttleNorm = 0.f;
    }
  }

} // namespace web

// ===== HTTP Handlers =====
static void handleTelemetry(AsyncWebServerRequest* req) {
  JsonDocument doc;
  doc["attitude"]["roll"]  = sensorData.roll;
  doc["attitude"]["pitch"] = sensorData.pitch;
  doc["attitude"]["yaw"]   = sensorData.yaw;
  doc["gyro"]["x"] = sensorData.gyroX; doc["gyro"]["y"] = sensorData.gyroY; doc["gyro"]["z"] = sensorData.gyroZ;
  doc["motors"]["fl"] = motorOutputs.motor_fl; doc["motors"]["fr"] = motorOutputs.motor_fr;
  doc["motors"]["rl"] = motorOutputs.motor_rl; doc["motors"]["rr"] = motorOutputs.motor_rr;
  doc["armed"] = systemArmed;
  doc["battery"] = batteryVoltage;
  doc["mode"] = flightModeName(getFlightMode());
  doc["input"]["throttle"] = controllerInput.throttleNorm;
  doc["input"]["roll"] = controllerInput.rollNorm;
  doc["input"]["pitch"] = controllerInput.pitchNorm;
  doc["input"]["yaw"] = controllerInput.yawNorm;
  doc["rcEnabled"] = g_webRcEnabled;
  String out; serializeJson(doc, out);
  req->send(200, "application/json", out);
}

static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  JsonDocument doc;
  auto err = deserializeJson(doc, data, len);
  if (err) { req->send(400, "text/plain", "JSON parse error"); return; }

  // RC
  if (doc["rc"].is<JsonObject>()) {
    JsonObject r = doc["rc"].as<JsonObject>();
    g_lastRcMs = millis();
    if (g_webRcEnabled) {
      controllerInput.rollNorm     = constrain((float)(r["roll"]     | 0), -1.f, 1.f);
      controllerInput.pitchNorm    = constrain((float)(r["pitch"]    | 0), -1.f, 1.f);
      controllerInput.yawNorm      = constrain((float)(r["yaw"]      | 0), -1.f, 1.f);
      controllerInput.throttleNorm = constrain((float)(r["throttle"] | 0),  0.f, 1.f);
    }
    req->send(200, "text/plain", "rc ok");
    return;
  }

  // Commands
  const char* cmd = doc["command"] | "";
  if (!strcmp(cmd,"CALIBRATE"))        { calibrateSensors();      req->send(200,"text/plain","CALIBRATE ok"); }
  else if (!strcmp(cmd,"RESET_PID"))   { resetPIDIntegrals();     req->send(200,"text/plain","RESET_PID ok"); }
  else if (!strcmp(cmd,"EMERGENCY_STOP")) { systemArmed=false; stopAllMotors(); req->send(200,"text/plain","EMERGENCY_STOP ok"); }
  else if (!strcmp(cmd,"ENABLE_WEB_RC"))  { g_webRcEnabled=true;  req->send(200,"text/plain","WEB_RC on"); }
  else if (!strcmp(cmd,"DISABLE_WEB_RC")) {
    g_webRcEnabled=false;
    controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f;
    controllerInput.throttleNorm=0.f;
    req->send(200,"text/plain","WEB_RC off");
  } else {
    req->send(400,"text/plain","unknown command");
  }
}

// ===== WebSocket =====
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:    Serial.printf("[WS] Client #%u connected\n", client->id()); break;
    case WS_EVT_DISCONNECT: Serial.printf("[WS] Client #%u disconnected\n", client->id()); break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT) {
        JsonDocument doc;
        if (deserializeJson(doc, data, len) == DeserializationError::Ok) {
          if (doc["rc"].is<JsonObject>()) {
            JsonObject r = doc["rc"].as<JsonObject>();
            g_lastRcMs = millis();
            if (g_webRcEnabled) {
              controllerInput.rollNorm     = constrain((float)(r["roll"]     | 0), -1.f, 1.f);
              controllerInput.pitchNorm    = constrain((float)(r["pitch"]    | 0), -1.f, 1.f);
              controllerInput.yawNorm      = constrain((float)(r["yaw"]      | 0), -1.f, 1.f);
              controllerInput.throttleNorm = constrain((float)(r["throttle"] | 0),  0.f, 1.f);
            }
          }
          const char* cmd = doc["command"] | "";
          if      (!strcmp(cmd,"CALIBRATE"))      calibrateSensors();
          else if (!strcmp(cmd,"RESET_PID"))      resetPIDIntegrals();
          else if (!strcmp(cmd,"EMERGENCY_STOP")) { systemArmed=false; stopAllMotors(); }
          else if (!strcmp(cmd,"ENABLE_WEB_RC"))  g_webRcEnabled=true;
          else if (!strcmp(cmd,"DISABLE_WEB_RC")) {
            g_webRcEnabled=false;
            controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f;
            controllerInput.throttleNorm=0.f;
          }
        }
      }
      break;
    }
    default: break;
  }
}
