// web_async.cpp — LittleFS 정적 서빙 + WebSocket/REST (cleaned)

#include "../include/web.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

#include <LittleFS.h>
#include <FS.h>

// ===== External symbols (다른 모듈에서 제공) =====
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

// ===== Module internals =====
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static DNSServer dns;

static bool g_webRcEnabled = false;
static unsigned long g_lastRcMs = 0;
static const unsigned long RC_TIMEOUT_MS = 500;

// ===== Helpers =====
static inline String flightModeName(uint8_t m) {
  switch (m) {
    case 0: return "STABILIZE";
    case 1: return "ANGLE";
    case 2: return "ACRO";
    default: return "UNKNOWN";
  }
}

// LittleFS 마운트: CSV의 파티션 name(라벨)과 맞춰주세요(예: "storage")
#ifndef FS_LABEL
#define FS_LABEL "storage"
#endif

#ifndef WEB_LOG_FS
#define WEB_LOG_FS
#endif

static bool mountFS() {
  // Arduino-ESP32 v3.x는 기본 basePath가 "/littlefs"일 수 있음 → 둘 다 시도
  const char* basePaths[] = { "/littlefs", "/" };
  const char* labels[]    = { FS_LABEL, nullptr, "littlefs", "spiffs" };

  for (auto base : basePaths) {
    for (auto label : labels) {
      if (LittleFS.begin(false, base, 10, label)) {
        Serial.printf("[FS] Mounted: base=%s, label=%s\n", base, label ? label : "(default)");
        return true;
      }
    }
  }
  Serial.println("[FS] mount failed");
  return false;
}

static void listFS() {
#if defined(WEB_LOG_FS)
  File root = LittleFS.open("/");
  if (!root) { Serial.println("[FS] open('/') failed"); return; }
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    Serial.printf("  %s (%u bytes)\n", f.name(), (unsigned)f.size());
  }
#endif
}

// ===== HTTP handlers =====
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
  doc["input"]["roll"]     = controllerInput.rollNorm;
  doc["input"]["pitch"]    = controllerInput.pitchNorm;
  doc["input"]["yaw"]      = controllerInput.yawNorm;
  doc["rcEnabled"] = g_webRcEnabled;

  String out; serializeJson(doc, out);
  req->send(200, "application/json", out);
}

static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  if (index != 0) return; // 첫 청크만 처리

  JsonDocument doc;
  if (deserializeJson(doc, data, len)) { req->send(400, "text/plain", "JSON parse error"); return; }

  // RC 입력
  if (doc["rc"].is<JsonObject>()) {
    JsonObject r = doc["rc"].as<JsonObject>();
    g_lastRcMs = millis();
    if (g_webRcEnabled) {
        controllerInput.rollNorm     = constrain(r["roll"].as<float>(), -1.f,  1.f);
        controllerInput.pitchNorm    = constrain(r["pitch"].as<float>(), -1.f,  1.f);
        controllerInput.yawNorm      = constrain(r["yaw"].as<float>(), -1.f,  1.f);
        controllerInput.throttleNorm = constrain(r["throttle"].as<float>(),  0.f,  1.f);
    }
    req->send(200, "text/plain", "rc ok");
    return;
  }

  // 명령
  const char* cmd = doc["command"] | "";
  if      (!strcmp(cmd, "CALIBRATE"))     { calibrateSensors();       req->send(200,"text/plain","CALIBRATE ok"); }
  else if (!strcmp(cmd, "RESET_PID"))     { resetPIDIntegrals();      req->send(200,"text/plain","RESET_PID ok"); }
  else if (!strcmp(cmd, "EMERGENCY_STOP")){ systemArmed=false; stopAllMotors(); req->send(200,"text/plain","EMERGENCY_STOP ok"); }
  else if (!strcmp(cmd, "ENABLE_WEB_RC")) { g_webRcEnabled=true;      req->send(200,"text/plain","WEB_RC on"); }
  else if (!strcmp(cmd, "DISABLE_WEB_RC")){ g_webRcEnabled=false; controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f; controllerInput.throttleNorm=0.f; req->send(200,"text/plain","WEB_RC off"); }
  else                                    { req->send(400,"text/plain","unknown command"); }
}

static void wsEvent(AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:    Serial.printf("[WS] Client #%u connected\n",   client->id()); break;
    case WS_EVT_DISCONNECT: Serial.printf("[WS] Client #%u disconnected\n", client->id()); break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT) {
        JsonDocument doc;
        if (!deserializeJson(doc, data, len)) {
          if (doc["rc"].is<JsonObject>()) {
            JsonObject r = doc["rc"].as<JsonObject>();
            g_lastRcMs = millis();
            if (g_webRcEnabled) {
              // 수정: | 0 대신 .as<float>() 사용
              controllerInput.rollNorm     = constrain(r["roll"].as<float>(), -1.f,  1.f);
              controllerInput.pitchNorm    = constrain(r["pitch"].as<float>(), -1.f,  1.f);
              controllerInput.yawNorm      = constrain(r["yaw"].as<float>(), -1.f,  1.f);
              controllerInput.throttleNorm = constrain(r["throttle"].as<float>(),  0.f,  1.f);

              // // 디버그 출력 추가 (여기에 넣어야 함)
              // Serial.printf("[WEB] RC: T=%.2f R=%.2f P=%.2f Y=%.2f\n",
              //   controllerInput.throttleNorm, controllerInput.rollNorm,
              //   controllerInput.pitchNorm, controllerInput.yawNorm);
            }
          }
          const char* cmd = doc["command"] | "";
          if      (!strcmp(cmd,"CALIBRATE"))      calibrateSensors();
          else if (!strcmp(cmd,"RESET_PID"))      resetPIDIntegrals();
          else if (!strcmp(cmd,"EMERGENCY_STOP")) { systemArmed=false; stopAllMotors(); }
          else if (!strcmp(cmd,"ENABLE_WEB_RC"))  g_webRcEnabled=true;
          else if (!strcmp(cmd,"DISABLE_WEB_RC")) { g_webRcEnabled=false; controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f; controllerInput.throttleNorm=0.f; }
        }
      }
      break;
    }
    default: break;
  }
}

// ===== Public API =====
namespace web {

    void begin() {
        Serial.println("Initializing web server...");
        DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache");

        const bool fsOK = mountFS();
        if (fsOK) {
            // 정적 파일: data/ 루트 → "/"
            server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
            listFS(); // WEB_LOG_FS 정의 시 파일 목록 출력
        } else {
            // 폴백(200 OK): 최소 페이지 제공하여 500 회피
            server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
            r->send(200, "text/html",
                "<!doctype html><meta charset=utf-8>"
                "<h3>Embedded UI</h3><p>LittleFS mount failed; serving fallback UI.</p>");
            });
        }

        // Captive portal 호환
        server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
        server.on("/gen_204",      HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });

        // REST
        server.on("/telemetry", HTTP_GET, handleTelemetry);
        server.on("/command",   HTTP_POST, [](AsyncWebServerRequest*){}, nullptr, handleCommand);

        // WebSocket
        ws.onEvent(wsEvent);
        server.addHandler(&ws);

        // DNS (AP 모드에서 캡티브 포털 유도)
        dns.start(53, "*", WiFi.softAPIP());

        server.begin();
        Serial.println("Web server started on port 80");
        Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
    }

    void loop() {
        dns.processNextRequest();

        // RC timeout
        if (g_webRcEnabled && (millis() - g_lastRcMs > RC_TIMEOUT_MS)) {
            controllerInput.rollNorm = controllerInput.pitchNorm = controllerInput.yawNorm = 0.f;
            controllerInput.throttleNorm = 0.f;
        }

        // 10Hz 텔레메트리 브로드캐스트
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
                doc["input"]["roll"]     = controllerInput.rollNorm;
                doc["input"]["pitch"]    = controllerInput.pitchNorm;
                doc["input"]["yaw"]      = controllerInput.yawNorm;
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

    bool isRcEnabled() {
        return g_webRcEnabled;
    }

} // namespace web
