// src/web_async.cpp - LittleFS로 분리된 정적 리소스를 서빙
// (LittleFS + 정적 파일 서빙)
#include "../include/web.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <LittleFS.h>     // ← 코어 기본 LittleFS
#include <ESPAsyncWebServer.h>
// #include <uri/UriRegex.h>  // 상단 include 추가

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

// If you want /fs directory listing for debug, define this
#define WEB_DEBUG_FS

namespace web {

  static AsyncWebServer server(80);

  // --- MIME helper (without caring about .gz; gzip handled in sendMaybeGz) ---
  static String mimeTypeFor(const String &path) {
    if (path.endsWith(".html")) return F("text/html");
    if (path.endsWith(".css"))  return F("text/css");
    if (path.endsWith(".js"))   return F("application/javascript");
    if (path.endsWith(".mjs"))  return F("text/javascript");
    if (path.endsWith(".ico"))  return F("image/x-icon");
    if (path.endsWith(".png"))  return F("image/png");
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return F("image/jpeg");
    if (path.endsWith(".svg"))  return F("image/svg+xml");
    if (path.endsWith(".gif"))  return F("image/gif");
    if (path.endsWith(".woff2"))return F("font/woff2");
    if (path.endsWith(".woff")) return F("font/woff");
    if (path.endsWith(".ttf"))  return F("font/ttf");
    if (path.endsWith(".json")) return F("application/json");
    if (path.endsWith(".wasm")) return F("application/wasm");
    if (path.endsWith(".txt"))  return F("text/plain");
    return F("application/octet-stream");
  }

  // --- gzip-aware sender ---
  static void sendMaybeGz(AsyncWebServerRequest *req, const char *path, const char *mime) {
    String p = path;
    String gz = p + F(".gz");
    if (LittleFS.exists(gz)) {
      // Serve pre-compressed file with correct headers
      AsyncWebServerResponse *res = req->beginResponse(LittleFS, gz, mime);
      res->addHeader(F("Content-Encoding"), F("gzip"));
      res->addHeader(F("Cache-Control"),    F("public, max-age=86400"));
      req->send(res);
    } else if (LittleFS.exists(p)) {
      AsyncWebServerResponse *res = req->beginResponse(LittleFS, p, mime);
      res->addHeader(F("Cache-Control"), F("public, max-age=86400"));
      req->send(res);
    } else {
      req->send(404, F("text/plain"), F("Not found"));
    }
  }

  #ifdef WEB_DEBUG_FS
  static void listLittleFsRoot(Print &out) {
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      out.printf("%s\t%u bytes\n", file.name(), (unsigned)file.size());
      file = root.openNextFile();
    }
  }
  #endif

  void begin() {
    // 1) Mount FS
    if (!LittleFS.begin()) {
      Serial.println(F("[web] LittleFS mount failed"));
      // LittleFS.format(); // Use only when you know what you're doing
    }

    // 2) Serve all static files. Do not prefix with "/littlefs".
    server.serveStatic("/littlefs", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("public, max-age=86400");

    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("public, max-age=86400");

    // 3) favicon (optional, silence logs if missing)
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *req) {
      if (LittleFS.exists("/favicon.ico") || LittleFS.exists("/favicon.ico.gz")) {
        sendMaybeGz(req, "/favicon.ico", "image/x-icon");
      } else {
        req->send(204); // No Content -> avoid noisy 404 logs
      }
    });

  #ifdef WEB_DEBUG_FS
    server.on("/fs", HTTP_GET, [](AsyncWebServerRequest *req) {
      String body;
      body.reserve(1024);
      struct StringPrinter : public Print { String &s; StringPrinter(String &ss) : s(ss) {} size_t write(uint8_t c) override { s += (char)c; return 1; } } sp(body);
      body += F("/ LittleFS listing\n-------------------\n");
      listLittleFsRoot(sp);
      req->send(200, F("text/plain"), body);
    });
  #endif

    // 4) onNotFound -> static file or SPA fallback to /index.html
    server.onNotFound([](AsyncWebServerRequest *req) {
      String path = req->url();

      // Gracefully handle accidental "/littlefs" prefix in requests
      const String prefix = F("/littlefs");
      if (path.startsWith(prefix)) {
        path = path.substring(prefix.length());
        if (path.isEmpty()) path = "/";
      }

      // Try to serve the exact file first (supports .gz)
      if (path == "/") {
        sendMaybeGz(req, "/index.html", "text/html");
        return;
      }

      if (LittleFS.exists(path) || LittleFS.exists(path + F(".gz"))) {
        String mime = mimeTypeFor(path);
        sendMaybeGz(req, path.c_str(), mime.c_str());
        return;
      }

      // SPA fallback
      sendMaybeGz(req, "/index.html", "text/html");
    });

    server.begin();
    Serial.println(F("[web] HTTP server started"));

    // 5) Optional: simple connectivity test file (non-gz to avoid confusion)
    if (!LittleFS.exists("/connecttest.txt")) {
      File w = LittleFS.open("/connecttest.txt", FILE_WRITE);
      if (w) { w.print("hello\n"); w.close(); }
    }
  }

  void loop() {
    // Currently no periodic work is required for ESPAsyncWebServer.
    // Keep this as a hook in case you want timed tasks (metrics, WiFi watchdog, etc.).
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
