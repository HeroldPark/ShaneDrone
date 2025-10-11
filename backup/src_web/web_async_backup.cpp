// web_async.cpp - 분리된 HTML/CSS/JS 헤더 파일 사용 버전
#include "../include/web.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

#include <LittleFS.h>   // 추가
#include <FS.h>

// 분리된 헤더 파일들 포함
#include "../include/drone_html.h"      // HTML 구조
#include "../include/drone_css.h"       // CSS 스타일
#include "../include/drone_js.h"        // JavaScript 코드
// #include "../include/drone_simple_html.h"  // 간단한 버전 (필요시)

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

// Module internal objects
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static DNSServer dns;

static bool g_webRcEnabled = false;
static unsigned long g_lastRcMs = 0;
static const unsigned long RC_TIMEOUT_MS = 500;

// 인터페이스 타입 설정
enum WebInterfaceType {
  WEB_INTERFACE_SIMPLE = 0,
  WEB_INTERFACE_3D = 1
};

static WebInterfaceType g_interfaceType = WEB_INTERFACE_3D; // 기본값

// Flight mode name helper
static inline String flightModeName(uint8_t m) {
  switch(m) {
    case 0: return "STABILIZE";
    case 1: return "ANGLE";
    case 2: return "ACRO";
    default: return "UNKNOWN";
  }
}

// // HTML 조합 함수
// String buildCompleteHTML() {
//   String html = String(DRONE_HTML);
//   html.replace("%CSS_CONTENT%", DRONE_CSS);
//   html.replace("%JS_CONTENT%", DRONE_JS);
//   return html;
// }

// ---- Add this in web_async.cpp ----
static String buildCompleteHTML() {
  // 헤더 내장 문자열(프로그램 메모리)에서 안전하게 읽어오기
  String tpl = String(FPSTR(DRONE_HTML));
  String css = String(FPSTR(DRONE_CSS));
  String js  = String(FPSTR(DRONE_JS));

  // 템플릿이 있다면 플레이스홀더 치환해서 사용
  if (tpl.length() >= 10) {
    tpl.replace("%CSS_CONTENT%", css);
    tpl.replace("%JS_CONTENT%",  js);
    return tpl;
  }

  // 템플릿이 비어있으면(지금처럼) 최소 폴백 페이지 생성
  if (css.length() < 10) {
    css = F("body{font:14px/1.5 system-ui,sans-serif;background:#0a0a0a;color:#fff}"
            "#canvas3d{position:fixed;inset:0} .overlay{position:fixed;top:10px;left:10px;"
            "background:rgba(0,0,0,.6);padding:8px 10px;border-radius:8px}");
  }
  if (js.length() < 10) {
    js = F("const $=s=>document.querySelector(s);"
           "let ws;window.addEventListener('DOMContentLoaded',()=>{"
           "ws=new WebSocket('ws://'+location.host+'/ws');"
           "ws.onopen=()=>console.log('WS connected');});");
  }

  String out;
  out.reserve(2048 + css.length() + js.length());
  out += F("<!doctype html><html lang='ko'><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>Shane Drone (embedded)</title><style>");
  out += css;
  out += F("</style></head><body><div id='canvas3d'></div>"
           "<div class='overlay'>LittleFS mount 실패 — 임베디드 UI 구동</div>"
           "<script src='https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js'></script>"
           "<script>");
  out += js;
  out += F("</script></body></html>");
  return out;
}

// LittleFS 마운트(라벨/베이스 경로 자동 호환)
static bool mountLittleFS() {
  // Arduino-ESP32 v3.x 기본 basePath는 "/littlefs"
  const char* basePaths[] = { "/littlefs", "/" };
  const char* labels[]    = { nullptr, "littlefs", "spiffs", "storage" };

  for (auto base : basePaths) {
    for (auto label : labels) {
      if (LittleFS.begin(false, base, 10, label)) {
        Serial.printf("[FS] Mounted: base=%s, label=%s\n", base, label ? label : "(default)");
        return true;
      }
    }
  }
  return false;
}

// 1) 마운트 유틸
static bool mountFS() {
  const char* basePaths[] = { "/littlefs", "/" };
  const char* labels[]    = { "storage", nullptr, "littlefs", "spiffs" };

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

// (선택) 파일 목록 찍기
static void listFS() {
  File root = LittleFS.open("/");
  if (!root) { Serial.println("[FS] open('/') failed"); return; }
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    Serial.printf("  %s (%u bytes)\n", f.name(), (unsigned)f.size());
  }
}

// Handler declarations
static void handleTelemetry(AsyncWebServerRequest* req);
static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

namespace web {

  void setInterfaceType(WebInterfaceType type) {
    g_interfaceType = type;
  }

  void begin() {
    Serial.println("Initializing web server...");
    
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache");

    // // 파일시스템 마운트
    // if (!LittleFS.begin(false)) {
    //   Serial.println("LittleFS mount failed");
    //   // FS 실패 → 간단한 오류 페이지 표시
    //   server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    //     r->send(500, "text/html", 
    //       "<h1>File System Error</h1><p>LittleFS mount failed. Please upload files.</p>");
    //   });
    // } else {
    //   server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    // }

    // File root = LittleFS.open("/");
    // for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    //   Serial.printf("  %s (%u bytes)\n", f.name(), (unsigned)f.size());
    // }

    // 2) 마운트 시도
    const bool fsOK = mountFS();

    if (fsOK) {
      // 3) 정적 파일 제공
      server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
      listFS(); // 확인용
    } else {
      // 4) 폴백 페이지는 200으로 내려서 500을 피합니다.
      server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
        r->send(200, "text/html",
          "<!doctype html><meta charset=utf-8>"
          "<h3>Embedded UI</h3>"
          "<p>LittleFS mount failed; serving fallback UI.</p>");
      });
    }
    
    // Captive portal routes
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    
    // // Main page - 분리된 헤더 파일들을 조합하여 전송
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ 
    //   if (g_interfaceType == WEB_INTERFACE_3D) {
    //     // 3D 인터페이스: 분리된 파일들을 조합
    //     String completeHTML = buildCompleteHTML();
    //     r->send(200, "text/html", completeHTML);
    //   } else {
    //     // Simple 인터페이스: 기존 단일 파일
    //     r->send(200, "text/html", DRONE_SIMPLE_HTML);
    //   }
    // });
    
    // // 인터페이스 전환 엔드포인트
    // server.on("/interface/3d", HTTP_GET, [](AsyncWebServerRequest* r){ 
    //   g_interfaceType = WEB_INTERFACE_3D;
    //   String completeHTML = buildCompleteHTML();
    //   r->send(200, "text/html", completeHTML);
    // });
    
    // server.on("/interface/simple", HTTP_GET, [](AsyncWebServerRequest* r){ 
    //   g_interfaceType = WEB_INTERFACE_SIMPLE;
    //   r->send(200, "text/html", DRONE_SIMPLE_HTML);
    // });
    
    // // URL 파라미터로 인터페이스 선택
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    //   if (r->hasParam("ui")) {
    //     String uiType = r->getParam("ui")->value();
    //     if (uiType == "3d") {
    //       String completeHTML = buildCompleteHTML();
    //       r->send(200, "text/html", completeHTML);
    //       return;
    //     } else if (uiType == "simple") {
    //       r->send(200, "text/html", DRONE_SIMPLE_HTML);
    //       return;
    //     }
    //   }
      
    //   // 기본값
    //   if (g_interfaceType == WEB_INTERFACE_3D) {
    //     String completeHTML = buildCompleteHTML();
    //     r->send(200, "text/html", completeHTML);
    //   } else {
    //     r->send(200, "text/html", DRONE_SIMPLE_HTML);
    //   }
    // });
    
    // 개별 파일 접근 엔드포인트 (디버깅용)
    #ifdef WEB_DEBUG_FS
    server.on("/debug/css", HTTP_GET, [](AsyncWebServerRequest* r){
      r->send(200, "text/css", DRONE_CSS);
    });
    
    server.on("/debug/js", HTTP_GET, [](AsyncWebServerRequest* r){
      r->send(200, "application/javascript", DRONE_JS);
    });
    
    server.on("/debug/html", HTTP_GET, [](AsyncWebServerRequest* r){
      r->send(200, "text/html", DRONE_HTML);
    });
    #endif
    
    // API endpoints
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, handleCommand);
    
    // WebSocket
    ws.onEvent(wsEvent);
    server.addHandler(&ws);
    
    // Start DNS
    dns.start(53, "*", WiFi.softAPIP());
    
    // Start server
    server.begin();
    Serial.println("Web server started on port 80");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("Interface type: %s\n", 
                  g_interfaceType == WEB_INTERFACE_3D ? "3D" : "Simple");
  }

  void loop() {
    dns.processNextRequest();
    
    // RC timeout check
    if (g_webRcEnabled && (millis() - g_lastRcMs > RC_TIMEOUT_MS)) {
      controllerInput.rollNorm = 0.f;
      controllerInput.pitchNorm = 0.f;
      controllerInput.yawNorm = 0.f;
      controllerInput.throttleNorm = 0.f;
    }
    
    // Broadcast telemetry at 10Hz
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last >= 100) {
      last = now;
      
      if (ws.count() > 0) {
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
        doc["mode"] = flightModeName(getFlightMode());
        doc["input"]["throttle"] = controllerInput.throttleNorm;
        doc["input"]["roll"] = controllerInput.rollNorm;
        doc["input"]["pitch"] = controllerInput.pitchNorm;
        doc["input"]["yaw"] = controllerInput.yawNorm;
        doc["rcEnabled"] = g_webRcEnabled;
        
        String out;
        serializeJson(doc, out);
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

// HTTP handlers (기존과 동일)
static void handleTelemetry(AsyncWebServerRequest* req) {
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
  doc["mode"] = flightModeName(getFlightMode());
  doc["input"]["throttle"] = controllerInput.throttleNorm;
  doc["input"]["roll"] = controllerInput.rollNorm;
  doc["input"]["pitch"] = controllerInput.pitchNorm;
  doc["input"]["yaw"] = controllerInput.yawNorm;
  doc["rcEnabled"] = g_webRcEnabled;
  
  String out;
  serializeJson(doc, out);
  req->send(200, "application/json", out);
}

static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  if (index != 0) return; // Only process first chunk
  
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    req->send(400, "text/plain", "JSON parse error");
    return;
  }
  
  // RC data
  if (doc["rc"].is<JsonObject>()) {
    JsonObject r = doc["rc"].as<JsonObject>();
    g_lastRcMs = millis();
    if (g_webRcEnabled) {
      controllerInput.rollNorm = constrain((float)(r["roll"] | 0), -1.f, 1.f);
      controllerInput.pitchNorm = constrain((float)(r["pitch"] | 0), -1.f, 1.f);
      controllerInput.yawNorm = constrain((float)(r["yaw"] | 0), -1.f, 1.f);
      controllerInput.throttleNorm = constrain((float)(r["throttle"] | 0), 0.f, 1.f);
    }
    req->send(200, "text/plain", "rc ok");
    return;
  }
  
  // Commands
  const char* cmd = doc["command"] | "";
  if (!strcmp(cmd,"CALIBRATE")) { 
    calibrateSensors(); 
    req->send(200,"text/plain","CALIBRATE ok"); 
  }
  else if (!strcmp(cmd,"RESET_PID")) { 
    resetPIDIntegrals(); 
    req->send(200,"text/plain","RESET_PID ok"); 
  }
  else if (!strcmp(cmd,"EMERGENCY_STOP")) { 
    systemArmed=false; 
    stopAllMotors(); 
    req->send(200,"text/plain","EMERGENCY_STOP ok"); 
  }
  else if (!strcmp(cmd,"ENABLE_WEB_RC")) { 
    g_webRcEnabled=true; 
    req->send(200,"text/plain","WEB_RC on"); 
  }
  else if (!strcmp(cmd,"DISABLE_WEB_RC")) { 
    g_webRcEnabled=false;
    controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f;
    controllerInput.throttleNorm=0.f;
    req->send(200,"text/plain","WEB_RC off"); 
  }
  else { 
    req->send(400,"text/plain","unknown command"); 
  }
}

static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch(type){
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT){
        JsonDocument doc;
        if (deserializeJson(doc, data, len)==DeserializationError::Ok) {
          if (doc["rc"].is<JsonObject>()) {
            JsonObject r = doc["rc"].as<JsonObject>();
            g_lastRcMs = millis();
            if (g_webRcEnabled) {
              controllerInput.rollNorm = constrain((float)(r["roll"] | 0), -1.f, 1.f);
              controllerInput.pitchNorm = constrain((float)(r["pitch"] | 0), -1.f, 1.f);
              controllerInput.yawNorm = constrain((float)(r["yaw"] | 0), -1.f, 1.f);
              controllerInput.throttleNorm = constrain((float)(r["throttle"] | 0), 0.f, 1.f);
            }
          }
          const char* cmd = doc["command"] | "";
          if (!strcmp(cmd,"CALIBRATE")) calibrateSensors();
          else if (!strcmp(cmd,"RESET_PID")) resetPIDIntegrals();
          else if (!strcmp(cmd,"EMERGENCY_STOP")) { systemArmed=false; stopAllMotors(); }
          else if (!strcmp(cmd,"ENABLE_WEB_RC")) g_webRcEnabled=true;
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