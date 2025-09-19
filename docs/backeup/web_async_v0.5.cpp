// src/web_async.cpp - 모바일 듀얼 스틱 + RC 파이프라인 통합판
// Drone 3D model 있는 버전 - v0.5 (외부 파일 시스템 사용)
#include "web.h"

// === Core/Net ===
#include <WiFi.h>
#include <DNSServer.h>

// === Async Web Server ===
#include <ESPAsyncWebServer.h>    // HTTP
#include <AsyncTCP.h>             // (ESP32)
#include <ArduinoJson.h>          // v7 API

// === File System ===
#include <SPIFFS.h>               // 또는 LittleFS.h 사용 가능
// #include <LittleFS.h>          // LittleFS 사용 시

// ===== 프로젝트 외부 심볼 (기존 코드와 연결) =====
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

// ===== 모듈 내부 객체/상태 =====
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static DNSServer dns;

// 웹 RC on/off 및 안전 타임아웃
static bool  g_webRcEnabled   = false;
static unsigned long g_lastRcMs = 0;
static const unsigned long RC_TIMEOUT_MS = 500; // 0.5s 미입력 시 안전중립

// ===== 유틸 =====
static inline String flightModeName(uint8_t m) {
  switch(m) {
    case 0: return "STABILIZE";
    case 1: return "ANGLE";
    case 2: return "ACRO";
    default: return "UNKNOWN";
  }
}

// ===== 파일 시스템 초기화 =====
bool initFileSystem() {
  // SPIFFS 초기화
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS 초기화 실패!");
    return false;
  }
  
  // LittleFS 사용 시:
  // if (!LittleFS.begin(true)) {
  //   Serial.println("LittleFS 초기화 실패!");
  //   return false;
  // }
  
  Serial.println("파일 시스템 초기화 완료");
  
  // 필수 파일들 존재 확인
  if (!SPIFFS.exists("/index.html")) {
    Serial.println("경고: /index.html 파일이 없습니다!");
  }
  if (!SPIFFS.exists("/css/style.css")) {
    Serial.println("경고: /css/style.css 파일이 없습니다!");
  }
  if (!SPIFFS.exists("/js/drone-controller.js")) {
    Serial.println("경고: /js/drone-controller.js 파일이 없습니다!");
  }
  
  return true;
}

// ===== 파일 제공 헬퍼 함수 =====
void serveFile(AsyncWebServerRequest* request, const char* path, const char* contentType) {
  if (SPIFFS.exists(path)) {
    request->send(SPIFFS, path, contentType);
  } else {
    Serial.printf("파일을 찾을 수 없음: %s\n", path);
    request->send(404, "text/plain", "File Not Found");
  }
}

// ===== 내부 선언 =====
static void handleTelemetry(AsyncWebServerRequest* req);
static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

// ===== 구현 =====
namespace web {

  void begin() {
    // // 파일 시스템 초기화
    // if (!initFileSystem()) {
    //   Serial.println("파일 시스템 초기화 실패 - 웹 서버를 시작할 수 없습니다!");
    //   return;
    // }

    Serial.println("웹 서버 초기화 시작...");

    // CORS & cache
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");
    Serial.println("[WEB] CORS 헤더 설정 완료");

    // 캡티브 포털 우회 라우트(안드/아이폰 미니브라우저 방지)
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/gen_204",     HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    // server.on("/ncsi.txt",    HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/plain","Microsoft NCSI"); });
    // server.on("/hotspot-detect.html", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/html","Success"); });
    // server.on("/success.txt", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/plain","success"); });
    Serial.println("[WEB] 캡티브 포털 라우트 설정 완료");

    // === 메인 HTML 파일 서빙 ===
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    //   serveFile(request, "/index.html", "text/html");
    // });

    // 메인 페이지 - 간단한 테스트 HTML
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
            Serial.println("[WEB] 루트 페이지 요청됨");
            request->send(200, "text/html", "<!DOCTYPE html><html><body><h1>Shane Drone Online</h1></body></html>");
        });
    Serial.println("[WEB] 메인 페이지 라우트 설정 완료");

    // === CSS 파일 서빙 ===
    server.on("/css/style.css", HTTP_GET, [](AsyncWebServerRequest* request) {
      serveFile(request, "/css/style.css", "text/css");
    });

    // === JavaScript 파일 서빙 ===
    server.on("/js/drone-controller.js", HTTP_GET, [](AsyncWebServerRequest* request) {
      serveFile(request, "/js/drone-controller.js", "application/javascript");
    });

    // === 정적 파일 서빙 (선택사항: 추가 리소스들) ===
    // 이미지, 폰트 등 추가 정적 파일들을 위한 일반적인 핸들러
    server.serveStatic("/assets/", SPIFFS, "/assets/");
    server.serveStatic("/images/", SPIFFS, "/images/");
    server.serveStatic("/fonts/", SPIFFS, "/fonts/");

    // === API 엔드포인트 ===
    // Telemetry / Command
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, handleCommand);

    // === WebSocket ===
    ws.onEvent(wsEvent);
    server.addHandler(&ws);

    // === DNS (AP에서만 효과적) ===
    dns.start(53, "*", WiFi.softAPIP());

    // // === 404 핸들러 ===
    // server.onNotFound([](AsyncWebServerRequest* request) {
    //   String message = "File Not Found\n\n";
    //   message += "URI: ";
    //   message += request->url();
    //   message += "\nMethod: ";
    //   message += (request->method() == HTTP_GET) ? "GET" : "POST";
    //   message += "\nArguments: ";
    //   message += request->args();
    //   message += "\n";
      
    //   for (uint8_t i = 0; i < request->args(); i++) {
    //     message += " " + request->argName(i) + ": " + request->arg(i) + "\n";
    //   }
      
    //   Serial.println("404 - " + request->url());
    //   request->send(404, "text/plain", message);
    // });

    server.onNotFound([](AsyncWebServerRequest* request) {
      Serial.printf("404 - %s\n", request->url().c_str());
      request->send(404, "text/plain", "Not Found: " + request->url());
    });

    // Telemetry
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, handleCommand);

    // WebSocket
    ws.onEvent(wsEvent);
    server.addHandler(&ws);

    // DNS
    dns.start(53, "*", WiFi.softAPIP());

    // 서버 시작
    server.begin();
    Serial.println("HTTP+WS 서버 시작됨 (포트 80, 경로 /ws) - 외부 파일 시스템 사용");
    
    // // 메모리 사용량 출력
    // Serial.printf("사용 가능한 힙 메모리: %d bytes\n", ESP.getFreeHeap());
    // Serial.printf("SPIFFS 총 용량: %d bytes\n", SPIFFS.totalBytes());
    // Serial.printf("SPIFFS 사용 용량: %d bytes\n", SPIFFS.usedBytes());

    Serial.println("웹 서버 시작 완료!");
    Serial.print("AP IP 주소: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("자유 힙 메모리: %d bytes\n", ESP.getFreeHeap());
  }

  void loop() {
    // DNS 요청 처리
    dns.processNextRequest();

    // RC 입력 타임아웃(안전중립)
    if (g_webRcEnabled && (millis() - g_lastRcMs > RC_TIMEOUT_MS)) {
      controllerInput.rollNorm = 0.f;
      controllerInput.pitchNorm = 0.f;
      controllerInput.yawNorm = 0.f;
      controllerInput.throttleNorm = 0.f;
    }

    // 10Hz 브로드캐스트
    static unsigned long last = 0;
    const unsigned long now = millis();
    if (now - last >= 100) {
      last = now;

      if (ws.count() > 0) {
        JsonDocument doc; // v7: 자동 확장
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

        doc["armed"]   = systemArmed;
        doc["battery"] = batteryVoltage;
        doc["mode"]    = flightModeName(getFlightMode());

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

  // === 유틸리티 함수들 ===
  
  // 파일 목록 출력 (디버깅용)
  void listFiles() {
    Serial.println("=== SPIFFS 파일 목록 ===");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    
    while (file) {
      Serial.printf("파일: /%s, 크기: %d bytes\n", file.name(), file.size());
      file = root.openNextFile();
    }
    Serial.println("========================");
  }
  
  // 파일 시스템 정보 출력
  void printFileSystemInfo() {
    Serial.printf("SPIFFS 총 용량: %d bytes\n", SPIFFS.totalBytes());
    Serial.printf("SPIFFS 사용 용량: %d bytes\n", SPIFFS.usedBytes());
    Serial.printf("SPIFFS 여유 용량: %d bytes\n", SPIFFS.totalBytes() - SPIFFS.usedBytes());
  }

} // namespace web

// ====== HTTP 핸들러 ======
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

  doc["armed"]   = systemArmed;
  doc["battery"] = batteryVoltage;
  doc["mode"]    = flightModeName(getFlightMode());

  doc["input"]["throttle"] = controllerInput.throttleNorm;
  doc["input"]["roll"]     = controllerInput.rollNorm;
  doc["input"]["pitch"]    = controllerInput.pitchNorm;
  doc["input"]["yaw"]      = controllerInput.yawNorm;

  doc["rcEnabled"] = g_webRcEnabled;

  String out; serializeJson(doc, out);
  req->send(200, "application/json", out);
}

// === RC 디버그 출력 간격(Hz) ===
#ifndef RC_DEBUG_HZ
#define RC_DEBUG_HZ 1   // 1Hz로 로그 (50Hz 그대로 찍으면 너무 많아요)
#endif

static inline void rcDebug(float thr, float roll, float pitch, float yaw, bool applied) {
  static unsigned long last=0; 
  const unsigned long now=millis(), interval=1000/RC_DEBUG_HZ;
  if (now - last >= interval) {
    last = now;
    Serial.printf("[RC %s] thr=%.2f roll=%.2f pitch=%.2f yaw=%.2f\n", applied?"APPLY":"RX", thr, roll, pitch, yaw);
  }
}

static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) { req->send(400, "text/plain", "JSON parse error"); return; }

  // 1) RC payload
  if (doc["rc"].is<JsonObject>()) {
    JsonObject r = doc["rc"].as<JsonObject>();
    g_lastRcMs = millis();
    if (g_webRcEnabled) {
      controllerInput.rollNorm     = constrain((float)r["roll"],  -1.f, 1.f);
      controllerInput.pitchNorm    = constrain((float)r["pitch"], -1.f, 1.f);
      controllerInput.yawNorm      = constrain((float)r["yaw"],   -1.f, 1.f);
      controllerInput.throttleNorm = constrain((float)r["throttle"], 0.f, 1.f);
      // ✅ 적용된 값 로그
      rcDebug(controllerInput.throttleNorm, controllerInput.rollNorm, controllerInput.pitchNorm, controllerInput.yawNorm, true);
    } else {
        // ✅ RC가 들어오긴 하지만 적용은 안 된 경우도 확인용 로그
        rcDebug((float)r["throttle"], (float)r["roll"], (float)r["pitch"], (float)r["yaw"], false);
    }
    req->send(200, "text/plain", "rc ok");
    return;
  }

  // 2) Command
  const char* cmd = doc["command"] | "";
  if (!strcmp(cmd,"CALIBRATE"))         { calibrateSensors();         req->send(200,"text/plain","CALIBRATE ok"); }
  else if (!strcmp(cmd,"RESET_PID"))     { resetPIDIntegrals();        req->send(200,"text/plain","RESET_PID ok"); }
  else if (!strcmp(cmd,"EMERGENCY_STOP")){ systemArmed=false; stopAllMotors(); req->send(200,"text/plain","EMERGENCY_STOP ok"); }
  else if (!strcmp(cmd,"ENABLE_WEB_RC")) { g_webRcEnabled=true;        req->send(200,"text/plain","WEB_RC on"); }
  else if (!strcmp(cmd,"DISABLE_WEB_RC")){ g_webRcEnabled=false;
                                            controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f;
                                            controllerInput.throttleNorm=0.f;
                                            req->send(200,"text/plain","WEB_RC off"); }
  else { req->send(400,"text/plain","unknown command"); }
}

static void wsEvent(AsyncWebSocket* /*server*/, AsyncWebSocketClient* /*client*/, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch(type){
    case WS_EVT_CONNECT:    Serial.println("[WS] client 연결"); break;
    case WS_EVT_DISCONNECT: Serial.println("[WS] client 종료"); break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT){
        JsonDocument doc;
        if (deserializeJson(doc, data, len)==DeserializationError::Ok) {
          // RC 우선
          if (doc["rc"].is<JsonObject>()) {
            JsonObject r = doc["rc"].as<JsonObject>();
            g_lastRcMs = millis();
            if (g_webRcEnabled) {
              controllerInput.rollNorm     = constrain((float)r["roll"],  -1.f, 1.f);
              controllerInput.pitchNorm    = constrain((float)r["pitch"], -1.f, 1.f);
              controllerInput.yawNorm      = constrain((float)r["yaw"],   -1.f, 1.f);
              controllerInput.throttleNorm = constrain((float)r["throttle"], 0.f, 1.f);
                // ✅ 적용된 값 로그
                rcDebug(controllerInput.throttleNorm, controllerInput.rollNorm, controllerInput.pitchNorm, controllerInput.yawNorm, true);
              } else {
                // ✅ 적용 안 되는 상태(RC OFF) 로그
                rcDebug((float)r["throttle"], (float)r["roll"], (float)r["pitch"], (float)r["yaw"], false);
            }
            break;
          }
          // Command
          const char* cmd = doc["command"] | "";
          if (!strcmp(cmd,"CALIBRATE"))          calibrateSensors();
          else if (!strcmp(cmd,"RESET_PID"))     resetPIDIntegrals();
          else if (!strcmp(cmd,"EMERGENCY_STOP")){ systemArmed=false; stopAllMotors(); }
          else if (!strcmp(cmd,"ENABLE_WEB_RC"))  g_webRcEnabled=true;
          else if (!strcmp(cmd,"DISABLE_WEB_RC")) { g_webRcEnabled=false;
                                                    controllerInput.rollNorm=controllerInput.pitchNorm=controllerInput.yawNorm=0.f;
                                                    controllerInput.throttleNorm=0.f; }
        }
      }
      break;
    }
    default: break;
  }
}