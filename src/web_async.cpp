// src/web_async.cpp - 모바일 듀얼 스틱 + RC 파이프라인 통합판
#include "web.h"

// === Core/Net ===
#include <WiFi.h>
#include <DNSServer.h>

// === Async Web Server ===
#include <ESPAsyncWebServer.h>    // HTTP
#include <AsyncTCP.h>             // (ESP32)
#include <ArduinoJson.h>          // v7 API

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

// ===== 미니 대시보드 + 듀얼 스틱(내장) HTML =====
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Shane Drone</title>
<style>
  :root { --card: rgba(255,255,255,.08); --stroke: rgba(255,255,255,.18); --stick-size: clamp(180px, 42vw, 320px); } /* 스틱 크기 */
  .stick { width: var(--stick-size); height: var(--stick-size); }
  *{box-sizing:border-box} 
  /* 본문을 스틱 높이만큼 위로 올려 버튼이 겹치지 않게 */
  body{padding-bottom: calc(var(--stick-size) + 28px + env(safe-area-inset-bottom));
  margin:0;font:16px/1.45 system-ui,Segoe UI,Roboto,Apple SD Gothic Neo,sans-serif;background:
  linear-gradient(135deg,#1e1e2e 0%,#2a2a3e 100%);color:#fff }
  .wrap{max-width:980px;margin:0 auto;padding:18px}
  h1{font-size:20px;margin:0 0 8px}
  .muted{opacity:.75;font-size:13px;margin-bottom:12px}
  .row{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(220px,1fr))}
  .card{background:var(--card);border:1px solid var(--stroke);border-radius:14px;padding:12px;backdrop-filter:blur(8px)}
  .kv{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px dashed var(--stroke)}
  .kv:last-child{border:none}
  .bar{height:8px;background:#222;border-radius:999px;overflow:hidden}
  .fill{height:100%;width:0%;transition:width .2s ease;background:linear-gradient(90deg,#7cf,#fc7,#f66)}
  .ws{display:inline-block;width:10px;height:10px;border-radius:50%;vertical-align:-1px;margin-right:6px;background:#f55}
  .ws.ok{background:#6f6}
  .btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
  button{appearance:none;border:0;border-radius:12px;padding:10px 12px;color:#001;cursor:pointer;font-weight:700}
  .b{background:#4fc3f7}.r{background:#ff8a80}.g{background:#80e27e}.y{background:#ffd54f}

  /* 세로 높이가 더 작은 폰에서 자동 축소 */
  @media (max-height: 700px) { :root { --stick-size: clamp(160px, 34vw, 260px); } }
  @media (max-height: 560px) { :root { --stick-size: clamp(140px, 28vw, 220px); } }

  /* 상단 RC pill을 토글용으로도 쓰기 위해 포인터 표시 */
  .rcpill { cursor: pointer; }

  /* 듀얼 스틱 오버레이 */
  .sticks{position:fixed;left:0;right:0;bottom:0;pointer-events:none;padding:12px;display:flex;justify-content:space-between;gap:12px}
  .stick{pointer-events:auto;touch-action:none;position:relative;
         background:rgba(255,255,255,.06);border:1px solid var(--stroke);border-radius:14px}
  .base,.knob{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);border-radius:50%}
  .base{width:56%;height:56%;border:1px solid rgba(255,255,255,.4)}
  .knob{width:28%;height:28%;background:rgba(255,255,255,.9);box-shadow:0 4px 20px rgba(0,0,0,.35)}
  .rcpill{display:inline-block;padding:6px 10px;border-radius:999px;background:rgba(255,255,255,.12);
          border:1px solid var(--stroke);margin-left:8px}
  .rcpill.on{background:#80e27e;color:#003300}

  /* 자세,입력값 */
  .kv3{
    display:grid;
    grid-template-columns: 1fr auto auto; /* 항목 | 센서 | RC */
    align-items:center;
    gap:10px;
    padding:6px 0;
    border-bottom:1px dashed var(--stroke);
  }
  .kv3.head{ font-weight:700; opacity:.85; border-bottom:1px solid var(--stroke); }
</style></head><body>

<div class="wrap">
  <h1><span id="ws" class="ws"></span>Shane Drone
    <span id="rcpill" class="rcpill">RC OFF</span>
  </h1>
  <div class="muted">실시간 상태 · 모바일 듀얼 스틱(왼쪽: 스로틀/요, 오른쪽: 롤/피치)</div>

  <div class="row">
    <div class="card">
      <div>센서 측정값</div>
      <div class="kv"><span>Roll</span><b id="roll">0.0°</b></div>
      <div class="kv"><span>Pitch</span><b id="pitch">0.0°</b></div>
      <div class="kv"><span>Yaw</span><b id="yaw">0.0°</b></div>
      <div class="kv"><span>배터리</span><b id="battery">0.00V</b></div>
      <div class="kv"><span>비행모드</span><b id="mode">—</b></div>
      <div class="kv"><span>ARM</span><b id="arm">DISARMED</b></div>
    </div>

    <div class="card">
      <div>조이스틱 입력</div>
      <div class="kv"><span>RC Roll</span><b id="rcRoll">0.00</b></div>
      <div class="kv"><span>RC Pitch</span><b id="rcPitch">0.00</b></div>
      <div class="kv"><span>RC Yaw</span><b id="rcYaw">0.00</b></div>
      <div class="kv"><span>RC Throttle</span><b id="rcThr">0%</b></div>
    </div>

    <!--
    <div class="card">
      <div>자세·입력</div>
      <div class="kv3 head"><span>항목</span><span>센서</span><span>RC</span></div>

      <div class="kv3"><span>Roll</span><b id="roll">0.0°</b><b id="rcRoll">0.00</b></div>
      <div class="kv3"><span>Pitch</span><b id="pitch">0.0°</b><b id="rcPitch">0.00</b></div>
      <div class="kv3"><span>Yaw</span><b id="yaw">0.0°</b><b id="rcYaw">0.00</b></div>
      <div class="kv3"><span>Throttle</span><span>—</span><b id="rcThr">0%</b></div>

      <div class="kv"><span>배터리</span><b id="battery">0.00V</b></div>
      <div class="kv"><span>비행모드</span><b id="mode">—</b></div>
      <div class="kv"><span>ARM</span><b id="arm">DISARMED</b></div>
    </div>
    -->

    <div class="card">
      <div>모터 출력</div>
      <div class="kv"><span>FL</span><b id="mflv">1000</b></div><div class="bar"><div id="mfl" class="fill"></div></div>
      <div class="kv"><span>FR</span><b id="mfrv">1000</b></div><div class="bar"><div id="mfr" class="fill"></div></div>
      <div class="kv"><span>RL</span><b id="mrlv">1000</b></div><div class="bar"><div id="mrl" class="fill"></div></div>
      <div class="kv"><span>RR</span><b id="mrrv">1000</b></div><div class="bar"><div id="mrr" class="fill"></div></div>
    </div>

    <div class="card">
      <div>제어 & 명령</div>
      <div class="btns">
        <button class="b" onclick="cmd('CALIBRATE')">센서 캘리브레이션</button>
        <button class="g" onclick="cmd('RESET_PID')">PID 리셋</button>
        <button class="r" onclick="cmd('EMERGENCY_STOP')">비상 정지</button>
        <button class="y" id="toggleRcBtn" onclick="toggleRc()">RC 토글</button>
      </div>
    </div>
  </div>
</div>

<!-- 듀얼 스틱 -->
<div class="sticks">
  <div id="stickL" class="stick"><div class="base"></div><div class="knob" id="knobL"></div></div>
  <div id="stickR" class="stick"><div class="base"></div><div class="knob" id="knobR"></div></div>
</div>

<script>
const $=s=>document.querySelector(s);
let ws,tRetry,rcEnabled=false, rc={throttle:0,yaw:0,roll:0,pitch:0};

// WS 연결 (80포트 /ws)
function connectWS(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = ()=>{ $('#ws').classList.add('ok'); if(tRetry) clearTimeout(tRetry); };
  ws.onclose = ()=>{ $('#ws').classList.remove('ok'); tRetry=setTimeout(connectWS,1200); };
  ws.onmessage = e => { try{ update(JSON.parse(e.data||'{}')); }catch{} };
}
connectWS();

// 폴링 백업(WS 끊기면)
setInterval(()=>{
  if(!ws || ws.readyState!==1){
    fetch('/telemetry').then(r=>r.json()).then(update).catch(()=>{});
  }
}, 900);

// UI 업데이트
function pct(v){ return Math.max(0, Math.min(100, ((v-1000)/1000)*100 )); }
function bar(id,val,label){ $(id).style.width = pct(val)+'%'; $(label).textContent = val; }

function setText(sel, text){ const el=document.querySelector(sel); if(el) el.textContent=text; }

function update(d){
  if(d.attitude){ 
    setText('#roll',  d.attitude.roll.toFixed(1)+'°');
    setText('#pitch', d.attitude.pitch.toFixed(1)+'°');
    setText('#yaw',   d.attitude.yaw.toFixed(1)+'°');
  }
  if(typeof d.battery!=='undefined') setText('#battery', d.battery.toFixed(2)+'V');
  if(typeof d.mode!=='undefined')    setText('#mode', d.mode);
  if(typeof d.armed!=='undefined')   setText('#arm', d.armed?'ARMED':'DISARMED');
  
  if(d.motors){ 
    bar('#mfl',d.motors.fl,'#mflv'); bar('#mfr',d.motors.fr,'#mfrv');
    bar('#mrl',d.motors.rl,'#mrlv'); bar('#mrr',d.motors.rr,'#mrrv'); 
  }
  if(typeof d.rcEnabled!=='undefined'){
    rcEnabled = !!d.rcEnabled; paintRcPill();
  }

  // RC 표시 로직 추가
  if (d.input) {
    if(typeof d.input.roll==='number')   setText('#rcRoll', d.input.roll.toFixed(2));
    if(typeof d.input.pitch==='number')  setText('#rcPitch', d.input.pitch.toFixed(2));
    if(typeof d.input.yaw==='number')    setText('#rcYaw', d.input.yaw.toFixed(2));
    if(typeof d.input.throttle==='number') setText('#rcThr', (d.input.throttle*100).toFixed(0)+'%');
  }

}

// 명령 전송
function cmd(c){
  const p = JSON.stringify({command:c});
  if(ws && ws.readyState===1) ws.send(p);
  else fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:p}).catch(()=>{});
}

// RC 토글
function toggleRc(){
  rcEnabled = !rcEnabled; paintRcPill();
  cmd(rcEnabled ? 'ENABLE_WEB_RC' : 'DISABLE_WEB_RC');
}
function paintRcPill(){
  const pill = $('#rcpill');
  pill.textContent = rcEnabled ? 'RC ON' : 'RC OFF';
  pill.classList.toggle('on', rcEnabled);
}

// === 듀얼 스틱(내장 구현, 외부 CDN 불필요) ===
function makeStick(rootSel, knobSel){
  const el = $(rootSel), knob=$(knobSel);
  const rect = ()=>el.getBoundingClientRect();
  let active=false, cx=0, cy=0, kx=0, ky=0, maxR=0;
  function center(){ const r=rect(); cx=r.left+r.width/2; cy=r.top+r.height/2; maxR=Math.min(r.width,r.height)*0.28; }
  function setKnob(x,y){ knob.style.transform=`translate(${x}px,${y}px)`; }
  function norm(dx,dy){ const r=Math.hypot(dx,dy)||1; const rr=Math.min(r,maxR); return { x:(dx/rr)*(rr/maxR), y:(dy/rr)*(rr/maxR) }; }
  function onDown(e){ active=true; const t=(e.touches?e.touches[0]:e); center(); onMove(e); }
  function onMove(e){
    if(!active) return;
    const t=(e.touches?e.touches[0]:e);
    const dx=t.clientX-cx, dy=t.clientY-cy;
    const v=norm(dx,dy); kx=v.x*maxR; ky=v.y*maxR; setKnob(kx,ky);
  }
  function onUp(){ active=false; kx=ky=0; setKnob(0,0); }
  el.addEventListener('pointerdown', onDown); el.addEventListener('pointermove', onMove);
  el.addEventListener('pointerup', onUp); el.addEventListener('pointercancel', onUp); el.addEventListener('pointerleave', onUp);
  el.addEventListener('touchstart', onDown,{passive:false}); el.addEventListener('touchmove', onMove,{passive:false});
  el.addEventListener('touchend', onUp); el.addEventListener('touchcancel', onUp);
  window.addEventListener('resize', center); center();
  return {
    value(){ // -1..+1
      const nx = (maxR? kx/maxR : 0);
      const ny = (maxR? ky/maxR : 0);
      return {x: nx, y: ny};
    }
  };
}

$('#rcpill').addEventListener('click', toggleRc);

const L = makeStick('#stickL','#knobL'); // 스로틀/요
const R = makeStick('#stickR','#knobR'); // 롤/피치

// 50Hz로 RC 전송
setInterval(()=>{
  if(!rcEnabled) return;
  const lv=L.value(), rv=R.value();
  // 매핑: 왼쪽 Y가 위로 갈수록 스로틀↑  (0..1), 왼쪽 X는 요(-1..+1)
  rc.throttle = Math.max(0, Math.min(1, (-lv.y+1)/2));
  rc.yaw      = Math.max(-1, Math.min(1, lv.x));
  // 오른쪽 X/Y는 롤/피치(-1..+1), 화면 위쪽이 +피치
  rc.roll     = Math.max(-1, Math.min(1, rv.x));
  rc.pitch    = Math.max(-1, Math.min(1, -rv.y));

  const payload = JSON.stringify({ rc });
  if(ws && ws.readyState===1) ws.send(payload);
  else fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body: payload}).catch(()=>{});
}, 20);
</script>
</body></html>
)HTML";

// ===== 내부 선언 =====
static void handleTelemetry(AsyncWebServerRequest* req);
static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

// ===== 구현 =====
namespace web {

  void begin() {
    // CORS & cache
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");

    // 캡티브 포털 우회 라우트(안드/아이폰 미니브라우저 방지)
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/gen_204",     HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/ncsi.txt",    HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/plain","Microsoft NCSI"); });
    server.on("/hotspot-detect.html", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/html","Success"); });
    server.on("/success.txt", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(200,"text/plain","success"); });

    // 정적 루트
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send(200, "text/html", INDEX_HTML); });

    // Telemetry / Command
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/command", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, handleCommand);

    // WebSocket
    ws.onEvent(wsEvent);
    server.addHandler(&ws);

    // DNS (AP에서만 효과적)
    dns.start(53, "*", WiFi.softAPIP());

    // 시작
    server.begin();
    Serial.println("HTTP+WS 서버 시작됨 (포트 80, 경로 /ws)");
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

  void setRcEnabled(bool on) {
    g_webRcEnabled = on;
    if (!on) {
      controllerInput.rollNorm = controllerInput.pitchNorm = controllerInput.yawNorm = 0.f;
      controllerInput.throttleNorm = 0.f;
    }
  }
  bool rcEnabled() { return g_webRcEnabled; }

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



