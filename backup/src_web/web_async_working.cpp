// web_async.cpp - 수정된 동작 버전
#include "../include/web.h"

// Core/Net
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
// #include <SPIFFS.h>
// #include <LittleFS.h>  // SPIFFS 대신 LittleFS

// External symbols
extern bool systemArmed;
extern float batteryVoltage;

// getFlightMode 함수 선언 추가
// uint8_t getFlightMode() {
//     // control.cpp에서 extern으로 가져오거나 기본값 반환
//     // return currentFlightMode;
//     return 0; // STABILIZE mode as default
// }
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

// Flight mode name helper
static inline String flightModeName(uint8_t m) {
  switch(m) {
    case 0: return "STABILIZE";
    case 1: return "ANGLE";
    case 2: return "ACRO";
    default: return "UNKNOWN";
  }
}

// Simplified HTML (no external CDN, all inline)
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Shane Drone</title>
<style>
:root{--card:rgba(255,255,255,.08);--stroke:rgba(255,255,255,.18);--stick-size:180px}
*{box-sizing:border-box;margin:0;padding:0}
body{font:16px/1.45 system-ui,-apple-system,sans-serif;background:linear-gradient(135deg,#1e1e2e,#2a2a3e);color:#fff;padding:18px;padding-bottom:220px}
.wrap{max-width:980px;margin:0 auto}
h1{font-size:20px;margin:0 0 8px}
.muted{opacity:.75;font-size:13px;margin-bottom:12px}
.row{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(220px,1fr))}
.card{background:var(--card);border:1px solid var(--stroke);border-radius:14px;padding:12px;backdrop-filter:blur(8px)}
.kv{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px dashed var(--stroke)}
.kv:last-child{border:none}
.bar{height:8px;background:#222;border-radius:999px;overflow:hidden}
.fill{height:100%;width:0%;transition:width .2s;background:linear-gradient(90deg,#7cf,#fc7,#f66)}
.ws{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;background:#f55}
.ws.ok{background:#6f6}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
button{border:0;border-radius:12px;padding:10px 12px;color:#001;cursor:pointer;font-weight:700}
.b{background:#4fc3f7}.r{background:#ff8a80}.g{background:#80e27e}.y{background:#ffd54f}
.rcpill{display:inline-block;padding:6px 10px;border-radius:999px;background:rgba(255,255,255,.12);border:1px solid var(--stroke);margin-left:8px;cursor:pointer}
.rcpill.on{background:#80e27e;color:#003300}
.sticks{position:fixed;left:0;right:0;bottom:0;pointer-events:none;padding:12px;display:flex;justify-content:space-between;gap:12px}
.stick{width:var(--stick-size);height:var(--stick-size);pointer-events:auto;touch-action:none;position:relative;background:rgba(255,255,255,.06);border:1px solid var(--stroke);border-radius:14px}
.base,.knob{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);border-radius:50%}
.base{width:56%;height:56%;border:1px solid rgba(255,255,255,.4)}
.knob{width:28%;height:28%;background:rgba(255,255,255,.9);box-shadow:0 4px 20px rgba(0,0,0,.35)}
</style>
</head><body>

<div class="wrap">
  <h1><span id="ws" class="ws"></span>Shane Drone
    <span id="rcpill" class="rcpill">RC OFF</span>
  </h1>
  <div class="muted">실시간 상태 · 모바일 듀얼 스틱</div>

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
        <button class="y" onclick="toggleRc()">RC 토글</button>
      </div>
    </div>
  </div>
</div>

<div class="sticks">
  <div id="stickL" class="stick"><div class="base"></div><div class="knob" id="knobL"></div></div>
  <div id="stickR" class="stick"><div class="base"></div><div class="knob" id="knobR"></div></div>
</div>

<script>
const $=s=>document.querySelector(s);
let ws,tRetry,rcEnabled=false,rc={throttle:0,yaw:0,roll:0,pitch:0};

function connectWS(){
  const wsUrl = 'ws://' + window.location.host + '/ws';
  console.log('Connecting to:', wsUrl);
  ws = new WebSocket(wsUrl);
  ws.onopen = ()=>{ 
    console.log('WebSocket connected');
    $('#ws').classList.add('ok'); 
    if(tRetry) clearTimeout(tRetry); 
  };
  ws.onclose = ()=>{ 
    console.log('WebSocket disconnected');
    $('#ws').classList.remove('ok'); 
    tRetry=setTimeout(connectWS,2000); 
  };
  ws.onerror = (e)=>{ 
    console.error('WebSocket error:', e); 
  };
  ws.onmessage = e => { 
    try{ 
      update(JSON.parse(e.data||'{}')); 
    }catch(err){
      console.error('Parse error:', err);
    } 
  };
}
connectWS();

function setText(sel,text){const el=$(sel);if(el)el.textContent=text;}
function pct(v){return Math.max(0,Math.min(100,((v-1000)/1000)*100));}
function bar(id,val,label){$(id).style.width=pct(val)+'%';$(label).textContent=val;}

function update(d){
  if(d.attitude){ 
    setText('#roll',d.attitude.roll.toFixed(1)+'°');
    setText('#pitch',d.attitude.pitch.toFixed(1)+'°');
    setText('#yaw',d.attitude.yaw.toFixed(1)+'°');
  }
  if(typeof d.battery!=='undefined') setText('#battery',d.battery.toFixed(2)+'V');
  if(typeof d.mode!=='undefined') setText('#mode',d.mode);
  if(typeof d.armed!=='undefined') setText('#arm',d.armed?'ARMED':'DISARMED');
  
  if(d.motors){ 
    bar('#mfl',d.motors.fl,'#mflv');
    bar('#mfr',d.motors.fr,'#mfrv');
    bar('#mrl',d.motors.rl,'#mrlv');
    bar('#mrr',d.motors.rr,'#mrrv');
  }
  if(typeof d.rcEnabled!=='undefined'){
    rcEnabled=!!d.rcEnabled;
    paintRcPill();
  }

  if(d.input){
    setText('#rcRoll',d.input.roll.toFixed(2));
    setText('#rcPitch',d.input.pitch.toFixed(2));
    setText('#rcYaw',d.input.yaw.toFixed(2));
    setText('#rcThr',(d.input.throttle*100).toFixed(0)+'%');
  }
}

function cmd(c){
  const payload = JSON.stringify({command:c});
  if(ws && ws.readyState===1){
    ws.send(payload);
  }else{
    fetch('/command',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:payload
    }).catch(err=>console.error('Command error:',err));
  }
}

function toggleRc(){
  rcEnabled=!rcEnabled;
  paintRcPill();
  cmd(rcEnabled?'ENABLE_WEB_RC':'DISABLE_WEB_RC');
}

function paintRcPill(){
  const pill=$('#rcpill');
  pill.textContent=rcEnabled?'RC ON':'RC OFF';
  pill.classList.toggle('on',rcEnabled);
}

// Dual stick implementation
function makeStick(rootSel,knobSel){
  const el=$(rootSel),knob=$(knobSel);
  if(!el || !knob) return {value:()=>({x:0,y:0})};
  
  const rect=()=>el.getBoundingClientRect();
  let active=false,cx=0,cy=0,kx=0,ky=0,maxR=0;
  
  function center(){const r=rect();cx=r.left+r.width/2;cy=r.top+r.height/2;maxR=Math.min(r.width,r.height)*0.28;}
  function setKnob(x,y){knob.style.transform='translate('+x+'px,'+y+'px)';}
  function norm(dx,dy){const r=Math.hypot(dx,dy)||1;const rr=Math.min(r,maxR);return{x:(dx/rr)*(rr/maxR),y:(dy/rr)*(rr/maxR)};}
  
  function onDown(e){
    e.preventDefault();
    active=true;
    center();
    onMove(e);
  }
  
  function onMove(e){
    if(!active)return;
    e.preventDefault();
    const t=(e.touches?e.touches[0]:e);
    const dx=t.clientX-cx,dy=t.clientY-cy;
    const v=norm(dx,dy);
    kx=v.x*maxR;ky=v.y*maxR;
    setKnob(kx,ky);
  }
  
  function onUp(){active=false;kx=ky=0;setKnob(0,0);}
  
  el.addEventListener('pointerdown',onDown);
  el.addEventListener('pointermove',onMove);
  el.addEventListener('pointerup',onUp);
  el.addEventListener('pointercancel',onUp);
  el.addEventListener('touchstart',onDown,{passive:false});
  el.addEventListener('touchmove',onMove,{passive:false});
  el.addEventListener('touchend',onUp);
  
  window.addEventListener('resize',center);
  center();
  
  return{value(){return{x:maxR?kx/maxR:0,y:maxR?ky/maxR:0};}}
}

$('#rcpill').addEventListener('click',toggleRc);

const L=makeStick('#stickL','#knobL');
const R=makeStick('#stickR','#knobR');

// Send RC data at 50Hz
setInterval(()=>{
  if(!rcEnabled)return;
  const lv=L.value(),rv=R.value();
  rc.throttle=Math.max(0,Math.min(1,(-lv.y+1)/2));
  rc.yaw=Math.max(-1,Math.min(1,lv.x));
  rc.roll=Math.max(-1,Math.min(1,rv.x));
  rc.pitch=Math.max(-1,Math.min(1,-rv.y));
  
  const payload=JSON.stringify({rc});
  if(ws && ws.readyState===1){
    ws.send(payload);
  }else{
    fetch('/command',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:payload
    }).catch(()=>{});
  }
},20);

// Fallback telemetry
setInterval(()=>{
  if(!ws || ws.readyState!==1){
    fetch('/telemetry')
      .then(r=>r.json())
      .then(update)
      .catch(()=>{});
  }
},1000);
</script>
</body></html>
)HTML";

// Handler declarations
static void handleTelemetry(AsyncWebServerRequest* req);
static void handleCommand(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total);
static void wsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

// Implementation
namespace web {

  void begin() {
    Serial.println("Initializing web server...");
    
    // Set default headers (simplified)
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache");
    
    // Routes for captive portal
    server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    server.on("/gen_204", HTTP_ANY, [](AsyncWebServerRequest* r){ r->send(204); });
    
    // Main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ 
      r->send(200, "text/html", INDEX_HTML);
    });
    
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
    Serial.println("HTTP+WS server started (port 80, path /ws)");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
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

// HTTP handlers
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