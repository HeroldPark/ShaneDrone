// drone_html.h - HTML 구조만 포함
#ifndef DRONE_HTML_H
#define DRONE_HTML_H

const char DRONE_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shane Drone 3D Control</title>
<style>%CSS_CONTENT%</style>
</head><body>

<div id="canvas3d"></div>

<div class="overlay top-left">
  <h3><span class="ws" id="ws"></span>연결 상태<span class="rcpill" id="rcpill">(RC OFF)</span></h3>
  <div class="kv"><span>ARM:</span><b id="arm">DISARMED</b></div>
  <div class="kv"><span>모드:</span><b id="mode">STABILIZE</b></div>
  <div class="kv"><span>배터리:</span><b id="battery">0.00V</b></div>
</div>

<div class="overlay top-right">
  <h3>자세 정보</h3>
  <div class="kv"><span>Roll:</span><b id="roll">0.0°</b></div>
  <div class="kv"><span>Pitch:</span><b id="pitch">0.0°</b></div>
  <div class="kv"><span>Yaw:</span><b id="yaw">0.0°</b></div>
</div>

<div class="overlay left-below">
  <h3>조이스틱 입력</h3>
  <div class="kv"><span>RC Roll:</span><b id="rcRoll">0.00</b></div>
  <div class="kv"><span>RC Pitch:</span><b id="rcPitch">0.00</b></div>
  <div class="kv"><span>RC Yaw:</span><b id="rcYaw">0.00</b></div>
  <div class="kv"><span>RC Throttle:</span><b id="rcThr">0%</b></div>
</div>

<div class="overlay right-below">
  <h3>모터 출력</h3>
  <div class="motor-grid">
    <div class="motor-item">
      <div>FL</div>
      <div class="motor-bar"><div class="motor-fill" id="mfl"></div></div>
      <div id="mflv">1000</div>
    </div>
    <div class="motor-item">
      <div>FR</div>
      <div class="motor-bar"><div class="motor-fill" id="mfr"></div></div>
      <div id="mfrv">1000</div>
    </div>
    <div class="motor-item">
      <div>RL</div>
      <div class="motor-bar"><div class="motor-fill" id="mrl"></div></div>
      <div id="mrlv">1000</div>
    </div>
    <div class="motor-item">
      <div>RR</div>
      <div class="motor-bar"><div class="motor-fill" id="mrr"></div></div>
      <div id="mrrv">1000</div>
    </div>
  </div>
</div>

<div class="overlay bottom-center">
  <h3>제어</h3>
  <div class="btns">
    <button onclick="cmd('CALIBRATE')">캘리브레이션</button>
    <button onclick="cmd('RESET_PID')">PID 리셋</button>
    <button class="danger" onclick="cmd('EMERGENCY_STOP')">비상정지</button>
    <button onclick="toggleRc()">RC 토글</button>
  </div>
</div>

<div class="sticks">
  <div class="stick" id="stickL"><div class="knob" id="knobL"></div></div>
  <div class="stick" id="stickR"><div class="knob" id="knobR"></div></div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>%JS_CONTENT%</script>
</body></html>
)HTML";

#endif // DRONE_HTML_H