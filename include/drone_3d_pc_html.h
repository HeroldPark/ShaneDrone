// drone_3d_html.h - 3D 시각화 HTML을 포함하는 헤더 파일
#ifndef DRONE_3D_HTML_H
#define DRONE_3D_HTML_H

const char DRONE_3D_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shane Drone 3D Control</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font:14px/1.5 system-ui,-apple-system,sans-serif;background:#0a0a0a;color:#fff;overflow:hidden}
#canvas3d{position:fixed;top:0;left:0;width:100%;height:100%;z-index:1}
.overlay{position:fixed;z-index:10;background:rgba(0,0,0,0.85);border-radius:12px;padding:15px;backdrop-filter:blur(10px)}
.top-left{top:20px;left:20px;min-width:200px}
.top-right{top:20px;right:20px;min-width:200px}
.bottom-left{bottom:20px;left:20px}
.top-center{top:20px;left:50%;transform:translateX(-50%);min-width:200px}
.bottom-right{bottom:20px;right:20px;min-width:250px}
h3{color:#4fc3f7;margin:0 0 10px;font-size:16px}
.kv{display:flex;justify-content:space-between;padding:4px 0;font-size:13px}
.kv span:first-child{color:rgba(255,255,255,0.7)}
.kv b{color:#4fc3f7}
.motor-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
.motor-item{background:rgba(255,255,255,0.05);padding:8px;border-radius:6px;text-align:center}
.motor-bar{height:6px;background:#333;border-radius:3px;overflow:hidden;margin:4px 0}
.motor-fill{height:100%;background:linear-gradient(90deg,#4fc3f7,#29b6f6);transition:width 0.2s}
.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
button{padding:8px 12px;border:0;border-radius:6px;background:#4fc3f7;color:#000;font-weight:600;cursor:pointer}
button:hover{background:#29b6f6}
.danger{background:#f44336}
.danger:hover{background:#e53935}
.success{background:#4caf50}
.ws{display:inline-block;width:8px;height:8px;border-radius:50%;background:#f44336;margin-right:8px}
.ws.ok{background:#4caf50}
.sticks{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);display:flex;gap:20px;z-index:20}
.stick{width:120px;height:120px;background:rgba(255,255,255,0.08);border:1px solid rgba(255,255,255,0.2);border-radius:12px;position:relative;touch-action:none}
.knob{width:36px;height:36px;background:radial-gradient(circle,#4fc3f7,#1976d2);border-radius:50%;position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);box-shadow:0 4px 12px rgba(0,0,0,0.5)}
.rcpill{display:inline-block;padding:4px 10px;border-radius:12px;background:rgba(255,255,255,0.1);margin-left:10px;cursor:pointer;font-size:12px}
.rcpill.on{background:#4caf50;color:#000}
</style>
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

<div class="overlay top-center">
  <h3>조이스틱 입력</h3>
  <div class="kv"><span>RC Roll:</span><b id="rcRoll">0.00</b></div>
  <div class="kv"><span>RC Pitch:</span><b id="rcPitch">0.00</b></div>
  <div class="kv"><span>RC Yaw:</span><b id="rcYaw">0.00</b></div>
  <div class="kv"><span>RC Throttle:</span><b id="rcThr">0%</b></div>
</div>

<div class="overlay bottom-left">
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

<div class="overlay bottom-right">
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
<script>
// WebSocket & Control
const $=s=>document.querySelector(s);
let ws,rcEnabled=false,rc={throttle:0,yaw:0,roll:0,pitch:0};
let droneData={roll:0,pitch:0,yaw:0,throttle:0,motors:{fl:1000,fr:1000,rl:1000,rr:1000}};

// Three.js variables
let scene,camera,renderer,drone,propellers=[];

function init3D(){
  const container=$('#canvas3d');
  
  // Scene
  scene=new THREE.Scene();
  scene.background=new THREE.Color(0x0a0a0a);
  
  // Camera
  camera=new THREE.PerspectiveCamera(60,window.innerWidth/window.innerHeight,0.1,1000);
  camera.position.set(3,3,3);
  camera.lookAt(0,0,0);
  
  // Renderer
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});
  renderer.setSize(window.innerWidth,window.innerHeight);
  renderer.shadowMap.enabled=true;
  container.appendChild(renderer.domElement);
  
  // Lights
  const ambient=new THREE.AmbientLight(0x404040,0.8);
  scene.add(ambient);
  const directional=new THREE.DirectionalLight(0xffffff,0.6);
  directional.position.set(5,5,5);
  directional.castShadow=true;
  scene.add(directional);
  
  // Drone model
  createDrone();
  
  // Ground grid
  const grid=new THREE.GridHelper(10,20,0x4fc3f7,0x4fc3f7);
  grid.material.transparent=true;
  grid.material.opacity=0.2;
  scene.add(grid);
  
  // Axes helper
  const axes=new THREE.AxesHelper(0.5);
  scene.add(axes);
  
  animate3D();
}

function createDrone(){
  drone=new THREE.Group();
  
  // Body
  const bodyGeo=new THREE.BoxGeometry(0.3,0.05,0.3);
  const bodyMat=new THREE.MeshLambertMaterial({color:0x333333});
  const body=new THREE.Mesh(bodyGeo,bodyMat);
  drone.add(body);
  
  // Arms & Motors
  const armGeo=new THREE.CylinderGeometry(0.01,0.01,0.4);
  const armMat=new THREE.MeshLambertMaterial({color:0x666666});
  const motorGeo=new THREE.CylinderGeometry(0.03,0.03,0.02);
  const motorMat=new THREE.MeshLambertMaterial({color:0x444444});
  
  const positions=[
    {x:0.2,z:0.2,rot:Math.PI/4},   // FL
    {x:-0.2,z:0.2,rot:-Math.PI/4}, // FR
    {x:0.2,z:-0.2,rot:-Math.PI/4}, // RL
    {x:-0.2,z:-0.2,rot:Math.PI/4}  // RR
  ];
  
  positions.forEach((pos,i)=>{
    // Arm
    const arm=new THREE.Mesh(armGeo,armMat);
    arm.position.set(pos.x*0.7,0,pos.z*0.7);
    arm.rotation.z=pos.rot;
    drone.add(arm);
    
    // Motor
    const motor=new THREE.Mesh(motorGeo,motorMat);
    motor.position.set(pos.x,0.035,pos.z);
    drone.add(motor);
    
    // Propeller
    const propGroup=new THREE.Group();
    const propGeo=new THREE.BoxGeometry(0.15,0.002,0.01);
    const propMat=new THREE.MeshLambertMaterial({
      color:i%2===0?0xff4444:0x44ff44,
      transparent:true,
      opacity:0.7
    });
    const prop1=new THREE.Mesh(propGeo,propMat);
    const prop2=new THREE.Mesh(propGeo,propMat);
    prop2.rotation.y=Math.PI/2;
    propGroup.add(prop1);
    propGroup.add(prop2);
    propGroup.position.set(pos.x,0.05,pos.z);
    propGroup.userData={speed:0,dir:i%2===0?1:-1};
    drone.add(propGroup);
    propellers.push(propGroup);
  });
  
  scene.add(drone);
}

function animate3D(){
  requestAnimationFrame(animate3D);
  
  // Update drone attitude (smooth interpolation)
  if(drone){
    const lerp=0.1;
    drone.rotation.x+=(THREE.MathUtils.degToRad(droneData.pitch)-drone.rotation.x)*lerp;
    drone.rotation.z+=(THREE.MathUtils.degToRad(-droneData.roll)-drone.rotation.z)*lerp;
    drone.rotation.y+=(THREE.MathUtils.degToRad(droneData.yaw)-drone.rotation.y)*lerp;
    
    // Propeller rotation based on motor values
    propellers.forEach((prop,i)=>{
      const motors=[droneData.motors.fl,droneData.motors.fr,droneData.motors.rl,droneData.motors.rr];
      const speed=(motors[i]-1000)/1000*0.5;
      prop.rotation.y+=speed*prop.userData.dir;
    });
    
    // Camera orbit
    const time=Date.now()*0.0005;
    camera.position.x=Math.cos(time)*3;
    camera.position.z=Math.sin(time)*3;
    camera.lookAt(drone.position);
  }
  
  renderer.render(scene,camera);
}

// WebSocket connection
function connectWS(){
  ws=new WebSocket('ws://'+window.location.host+'/ws');
  ws.onopen=()=>{$('#ws').classList.add('ok');};
  ws.onclose=()=>{$('#ws').classList.remove('ok');setTimeout(connectWS,2000);};
  ws.onmessage=e=>{
    try{
      const d=JSON.parse(e.data);
      updateUI(d);
      // Update 3D model data
      if(d.attitude){
        droneData.roll=d.attitude.roll;
        droneData.pitch=d.attitude.pitch;
        droneData.yaw=d.attitude.yaw;
      }
      if(d.motors){
        droneData.motors=d.motors;
      }
      if(d.input && d.input.throttle!==undefined){
        droneData.throttle=d.input.throttle;
      }
    }catch(err){console.error(err);}
  };
}

function updateUI(d){
  if(d.attitude){
    $('#roll').textContent=d.attitude.roll.toFixed(1)+'°';
    $('#pitch').textContent=d.attitude.pitch.toFixed(1)+'°';
    $('#yaw').textContent=d.attitude.yaw.toFixed(1)+'°';
  }
  if(d.battery!==undefined) $('#battery').textContent=d.battery.toFixed(2)+'V';
  if(d.mode!==undefined) $('#mode').textContent=d.mode;
  if(d.armed!==undefined) $('#arm').textContent=d.armed?'ARMED':'DISARMED';
  if(d.motors){
    updateMotor('fl',d.motors.fl);
    updateMotor('fr',d.motors.fr);
    updateMotor('rl',d.motors.rl);
    updateMotor('rr',d.motors.rr);
  }
  if(d.rcEnabled!==undefined){
    rcEnabled=d.rcEnabled;
    $('#rcpill').textContent=rcEnabled?'(RC ON)':'(RC OFF)';
    $('#rcpill').classList.toggle('on',rcEnabled);
  }
  
  // Update RC input display
  if(d.input){
    $('#rcRoll').textContent=d.input.roll.toFixed(2);
    $('#rcPitch').textContent=d.input.pitch.toFixed(2);
    $('#rcYaw').textContent=d.input.yaw.toFixed(2);
    $('#rcThr').textContent=(d.input.throttle*100).toFixed(0)+'%';
  }
}

function updateMotor(m,v){
  const pct=Math.max(0,Math.min(100,((v-1000)/1000)*100));
  $('#m'+m).style.width=pct+'%';
  $('#m'+m+'v').textContent=v;
}

function cmd(c){
  const p=JSON.stringify({command:c});
  if(ws && ws.readyState===1) ws.send(p);
}

function toggleRc(){
  rcEnabled=!rcEnabled;
  $('#rcpill').textContent=rcEnabled?'(RC ON)':'(RC OFF)';
  $('#rcpill').classList.toggle('on',rcEnabled);
  cmd(rcEnabled?'ENABLE_WEB_RC':'DISABLE_WEB_RC');
}

// Dual stick controls
function makeStick(el,knob){
  let active=false,cx,cy,kx=0,ky=0,maxR=50;
  
  function start(e){
    e.preventDefault();
    active=true;
    const r=el.getBoundingClientRect();
    cx=r.left+r.width/2;
    cy=r.top+r.height/2;
    move(e);
  }
  
  function move(e){
    if(!active)return;
    const t=e.touches?e.touches[0]:e;
    let dx=t.clientX-cx,dy=t.clientY-cy;
    const d=Math.sqrt(dx*dx+dy*dy);
    if(d>maxR){dx=dx/d*maxR;dy=dy/d*maxR;}
    kx=dx;ky=dy;
    knob.style.transform=`translate(${kx}px,${ky}px)`;
  }
  
  function end(){
    active=false;kx=ky=0;
    knob.style.transform='translate(0,0)';
  }
  
  el.addEventListener('mousedown',start);
  el.addEventListener('mousemove',move);
  el.addEventListener('mouseup',end);
  el.addEventListener('mouseleave',end);
  el.addEventListener('touchstart',start);
  el.addEventListener('touchmove',move);
  el.addEventListener('touchend',end);
  
  return {value:()=>({x:kx/maxR,y:-ky/maxR})};
}

// Initialize
window.addEventListener('DOMContentLoaded',()=>{
  init3D();
  connectWS();
  
  $('#rcpill').addEventListener('click',toggleRc);
  
  const L=makeStick($('#stickL'),$('#knobL'));
  const R=makeStick($('#stickR'),$('#knobR'));
  
  // Send RC data at 50Hz
  setInterval(()=>{
    if(!rcEnabled)return;
    const lv=L.value(),rv=R.value();
    rc.throttle=Math.max(0,Math.min(1,(lv.y+1)/2));
    rc.yaw=lv.x;
    rc.roll=rv.x;
    rc.pitch=rv.y;
    if(ws && ws.readyState===1){
      ws.send(JSON.stringify({rc}));
    }
  },20);
});

window.addEventListener('resize',()=>{
  camera.aspect=window.innerWidth/window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth,window.innerHeight);
});
</script>
</body></html>
)HTML";

#endif // DRONE_3D_HTML_H