const $=s=>document.querySelector(s);
let ws,rcEnabled=false,rc={throttle:0,yaw:0,roll:0,pitch:0};
let droneData={roll:0,pitch:0,yaw:0,throttle:0,motors:{fl:1000,fr:1000,rl:1000,rr:1000}};

let scene,camera,renderer,drone,propellers=[];

function isMobile() {
  return /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent) || 
         (window.innerWidth <= 768);
}

function init3D(){
  const container=$('#canvas3d');
  if (!container) {
    console.error('Canvas container not found!');
    return;
  }
  
  try {
    scene=new THREE.Scene();
    scene.background=new THREE.Color(0x0a0a0a);
    
    camera=new THREE.PerspectiveCamera(60,window.innerWidth/window.innerHeight,0.1,1000);
    
    if (isMobile()) {
      camera.position.set(2.5,2.5,2.5);
    } else {
      camera.position.set(4,4,4);
    }
    camera.lookAt(0,0,0);
    
    renderer=new THREE.WebGLRenderer({
      antialias: !isMobile(),
      alpha:true,
      powerPreference: "high-performance"
    });
    renderer.setSize(window.innerWidth,window.innerHeight);
    
    if (isMobile()) {
      renderer.setPixelRatio(1);
      renderer.shadowMap.enabled=false;
    } else {
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      renderer.shadowMap.enabled=true;
    }
    
    container.appendChild(renderer.domElement);
    
    const ambient=new THREE.AmbientLight(0x404040, isMobile() ? 1.2 : 1.0);
    scene.add(ambient);
    
    const directional=new THREE.DirectionalLight(0xffffff, isMobile() ? 1.0 : 0.8);
    directional.position.set(5,5,5);
    if (!isMobile()) {
      directional.castShadow=true;
    }
    scene.add(directional);
    
    createDrone();
    
    const grid=new THREE.GridHelper(10,20,0x4fc3f7,0x4fc3f7);
    grid.material.transparent=true;
    grid.material.opacity=0.4;
    scene.add(grid);
    
    const axes=new THREE.AxesHelper(1.5);
    scene.add(axes);
    
    animate3D();
    
    console.log('3D scene initialized successfully');
  } catch (error) {
    console.error('Failed to initialize 3D scene:', error);
  }
}

function createDrone(){
  drone=new THREE.Group();
  
  const scale = isMobile() ? 1.0 : 0.8;
  
  // === 메인 바디 (직사각형 - 90도 회전) ===
  const bodyGeo=new THREE.BoxGeometry(0.7*scale, 0.25*scale, 1.4*scale);
  const bodyMat=new THREE.MeshLambertMaterial({color:0x666666});
  const body=new THREE.Mesh(bodyGeo, bodyMat);
  drone.add(body);
  
  // 바디 상단 디테일
  const topPanelGeo=new THREE.BoxGeometry(0.5*scale, 0.05*scale, 1.2*scale);
  const topPanelMat=new THREE.MeshLambertMaterial({color:0x888888});
  const topPanel=new THREE.Mesh(topPanelGeo, topPanelMat);
  topPanel.position.y = 0.15*scale;
  drone.add(topPanel);
  
  // === 4개의 암 구조 (45도 대각선) ===
  const armConfigs = [
    {x: -0.7, z: 0.7, name: 'FL', color: 0xff3333},   // Front Left - 밝은 빨강
    {x: 0.7, z: 0.7, name: 'FR', color: 0xff3333},    // Front Right - 밝은 빨강
    {x: -0.7, z: -0.7, name: 'RL', color: 0x33ff33},  // Rear Left - 초록
    {x: 0.7, z: -0.7, name: 'RR', color: 0x33ff33}    // Rear Right - 초록
  ];
  
  armConfigs.forEach((config, i) => {
    // 암 마운트 (바디에서 나오는 부분)
    const mountGeo = new THREE.BoxGeometry(0.15*scale, 0.2*scale, 0.15*scale);
    const mountMat = new THREE.MeshLambertMaterial({color:0x555555});
    const mount = new THREE.Mesh(mountGeo, mountMat);
    mount.position.set(config.x * 0.3*scale, 0, config.z * 0.5*scale);
    drone.add(mount);
    
    // 모터 마운트 - 바디에서 더 멀리 배치
    const motorMountGeo = new THREE.CylinderGeometry(0.12*scale, 0.1*scale, 0.15*scale);
    const motorMountMat = new THREE.MeshLambertMaterial({color:0x444444});
    const motorMount = new THREE.Mesh(motorMountGeo, motorMountMat);
    motorMount.position.set(config.x * 0.8*scale, 0, config.z * 1.0*scale);
    drone.add(motorMount);
    
    // 모터 상단
    const motorTopGeo = new THREE.CylinderGeometry(0.1*scale, 0.1*scale, 0.05*scale);
    const motorTopMat = new THREE.MeshLambertMaterial({color:0x333333});
    const motorTop = new THREE.Mesh(motorTopGeo, motorTopMat);
    motorTop.position.set(config.x * 0.8*scale, 0.1*scale, config.z * 1.0*scale);
    drone.add(motorTop);
    
    // 프로펠러 그룹
    const propGroup = new THREE.Group();
    
    // 2날 프로펠러
    const blade1Geo = new THREE.BoxGeometry(0.5*scale, 0.01*scale, 0.05*scale);
    const blade2Geo = new THREE.BoxGeometry(0.05*scale, 0.01*scale, 0.5*scale);
    const bladeMat = new THREE.MeshLambertMaterial({
      color: config.color,
      transparent: true,
      opacity: 0.7
    });
    
    const blade1 = new THREE.Mesh(blade1Geo, bladeMat);
    const blade2 = new THREE.Mesh(blade2Geo, bladeMat);
    
    propGroup.add(blade1);
    propGroup.add(blade2);
    propGroup.position.set(config.x * 0.8*scale, 0.15*scale, config.z * 1.0*scale);
    propGroup.userData = {speed: 0, dir: i % 2 === 0 ? 1 : -1};
    
    drone.add(propGroup);
    propellers.push(propGroup);
    
    // 랜딩 기어
    const legGeo = new THREE.CylinderGeometry(0.02*scale, 0.02*scale, 0.4*scale);
    const legMat = new THREE.MeshLambertMaterial({color:0x555555});
    const leg = new THREE.Mesh(legGeo, legMat);
    leg.position.set(config.x * 0.6*scale, -0.2*scale, config.z * 0.8*scale);
    drone.add(leg);
    
    // 랜딩 패드
    const padGeo = new THREE.SphereGeometry(0.04*scale);
    const padMat = new THREE.MeshLambertMaterial({color:0x333333});
    const pad = new THREE.Mesh(padGeo, padMat);
    pad.position.set(config.x * 0.6*scale, -0.4*scale, config.z * 0.8*scale);
    drone.add(pad);
  });
  
  // === 중앙 안테나/GPS ===
  const antennaGeo = new THREE.CylinderGeometry(0.015*scale, 0.015*scale, 0.2*scale);
  const antennaMat = new THREE.MeshLambertMaterial({color:0xFFFF66});
  const antenna = new THREE.Mesh(antennaGeo, antennaMat);
  antenna.position.set(0, 0.25*scale, 0);
  drone.add(antenna);
  
  // === Top 안테나/GPS ===
  const antennaTopGeo = new THREE.SphereGeometry(0.03*scale);
  const antennaTopMat = new THREE.MeshLambertMaterial({color:0x333333});
  const antennaTop = new THREE.Mesh(antennaTopGeo, antennaTopMat);
  antennaTop.position.set(0, 0.35*scale, 0);
  drone.add(antennaTop);
  
  // === 전면/후면 표시 LED ===
  const ledGeo = new THREE.SphereGeometry(0.04*scale);
  
  // 전면 LED (빨강) - Z축 양수 방향
  const frontLED = new THREE.Mesh(ledGeo, new THREE.MeshBasicMaterial({color:0xff0000}));
  frontLED.position.set(0, 0.05*scale, 0.8*scale);
  drone.add(frontLED);
  
  // 후면 LED (초록) - Z축 음수 방향
  const rearLED = new THREE.Mesh(ledGeo, new THREE.MeshBasicMaterial({color:0x00ff00}));
  rearLED.position.set(0, 0.05*scale, -0.8*scale);
  drone.add(rearLED);
  
  scene.add(drone);
  console.log('Detailed drone model created');
}

function animate3D(){
  requestAnimationFrame(animate3D);
  
  if(drone && camera && renderer){
    const lerp=0.1;
    drone.rotation.x+=(THREE.MathUtils.degToRad(droneData.pitch)-drone.rotation.x)*lerp;
    drone.rotation.z+=(THREE.MathUtils.degToRad(-droneData.roll)-drone.rotation.z)*lerp;
    drone.rotation.y+=(THREE.MathUtils.degToRad(droneData.yaw)-drone.rotation.y)*lerp;
    
    propellers.forEach((prop,i)=>{
      if (prop && prop.userData) {
        const motors=[droneData.motors.fl,droneData.motors.fr,droneData.motors.rl,droneData.motors.rr];
        const speed=(motors[i]-1000)/1000*0.5;
        prop.rotation.z+=speed*prop.userData.dir;
        
        // 모터 속도에 따라 투명도 조정
        const opacity = 0.2 + (speed * 0.3);
        if (prop.children[0]) {
          prop.children[0].material.opacity = Math.min(opacity, 0.6);
        }
      }
    });
    
    const time=Date.now()*(isMobile() ? 0.0002 : 0.0003);
    const radius = isMobile() ? 3 : 4;
    camera.position.x=Math.cos(time)*radius;
    camera.position.z=Math.sin(time)*radius;
    camera.position.y=isMobile() ? 2.5 : 3;
    camera.lookAt(drone.position);
    
    renderer.render(scene,camera);
  }
}

function connectWS(){
  ws=new WebSocket('ws://'+window.location.host+'/ws');
  ws.onopen=()=>{$('#ws').classList.add('ok');};
  ws.onclose=()=>{$('#ws').classList.remove('ok');setTimeout(connectWS,2000);};
  ws.onmessage=e=>{
    try{
      const d=JSON.parse(e.data);
      updateUI(d);
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
  
  if(d.input){
    $('#rcRoll').textContent=d.input.roll.toFixed(2);
    $('#rcPitch').textContent=d.input.pitch.toFixed(2);
    $('#rcYaw').textContent=d.input.yaw.toFixed(2);
    $('#rcThr').textContent=(d.input.throttle*100).toFixed(0)+'%';
  }
}

function updateMotor(m,v){
  const pct=Math.max(0,Math.min(100,((v-1000)/1000)*100));
  const el = $('#m'+m);
  if (el) {
    el.style.width=pct+'%';
  }
  const elv = $('#m'+m+'v');
  if (elv) {
    elv.textContent=v;
  }
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

function makeStick(el,knob){
  if (!el || !knob) {
    console.error('Stick element not found');
    return {value:()=>({x:0,y:0})};
  }
  
  let active=false,cx=0,cy=0,kx=0,ky=0,maxR=60;
  
  function updateBounds(){
    const rect=el.getBoundingClientRect();
    cx=rect.left+rect.width/2;
    cy=rect.top+rect.height/2;
    maxR=Math.min(rect.width,rect.height)*0.35;
  }
  
  function getEventPos(e){
    if (e.touches && e.touches.length > 0) {
      return {x: e.touches[0].clientX, y: e.touches[0].clientY};
    }
    return {x: e.clientX, y: e.clientY};
  }
  
  function start(e){
    e.preventDefault();
    e.stopPropagation();
    active=true;
    updateBounds();
    
    const pos = getEventPos(e);
    move_internal(pos.x, pos.y);
  }
  
  function move_internal(clientX, clientY) {
    if (!active) return;
    
    let dx = clientX - cx;
    let dy = clientY - cy;
    const distance = Math.sqrt(dx*dx + dy*dy);
    
    if (distance > maxR) {
      dx = (dx / distance) * maxR;
      dy = (dy / distance) * maxR;
    }
    
    kx = dx;
    ky = dy;
    
    if (knob) {
      knob.style.transform = 'translate(' + kx + 'px,' + ky + 'px)';
    }
  }
  
  function move(e){
    if (!active) return;
    e.preventDefault();
    e.stopPropagation();
    
    const pos = getEventPos(e);
    move_internal(pos.x, pos.y);
  }
  
  function end(e){
    if (e) {
      e.preventDefault();
      e.stopPropagation();
    }
    active=false;
    kx=0;
    ky=0;
    
    if (knob) {
      knob.style.transform='translate(0px,0px)';
    }
  }
  
  el.addEventListener('touchstart', start, {passive:false});
  el.addEventListener('mousedown', start, {passive:false});
  
  document.addEventListener('touchmove', move, {passive:false});
  document.addEventListener('mousemove', move, {passive:false});
  
  document.addEventListener('touchend', end, {passive:false});
  document.addEventListener('touchcancel', end, {passive:false});
  document.addEventListener('mouseup', end, {passive:false});
  
  if ('PointerEvent' in window) {
    el.addEventListener('pointerdown', start, {passive:false});
    document.addEventListener('pointermove', move, {passive:false});
    document.addEventListener('pointerup', end, {passive:false});
    document.addEventListener('pointercancel', end, {passive:false});
  }
  
  updateBounds();
  
  window.addEventListener('resize', updateBounds);
  window.addEventListener('orientationchange', ()=>{
    setTimeout(updateBounds, 100);
  });
  
  return {
    value: ()=> ({
      x: maxR > 0 ? (kx / maxR) : 0,
      y: maxR > 0 ? (-ky / maxR) : 0
    })
  };
}

window.addEventListener('DOMContentLoaded',()=>{
  setTimeout(init3D, 100);
  connectWS();
  
  const rcPill = $('#rcpill');
  if (rcPill) {
    rcPill.addEventListener('click', toggleRc);
  }
  
  const stickL = $('#stickL');
  const knobL = $('#knobL');
  const stickR = $('#stickR');
  const knobR = $('#knobR');
  
  if (stickL && knobL && stickR && knobR) {
    const L = makeStick(stickL, knobL);
    const R = makeStick(stickR, knobR);
    
    setInterval(()=>{
      if(!rcEnabled) return;
      
      const lv = L.value();
      const rv = R.value();
      
      rc.throttle = Math.max(0, Math.min(1, (lv.y + 1) / 2));
      rc.yaw = Math.max(-1, Math.min(1, lv.x));
      rc.roll = Math.max(-1, Math.min(1, rv.x));
      rc.pitch = Math.max(-1, Math.min(1, -rv.y));
      
      if(ws && ws.readyState === 1){
        ws.send(JSON.stringify({rc}));
      }
    }, 20);
  }
});

window.addEventListener('resize',()=>{
  if(camera && renderer){
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  }
});

window.addEventListener('orientationchange', ()=>{
  setTimeout(()=>{
    if(camera && renderer){
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }
  }, 100);
});