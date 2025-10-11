const $=s=>document.querySelector(s);
let ws,rcEnabled=false,rc={throttle:0,yaw:0,roll:0,pitch:0};
let droneData={roll:0,pitch:0,yaw:0,throttle:0,motors:{fl:1000,fr:1000,rl:1000,rr:1000}};

let scene,camera,renderer,drone,propellers=[];

// 모바일 디바이스 감지
function isMobile() {
  return /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent) || 
         (window.innerWidth <= 768);
}

/* ---------- 프롭 블러 텍스처 ---------- */
function makeBlurTexture() {
  const c=document.createElement('canvas');
  const sz=256; c.width=c.height=sz;
  const ctx=c.getContext('2d');
  const g=ctx.createRadialGradient(sz/2,sz/2,10, sz/2,sz/2,sz/2);
  g.addColorStop(0,'rgba(255,255,255,0.15)');
  g.addColorStop(0.35,'rgba(255,255,255,0.06)');
  g.addColorStop(1,'rgba(255,255,255,0.0)');
  ctx.fillStyle=g; ctx.beginPath(); ctx.arc(sz/2,sz/2,sz/2,0,Math.PI*2); ctx.fill();
  const tex=new THREE.CanvasTexture(c);
  tex.wrapS=tex.wrapT=THREE.ClampToEdgeWrapping;
  tex.anisotropy=4;
  return tex;
}

/* ---------- 소재 팔레트 ---------- */
function createMavicStyleMaterials() {
  return {
    body:  new THREE.MeshStandardMaterial({color:0x5c5f66, roughness:0.6, metalness:0.1}),
    top:   new THREE.MeshStandardMaterial({color:0x90949b, roughness:0.45, metalness:0.2}),
    arm:   new THREE.MeshStandardMaterial({color:0x2c2f33, roughness:0.5, metalness:0.15}),
    motor: new THREE.MeshStandardMaterial({color:0x1f2329, roughness:0.35, metalness:0.4}),
    cam:   new THREE.MeshStandardMaterial({color:0x141414, roughness:0.4, metalness:0.25}),
    lens:  new THREE.MeshStandardMaterial({color:0xa7abb1, roughness:0.2, metalness:0.8}),
    accent:new THREE.MeshStandardMaterial({color:0x3aa0ff, roughness:0.6, metalness:0.1}),
    prop:  new THREE.MeshBasicMaterial({transparent:true, opacity:0.85, depthWrite:false, map: makeBlurTexture()})
  };
}

/* ---------- 3D 초기화 (밝은 배경/조명) ---------- */
function init3D(){
  const container=$('#canvas3d');
  if(!container){ console.error('Canvas container not found!'); return; }

  scene=new THREE.Scene();
  scene.background=new THREE.Color(0xf5f7fa); // 밝은 그레이 배경

  camera=new THREE.PerspectiveCamera(60,window.innerWidth/window.innerHeight,0.1,1000);
  if(isMobile()) camera.position.set(2.6,2.4,2.6); else camera.position.set(4.2,3.2,4.0);
  camera.lookAt(0,0,0);

  renderer=new THREE.WebGLRenderer({antialias:!isMobile(), alpha:true, powerPreference:"high-performance"});
  renderer.setSize(window.innerWidth,window.innerHeight);
  renderer.setPixelRatio(isMobile()?1:Math.min(window.devicePixelRatio,2));
  renderer.shadowMap.enabled=!isMobile();
  container.appendChild(renderer.domElement);

  // 스튜디오 톤 조명
  const hemi=new THREE.HemisphereLight(0xffffff,0xdde1e7,isMobile()?0.9:0.8);
  scene.add(hemi);
  const key=new THREE.DirectionalLight(0xffffff,isMobile()?0.9:0.8);
  key.position.set(5,6,4); key.castShadow=!isMobile(); scene.add(key);
  const rim=new THREE.DirectionalLight(0xffffff,0.25);
  rim.position.set(-4,3,-5); scene.add(rim);

  createDrone();

  // 은은한 바닥/축
  const grid=new THREE.GridHelper(10,20,0x9fbad0,0x9fbad0);
  grid.material.transparent=true; grid.material.opacity=0.18; scene.add(grid);
  const axes=new THREE.AxesHelper(1.5); axes.material.transparent=true; axes.material.opacity=0.3; scene.add(axes);

  animate3D();
}

/* ---------- 드론 모델 (이미지 스타일) ---------- */
function createDrone(){
  drone=new THREE.Group();
  const scale=isMobile()?1.15:1.0;
  const M=createMavicStyleMaterials();

  // 본체
  const bodyGeo=new THREE.BoxGeometry(0.95*scale,0.18*scale,0.62*scale);
  const body=new THREE.Mesh(bodyGeo,M.body); body.castShadow=true; body.receiveShadow=true;
  body.position.y=0.12*scale; drone.add(body);

  // 상판
  const topGeo=new THREE.BoxGeometry(1.00*scale,0.06*scale,0.70*scale);
  const top=new THREE.Mesh(topGeo,M.top); top.position.y=0.21*scale; top.castShadow=true; drone.add(top);

  // 전면부
  const nose=new THREE.Mesh(new THREE.BoxGeometry(0.32*scale,0.10*scale,0.18*scale),M.cam);
  nose.position.set(0,0.12*scale,0.37*scale); drone.add(nose);

  // 짐벌 베이스
  const gimbalBase=new THREE.Mesh(new THREE.BoxGeometry(0.26*scale,0.10*scale,0.10*scale),M.arm);
  gimbalBase.position.set(0,0.02*scale,0.42*scale); drone.add(gimbalBase);

  // 카메라 + 렌즈
  const camBox=new THREE.Mesh(new THREE.BoxGeometry(0.22*scale,0.16*scale,0.14*scale),M.cam);
  camBox.position.set(0,-0.02*scale,0.50*scale); drone.add(camBox);
  const lens=new THREE.Mesh(new THREE.CylinderGeometry(0.06*scale,0.06*scale,0.03*scale,32),M.lens);
  lens.rotation.x=Math.PI/2; lens.position.set(0,-0.02*scale,0.57*scale); drone.add(lens);

  // 팔 + 모터 + 프롭(블러 디스크)
  const armGeo=new THREE.BoxGeometry(0.95*scale,0.06*scale,0.06*scale);
  const corners=[
    {sx: 1, sz: 1, rot:  Math.PI/4},
    {sx:-1, sz: 1, rot: -Math.PI/4},
    {sx: 1, sz:-1, rot: -Math.PI/4},
    {sx:-1, sz:-1, rot:  Math.PI/4}
  ];

  propellers=[];
  corners.forEach((c,i)=>{
    const arm=new THREE.Mesh(armGeo,M.arm);
    arm.position.set(c.sx*0.38*scale, 0.08*scale, c.sz*0.38*scale);
    arm.rotation.y=c.rot;
    arm.castShadow=true; drone.add(arm);

    const pod=new THREE.Mesh(new THREE.CylinderGeometry(0.09*scale,0.09*scale,0.12*scale,24), M.motor);
    pod.position.set(c.sx*0.68*scale, 0.12*scale, c.sz*0.68*scale);
    pod.rotation.x=Math.PI/2; pod.castShadow=true; drone.add(pod);

    const prop=new THREE.Mesh(new THREE.CircleGeometry(0.35*scale, 48), M.prop);
    prop.position.set(c.sx*0.68*scale, 0.13*scale, c.sz*0.68*scale);
    prop.rotation.x=-Math.PI/2;
    prop.userData={speed:0, dir:(i%2===0?1:-1)};
    propellers.push(prop);
    drone.add(prop);
  });

  scene.add(drone);
  console.log('Drone model created');
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
        const speed=(motors[i]-1000)/1000*0.7;
        prop.rotation.z+=speed*prop.userData.dir*0.6; // 수평 디스크 회전
      }
    });

    // 카메라 오토 오빗(느리게)
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
  if(d.battery!==undefined){
    $('#battery').textContent=d.battery.toFixed(2)+'V';
  }
  if(d.armed!==undefined){
    $('#armed').textContent=d.armed?'ON':'OFF';
    $('#armed').classList.toggle('on',d.armed);
  }
  if(d.mode){
    $('#mode').textContent=d.mode;
  }
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

// 버튼 이벤트
window.addEventListener('DOMContentLoaded',()=>{
  init3D();
  connectWS();
  
  $('#btnCalibrate')?.addEventListener('click', ()=>{
    fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'CALIBRATE'})});
  });
  $('#btnResetPID')?.addEventListener('click', ()=>{
    fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'RESET_PID'})});
  });
  $('#btnEStop')?.addEventListener('click', ()=>{
    fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:'EMERGENCY_STOP'})});
  });
  $('#btnRcToggle')?.addEventListener('click', ()=>{
    const command= rcEnabled ? 'DISABLE_WEB_RC' : 'ENABLE_WEB_RC';
    fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command})});
  });

  // 조이스틱
  function makeStick(el, knob){
    let rect=el.getBoundingClientRect();
    let cx=rect.left+rect.width/2, cy=rect.top+rect.height/2;
    let active=false, maxR=Math.min(rect.width,rect.height)/2 - 8;
    let kx=0, ky=0;

    function updateBounds(){
      rect=el.getBoundingClientRect();
      cx=rect.left+rect.width/2; cy=rect.top+rect.height/2;
      maxR=Math.min(rect.width,rect.height)/2 - 8;
    }
    function getEventPos(e){
      if (e.touches && e.touches.length > 0) {
        return {x: e.touches[0].clientX, y: e.touches[0].clientY};
      }
      return {x: e.clientX, y: e.clientY};
    }
    function start(e){
      e.preventDefault(); e.stopPropagation();
      active=true; updateBounds();
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
      kx = dx; ky = dy;
      if (knob) knob.style.transform = 'translate(' + kx + 'px,' + ky + 'px)';
    }
    function move(e){
      if (!active) return;
      e.preventDefault(); e.stopPropagation();
      const pos = getEventPos(e);
      move_internal(pos.x, pos.y);
    }
    function end(e){
      if (e) { e.preventDefault(); e.stopPropagation(); }
      active=false; kx=0; ky=0;
      if (knob) knob.style.transform='translate(0px,0px)';
    }
    el.addEventListener('mousedown',start);
    window.addEventListener('mousemove',move);
    window.addEventListener('mouseup',end);

    el.addEventListener('touchstart',start,{passive:false});
    window.addEventListener('touchmove',move,{passive:false});
    window.addEventListener('touchend',end,{passive:false});
    window.addEventListener('touchcancel',end,{passive:false});

    return {
      value:()=>({
        x: Math.max(-1, Math.min(1, kx / maxR)),
        y: Math.max(-1, Math.min(1, ky / maxR))
      })
    };
  }

  const stickL = $('#stickL');
  const knobL = $('#knobL');
  const stickR = $('#stickR');
  const knobR = $('#knobR');
  
  if (stickL && knobL && stickR && knobR) {
    const L = makeStick(stickL, knobL);
    const R = makeStick(stickR, knobR);
    
    // RC 데이터 전송 (50Hz)
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
  } else {
    console.error('Joystick elements not found');
  }
});

// 창 크기 변경
window.addEventListener('resize',()=>{
  if(camera && renderer){
    camera.aspect=window.innerWidth/window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  }
});

// 방향전환 처리
window.addEventListener('orientationchange', ()=>{
  setTimeout(()=>{
    if(camera && renderer){
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }
  }, 100);
});
