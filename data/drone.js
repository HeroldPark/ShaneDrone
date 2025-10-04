const $=s=>document.querySelector(s);
let ws,rcEnabled=false,rc={throttle:0,yaw:0,roll:0,pitch:0};
let droneData={roll:0,pitch:0,yaw:0,throttle:0,motors:{fl:1000,fr:1000,rl:1000,rr:1000}};

let scene,camera,renderer,drone,propellers=[];

// 모바일 디바이스 감지
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
    
    // 모바일에서 더 가까이 배치
    if (isMobile()) {
      camera.position.set(2.5,2.5,2.5);
    } else {
      camera.position.set(4,4,4);
    }
    camera.lookAt(0,0,0);
    
    renderer=new THREE.WebGLRenderer({
      antialias: !isMobile(), // 모바일에서는 안티앨리어싱 비활성화
      alpha:true,
      powerPreference: "high-performance"
    });
    renderer.setSize(window.innerWidth,window.innerHeight);
    
    // 모바일 최적화
    if (isMobile()) {
      renderer.setPixelRatio(1); // 모바일에서는 픽셀비율 1로 고정
      renderer.shadowMap.enabled=false; // 그림자 비활성화
    } else {
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      renderer.shadowMap.enabled=true;
    }
    
    container.appendChild(renderer.domElement);
    
    // 조명 설정
    const ambient=new THREE.AmbientLight(0x404040, isMobile() ? 1.2 : 1.0);
    scene.add(ambient);
    
    const directional=new THREE.DirectionalLight(0xffffff, isMobile() ? 1.0 : 0.8);
    directional.position.set(5,5,5);
    if (!isMobile()) {
      directional.castShadow=true;
    }
    scene.add(directional);
    
    createDrone();
    
    // 그리드와 축
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
  
  // 모바일에서 더 큰 크기로 설정
  const scale = isMobile() ? 1.2 : 1.0;
  
  const bodyGeo=new THREE.BoxGeometry(0.8*scale,0.15*scale,0.8*scale);
  const bodyMat=new THREE.MeshLambertMaterial({color:0x777777});
  const body=new THREE.Mesh(bodyGeo,bodyMat);
  drone.add(body);
  
  const armGeo=new THREE.CylinderGeometry(0.03*scale,0.03*scale,1.0*scale);
  const armMat=new THREE.MeshLambertMaterial({color:0xaaaaaa});
  const motorGeo=new THREE.CylinderGeometry(0.08*scale,0.08*scale,0.06*scale);
  const motorMat=new THREE.MeshLambertMaterial({color:0x888888});
  
  const positions=[
    {x:0.5*scale,z:0.5*scale,rot:Math.PI/4},
    {x:-0.5*scale,z:0.5*scale,rot:-Math.PI/4},
    {x:0.5*scale,z:-0.5*scale,rot:-Math.PI/4},
    {x:-0.5*scale,z:-0.5*scale,rot:Math.PI/4}
  ];
  
  positions.forEach((pos,i)=>{
    const arm=new THREE.Mesh(armGeo,armMat);
    arm.position.set(pos.x*0.7,0,pos.z*0.7);
    arm.rotation.z=pos.rot;
    drone.add(arm);
    
    const motor=new THREE.Mesh(motorGeo,motorMat);
    motor.position.set(pos.x,0.08*scale,pos.z);
    drone.add(motor);
    
    const propGroup=new THREE.Group();
    const propGeo=new THREE.BoxGeometry(0.4*scale,0.008*scale,0.03*scale);
    const propMat=new THREE.MeshLambertMaterial({
      color:i%2===0?0xff8888:0x88ff88,
      transparent:true,
      opacity:0.9
    });
    const prop1=new THREE.Mesh(propGeo,propMat);
    const prop2=new THREE.Mesh(propGeo,propMat);
    prop2.rotation.y=Math.PI/2;
    propGroup.add(prop1);
    propGroup.add(prop2);
    propGroup.position.set(pos.x,0.12*scale,pos.z);
    propGroup.userData={speed:0,dir:i%2===0?1:-1};
    drone.add(propGroup);
    propellers.push(propGroup);
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
        const speed=(motors[i]-1000)/1000*0.5;
        prop.rotation.y+=speed*prop.userData.dir;
      }
    });
    
    // 카메라 회전 (모바일에서는 더 느리게)
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

// 개선된 조이스틱 - 터치 이벤트 완전 재작성
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
    
    console.log('Stick start:', el.id);
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
    
    console.log('Stick end:', el.id);
  }
  
  // 이벤트 리스너 등록
  el.addEventListener('touchstart', start, {passive:false});
  el.addEventListener('mousedown', start, {passive:false});
  
  document.addEventListener('touchmove', move, {passive:false});
  document.addEventListener('mousemove', move, {passive:false});
  
  document.addEventListener('touchend', end, {passive:false});
  document.addEventListener('touchcancel', end, {passive:false});
  document.addEventListener('mouseup', end, {passive:false});
  
  // 포인터 이벤트 지원
  if ('PointerEvent' in window) {
    el.addEventListener('pointerdown', start, {passive:false});
    document.addEventListener('pointermove', move, {passive:false});
    document.addEventListener('pointerup', end, {passive:false});
    document.addEventListener('pointercancel', end, {passive:false});
  }
  
  // 초기 bounds 설정
  updateBounds();
  
  // 리사이즈 및 방향전환 처리
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

// DOM 로드 완료 후 초기화
window.addEventListener('DOMContentLoaded',()=>{
  console.log('DOM loaded, initializing...');
  
  // 3D 초기화
  setTimeout(init3D, 100); // 약간의 딜레이로 안정성 확보
  
  // WebSocket 연결
  connectWS();
  
  // RC 토글 버튼
  const rcPill = $('#rcpill');
  if (rcPill) {
    rcPill.addEventListener('click', toggleRc);
  }
  
  // 조이스틱 초기화
  const stickL = $('#stickL');
  const knobL = $('#knobL');
  const stickR = $('#stickR');
  const knobR = $('#knobR');
  
  if (stickL && knobL && stickR && knobR) {
    const L = makeStick(stickL, knobL);
    const R = makeStick(stickR, knobR);
    
    console.log('Joysticks initialized');
    
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

// 윈도우 리사이즈 처리
window.addEventListener('resize',()=>{
  if(camera && renderer){
    camera.aspect = window.innerWidth / window.innerHeight;
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