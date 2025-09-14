// drone-controller.js - 3D 드론 컨트롤러 및 UI 관리
// Shane Drone Web Interface v0.4

// === 전역 변수 ===
const $ = s => document.querySelector(s);
let ws, tRetry, rcEnabled = false;
let rc = { throttle: 0, yaw: 0, roll: 0, pitch: 0 };

// Three.js 3D 드론 시각화 변수
let scene, camera, renderer, drone, propellers = [];
let droneGroup, currentAttitude = { roll: 0, pitch: 0, yaw: 0 };
let targetAttitude = { roll: 0, pitch: 0, yaw: 0 };
let motorOutputs = { FL: 0, FR: 0, RL: 0, RR: 0 };
let autoRotate = false;
let cameraDistance = 4;

// === WebSocket 연결 관리 ===
function connectWS() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => {
        $('#ws').classList.add('ok');
        if (tRetry) clearTimeout(tRetry);
    };
    ws.onclose = () => {
        $('#ws').classList.remove('ok');
        tRetry = setTimeout(connectWS, 1200);
    };
    ws.onmessage = e => {
        try {
            update(JSON.parse(e.data || '{}'));
        } catch (err) {
            console.warn('WebSocket message parse error:', err);
        }
    };
}

// 폴링 백업(WS 끊기면)
setInterval(() => {
    if (!ws || ws.readyState !== 1) {
        fetch('/telemetry')
            .then(r => r.json())
            .then(update)
            .catch(() => {});
    }
}, 900);

// === UI 업데이트 함수들 ===
function pct(v) {
    return Math.max(0, Math.min(100, ((v - 1000) / 1000) * 100));
}

function bar(id, val, label) {
    $(id).style.width = pct(val) + '%';
    $(label).textContent = val;
}

function setText(sel, text) {
    const el = document.querySelector(sel);
    if (el) el.textContent = text;
}

function update(data) {
    // 센서 데이터 업데이트
    if (data.attitude) {
        setText('#roll', data.attitude.roll.toFixed(1) + '°');
        setText('#pitch', data.attitude.pitch.toFixed(1) + '°');
        setText('#yaw', data.attitude.yaw.toFixed(1) + '°');
        
        // 3D 드론 모델 업데이트
        if (droneGroup) {
            targetAttitude.roll = data.attitude.roll;
            targetAttitude.pitch = data.attitude.pitch;
            targetAttitude.yaw = data.attitude.yaw;
        }
    }

    // 시스템 상태 업데이트
    if (typeof data.battery !== 'undefined') setText('#battery', data.battery.toFixed(2) + 'V');
    if (typeof data.mode !== 'undefined') setText('#mode', data.mode);
    if (typeof data.armed !== 'undefined') setText('#arm', data.armed ? 'ARMED' : 'DISARMED');
    
    // 모터 출력 업데이트
    if (data.motors) {
        bar('#mfl', data.motors.fl, '#mflv');
        bar('#mfr', data.motors.fr, '#mfrv');
        bar('#mrl', data.motors.rl, '#mrlv');
        bar('#mrr', data.motors.rr, '#mrrv');
        
        // 모터 출력을 3D 모델에 반영
        if (propellers.length > 0) {
            motorOutputs.FL = data.motors.fl;
            motorOutputs.FR = data.motors.fr;
            motorOutputs.RL = data.motors.rl;
            motorOutputs.RR = data.motors.rr;
        }
    }
    
    // RC 상태 업데이트
    if (typeof data.rcEnabled !== 'undefined') {
        rcEnabled = !!data.rcEnabled;
        paintRcPill();
    }

    // RC 입력 표시
    if (data.input) {
        if (typeof data.input.roll === 'number') setText('#rcRoll', data.input.roll.toFixed(2));
        if (typeof data.input.pitch === 'number') setText('#rcPitch', data.input.pitch.toFixed(2));
        if (typeof data.input.yaw === 'number') setText('#rcYaw', data.input.yaw.toFixed(2));
        if (typeof data.input.throttle === 'number') setText('#rcThr', (data.input.throttle * 100).toFixed(0) + '%');
    }
}

// === 명령 전송 ===
function cmd(command) {
    const payload = JSON.stringify({ command });
    if (ws && ws.readyState === 1) {
        ws.send(payload);
    } else {
        fetch('/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: payload
        }).catch(() => {});
    }
}

// === RC 제어 ===
function toggleRc() {
    rcEnabled = !rcEnabled;
    paintRcPill();
    cmd(rcEnabled ? 'ENABLE_WEB_RC' : 'DISABLE_WEB_RC');
}

function paintRcPill() {
    const pill = $('#rcpill');
    pill.textContent = rcEnabled ? 'RC ON' : 'RC OFF';
    pill.classList.toggle('on', rcEnabled);
}

// === 듀얼 스틱 컨트롤러 ===
function makeStick(rootSel, knobSel) {
    const el = $(rootSel);
    const knob = $(knobSel);
    const rect = () => el.getBoundingClientRect();
    let active = false, cx = 0, cy = 0, kx = 0, ky = 0, maxR = 0;

    function center() {
        const r = rect();
        cx = r.left + r.width / 2;
        cy = r.top + r.height / 2;
        maxR = Math.min(r.width, r.height) * 0.28;
    }

    function setKnob(x, y) {
        knob.style.transform = `translate(${x}px,${y}px)`;
    }

    function norm(dx, dy) {
        const r = Math.hypot(dx, dy) || 1;
        const rr = Math.min(r, maxR);
        return {
            x: (dx / rr) * (rr / maxR),
            y: (dy / rr) * (rr / maxR)
        };
    }

    function onDown(e) {
        active = true;
        const t = (e.touches ? e.touches[0] : e);
        center();
        onMove(e);
    }

    function onMove(e) {
        if (!active) return;
        const t = (e.touches ? e.touches[0] : e);
        const dx = t.clientX - cx;
        const dy = t.clientY - cy;
        const v = norm(dx, dy);
        kx = v.x * maxR;
        ky = v.y * maxR;
        setKnob(kx, ky);
    }

    function onUp() {
        active = false;
        kx = ky = 0;
        setKnob(0, 0);
    }

    // 이벤트 리스너
    el.addEventListener('pointerdown', onDown);
    el.addEventListener('pointermove', onMove);
    el.addEventListener('pointerup', onUp);
    el.addEventListener('pointercancel', onUp);
    el.addEventListener('pointerleave', onUp);
    el.addEventListener('touchstart', onDown, { passive: false });
    el.addEventListener('touchmove', onMove, { passive: false });
    el.addEventListener('touchend', onUp);
    el.addEventListener('touchcancel', onUp);
    window.addEventListener('resize', center);
    center();

    return {
        value() { // -1..+1
            const nx = (maxR ? kx / maxR : 0);
            const ny = (maxR ? ky / maxR : 0);
            return { x: nx, y: ny };
        }
    };
}

// === 3D 드론 시각화 ===
function init3DDrone() {
    const canvas = $('#drone-3d-canvas');
    if (!canvas) return;

    // Scene 생성
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0a);
    
    // Camera 설정
    camera = new THREE.PerspectiveCamera(75, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
    camera.position.set(2, 2, 2);
    camera.lookAt(0, 0, 0);
    
    // Renderer 설정
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    
    // 조명 설정
    const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 5, 5);
    directionalLight.castShadow = true;
    scene.add(directionalLight);
    
    // 포인트 라이트 추가 (드론 중심)
    const pointLight = new THREE.PointLight(0x4fc3f7, 0.5, 10);
    pointLight.position.set(0, 1, 0);
    scene.add(pointLight);
    
    // 드론 모델 생성
    createEnhancedDrone();
    
    // 지면 생성
    create3DGround();
    
    // 좌표계 헬퍼
    const axesHelper = new THREE.AxesHelper(1);
    scene.add(axesHelper);
    
    // 애니메이션 시작
    animate3D();
}

function createEnhancedDrone() {
    droneGroup = new THREE.Group();
    
    // 드론 바디 (중앙부)
    const bodyGeometry = new THREE.BoxGeometry(0.3, 0.08, 0.3);
    const bodyMaterial = new THREE.MeshLambertMaterial({ 
        color: 0x222222,
        transparent: true,
        opacity: 0.9 
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.castShadow = true;
    droneGroup.add(body);
    
    // LED 표시등 (중앙)
    const ledGeometry = new THREE.SphereGeometry(0.02, 8, 8);
    const ledMaterial = new THREE.MeshBasicMaterial({ 
        color: 0x00ff00,
        emissive: 0x004400 
    });
    const centerLED = new THREE.Mesh(ledGeometry, ledMaterial);
    centerLED.position.y = 0.05;
    droneGroup.add(centerLED);
    
    // 드론 암 (4개) 및 프로펠러
    const armGeometry = new THREE.CylinderGeometry(0.008, 0.012, 0.35);
    const armMaterial = new THREE.MeshLambertMaterial({ color: 0x444444 });
    
    const armPositions = [
        { x: 0.18, z: 0.18, rot: Math.PI/4, color: 0xff4444 },   // FL - Red
        { x: -0.18, z: 0.18, rot: -Math.PI/4, color: 0x44ff44 }, // FR - Green  
        { x: 0.18, z: -0.18, rot: -Math.PI/4, color: 0xff4444 }, // RL - Red
        { x: -0.18, z: -0.18, rot: Math.PI/4, color: 0x44ff44 }  // RR - Green
    ];
    
    propellers = []; // 프로펠러 배열 초기화
    
    armPositions.forEach((pos, i) => {
        // 암
        const arm = new THREE.Mesh(armGeometry, armMaterial);
        arm.position.set(pos.x * 0.85, 0, pos.z * 0.85);
        arm.rotation.z = pos.rot;
        arm.castShadow = true;
        droneGroup.add(arm);
        
        // 모터
        const motorGeometry = new THREE.CylinderGeometry(0.025, 0.025, 0.015);
        const motorMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
        const motor = new THREE.Mesh(motorGeometry, motorMaterial);
        motor.position.set(pos.x, 0.025, pos.z);
        motor.castShadow = true;
        droneGroup.add(motor);
        
        // LED 표시등 (각 암 끝)
        const armLED = new THREE.Mesh(ledGeometry, 
            new THREE.MeshBasicMaterial({ 
                color: pos.color,
                emissive: pos.color,
                emissiveIntensity: 0.3
            }));
        armLED.position.set(pos.x, 0.04, pos.z);
        droneGroup.add(armLED);
        
        // 프로펠러 그룹
        const propGroup = new THREE.Group();
        
        // 프로펠러 블레이드
        const blade1Geometry = new THREE.BoxGeometry(0.12, 0.001, 0.008);
        const blade2Geometry = new THREE.BoxGeometry(0.008, 0.001, 0.12);
        const propMaterial = new THREE.MeshLambertMaterial({ 
            color: pos.color,
            transparent: true,
            opacity: 0.7 
        });
        
        const blade1 = new THREE.Mesh(blade1Geometry, propMaterial);
        const blade2 = new THREE.Mesh(blade2Geometry, propMaterial);
        
        propGroup.add(blade1);
        propGroup.add(blade2);
        propGroup.position.set(pos.x, 0.04, pos.z);
        
        droneGroup.add(propGroup);
        propellers.push({
            group: propGroup,
            speed: 0,
            direction: i % 2 === 0 ? 1 : -1 // 교대로 회전 방향
        });
    });
    
    // 안테나
    const antennaGeometry = new THREE.CylinderGeometry(0.002, 0.002, 0.08);
    const antennaMaterial = new THREE.MeshLambertMaterial({ color: 0x888888 });
    const antenna = new THREE.Mesh(antennaGeometry, antennaMaterial);
    antenna.position.set(-0.1, 0.08, 0);
    droneGroup.add(antenna);
    
    scene.add(droneGroup);
}

function create3DGround() {
    // 투명한 지면
    const groundGeometry = new THREE.PlaneGeometry(8, 8);
    const groundMaterial = new THREE.MeshLambertMaterial({ 
        color: 0x2d5a87,
        transparent: true,
        opacity: 0.2 
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -1.5;
    ground.receiveShadow = true;
    scene.add(ground);
    
    // 격자 추가
    const gridHelper = new THREE.GridHelper(8, 16, 0x4fc3f7, 0x4fc3f7);
    gridHelper.position.y = -1.49;
    gridHelper.material.transparent = true;
    gridHelper.material.opacity = 0.3;
    scene.add(gridHelper);
}

function animate3D() {
    requestAnimationFrame(animate3D);
    
    if (!droneGroup || !renderer) return;
    
    // 자세 보간 (부드러운 움직임)
    const lerpFactor = 0.08;
    currentAttitude.roll += (targetAttitude.roll - currentAttitude.roll) * lerpFactor;
    currentAttitude.pitch += (targetAttitude.pitch - currentAttitude.pitch) * lerpFactor;
    currentAttitude.yaw += (targetAttitude.yaw - currentAttitude.yaw) * lerpFactor;
    
    // 드론 회전 적용 (라디안 변환)
    droneGroup.rotation.x = THREE.MathUtils.degToRad(currentAttitude.pitch);
    droneGroup.rotation.z = THREE.MathUtils.degToRad(-currentAttitude.roll);
    droneGroup.rotation.y = THREE.MathUtils.degToRad(currentAttitude.yaw);
    
    // 프로펠러 회전 및 모터 출력 반영
    propellers.forEach((prop, i) => {
        const motorNames = ['FL', 'FR', 'RL', 'RR'];
        const motorOutput = motorOutputs[motorNames[i]] || 1000;
        const normalizedOutput = Math.max(0, (motorOutput - 1000) / 1000); // 0-1 정규화
        
        // 모터 출력에 따른 회전 속도 계산
        prop.speed = normalizedOutput * 30; // 최대 30의 회전 속도
        prop.group.rotation.y += prop.speed * 0.02 * prop.direction;
        
        // 모터 출력에 따른 프로펠러 투명도 조절 (회전 효과)
        prop.group.children.forEach(blade => {
            blade.material.opacity = 0.7 - (normalizedOutput * 0.4); // 빠를수록 투명
        });
    });
    
    // 카메라 자동 회전
    if (autoRotate) {
        const time = Date.now() * 0.0005;
        camera.position.x = Math.cos(time) * cameraDistance;
        camera.position.z = Math.sin(time) * cameraDistance;
        camera.position.y = 2;
        camera.lookAt(droneGroup.position);
    }
    
    // 스로틀에 따른 드론 높이 조절
    const throttleValue = rc.throttle || 0;
    droneGroup.position.y = -0.5 + (throttleValue * 1.5); // 스로틀에 따라 높이 변경
    
    renderer.render(scene, camera);
}

// === 카메라 제어 함수들 ===
function resetCamera() {
    camera.position.set(2, 2, 2);
    camera.lookAt(0, 0, 0);
    cameraDistance = 4;
    autoRotate = false;
}

function toggleAutoRotate() {
    autoRotate = !autoRotate;
    if (!autoRotate) {
        camera.lookAt(droneGroup.position);
    }
}

// === 이벤트 핸들러 ===
function handleResize() {
    if (!camera || !renderer) return;
    const canvas = $('#drone-3d-canvas');
    if (!canvas) return;
    
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
}

function setup3DControls() {
    const canvas = $('#drone-3d-canvas');
    if (!canvas) return;
    
    let isDragging = false;
    let previousMousePosition = { x: 0, y: 0 };
    
    // 마우스 이벤트
    canvas.addEventListener('mousedown', (e) => {
        isDragging = true;
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    canvas.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        
        const deltaMove = {
            x: e.clientX - previousMousePosition.x,
            y: e.clientY - previousMousePosition.y
        };
        
        // 카메라 회전
        const rotationSpeed = 0.005;
        const cosAngle = Math.cos(deltaMove.x * rotationSpeed);
        const sinAngle = Math.sin(deltaMove.x * rotationSpeed);
        
        const newX = camera.position.x * cosAngle - camera.position.z * sinAngle;
        const newZ = camera.position.x * sinAngle + camera.position.z * cosAngle;
        
        camera.position.x = newX;
        camera.position.z = newZ;
        
        camera.lookAt(droneGroup.position);
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });
    
    canvas.addEventListener('mouseup', () => {
        isDragging = false;
    });
    
    // 휠 줌
    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const zoomSpeed = 0.1;
        cameraDistance += e.deltaY * zoomSpeed * 0.01;
        cameraDistance = Math.max(1, Math.min(10, cameraDistance));
        
        const direction = camera.position.clone().normalize();
        camera.position.copy(direction.multiplyScalar(cameraDistance));
        camera.lookAt(droneGroup.position);
    });
}

// === 초기화 ===
function initializeSticks() {
    $('#rcpill').addEventListener('click', toggleRc);
    
    const L = makeStick('#stickL', '#knobL'); // 스로틀/요
    const R = makeStick('#stickR', '#knobR'); // 롤/피치
    
    // 50Hz로 RC 전송
    setInterval(() => {
        if (!rcEnabled) return;
        
        const lv = L.value();
        const rv = R.value();
        
        // 매핑: 왼쪽 Y가 위로 갈수록 스로틀↑ (0..1), 왼쪽 X는 요(-1..+1)
        rc.throttle = Math.max(0, Math.min(1, (-lv.y + 1) / 2));
        rc.yaw = Math.max(-1, Math.min(1, lv.x));
        
        // 오른쪽 X/Y는 롤/피치(-1..+1), 화면 위쪽이 +피치
        rc.roll = Math.max(-1, Math.min(1, rv.x));
        rc.pitch = Math.max(-1, Math.min(1, -rv.y));

        const payload = JSON.stringify({ rc });
        if (ws && ws.readyState === 1) {
            ws.send(payload);
        } else {
            fetch('/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: payload
            }).catch(() => {});
        }
    }, 20);
}

// === 메인 초기화 ===
window.addEventListener('load', () => {
    // WebSocket 연결 시작
    connectWS();
    
    // 3D 드론 초기화
    init3DDrone();
    setup3DControls();
    
    // 스틱 컨트롤러 초기화
    initializeSticks();
    
    // 윈도우 리사이즈 핸들러
    window.addEventListener('resize', handleResize);
    
    console.log('Shane Drone Controller v0.4 initialized');
});