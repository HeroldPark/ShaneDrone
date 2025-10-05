// Shane Drone Control - JavaScript

// Global variables
let ws = null;
let rcEnabled = false;
let rcData = {
    throttle: 0,
    roll: 0,
    pitch: 0,
    yaw: 0
};

// WebSocket connection
function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${protocol}//${window.location.host}/ws`);
    
    ws.onopen = function() {
        console.log('WebSocket connected');
        document.getElementById('ws-status').classList.add('connected');
    };
    
    ws.onclose = function() {
        console.log('WebSocket disconnected');
        document.getElementById('ws-status').classList.remove('connected');
        setTimeout(connectWebSocket, 2000);
    };
    
    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            updateDashboard(data);
        } catch (e) {
            console.error('Error parsing WebSocket data:', e);
        }
    };
    
    ws.onerror = function(error) {
        console.error('WebSocket error:', error);
    };
}

// Update dashboard with telemetry data
function updateDashboard(data) {
    // Attitude
    if (data.attitude) {
        updateElement('roll', `${data.attitude.roll.toFixed(1)}°`);
        updateElement('pitch', `${data.attitude.pitch.toFixed(1)}°`);
        updateElement('yaw', `${data.attitude.yaw.toFixed(1)}°`);
        
        // Update visual display
        updateElement('visual-roll', `${data.attitude.roll.toFixed(1)}°`);
        updateElement('visual-pitch', `${data.attitude.pitch.toFixed(1)}°`);
        updateElement('visual-yaw', `${data.attitude.yaw.toFixed(1)}°`);
        
        // Update drone visualization
        updateDroneVisualization(data.attitude.roll, data.attitude.pitch, data.attitude.yaw);
    }
    
    // Gyro
    if (data.gyro) {
        updateElement('gyro-x', `${data.gyro.x.toFixed(1)}°/s`);
        updateElement('gyro-y', `${data.gyro.y.toFixed(1)}°/s`);
        updateElement('gyro-z', `${data.gyro.z.toFixed(1)}°/s`);
    }
    
    // Altitude (기본값 0.0 - 백엔드에서 전송하지 않음)
    if (data.altitude !== undefined) {
        updateElement('altitude', `${data.altitude.toFixed(1)}m`);
    } else {
        updateElement('altitude', '0.0m');
    }
    
    // Battery
    if (data.battery !== undefined) {
        const voltage = data.battery;
        updateElement('battery', `${voltage.toFixed(2)}V`);
        updateBatteryBar(voltage);
    }
    
    // Flight mode
    if (data.mode !== undefined) {
        updateElement('mode', data.mode);
    }
    
    // ARM status
    if (data.armed !== undefined) {
        const armElement = document.getElementById('arm');
        armElement.textContent = data.armed ? 'ARMED' : 'DISARMED';
        armElement.className = data.armed ? 'value arm-status armed' : 'value arm-status disarmed';
        
        // Update propeller animation based on armed status
        updatePropellerAnimation(data.armed);
    }
    
    // Motors - 백엔드와 일치하는 필드명 사용
    if (data.motors) {
        updateMotor('fl', data.motors.fl);
        updateMotor('fr', data.motors.fr);
        updateMotor('rl', data.motors.rl);
        updateMotor('rr', data.motors.rr);
        
        // Update thrust arrow based on average motor output
        updateThrustArrow(data.motors);
    }
    
    // RC Input - 백엔드에서 input 객체로 전송
    if (data.input) {
        updateRCDisplay(data.input);
    }
    
    // RC Enabled status
    if (data.rcEnabled !== undefined) {
        rcEnabled = data.rcEnabled;
        updateRCPill();
        
        if(rcEnabled) {
            data.armed = true;  // for test
            updatePropellerAnimation(data.armed);
        }
    }
}

// Update drone visualization based on attitude
function updateDroneVisualization(roll, pitch, yaw) {
    const droneBody = document.getElementById('drone-body');
    if (droneBody) {
        // Apply 3D-like rotation effect
        // Roll: rotate around center
        // Pitch: scale and skew to simulate perspective
        // Yaw: rotate the entire body
        
        const rollRotation = roll;
        const pitchScale = 1 - Math.abs(pitch) / 180 * 0.3; // Scale down when pitched
        const yawRotation = yaw;
        
        droneBody.style.transformOrigin = 'center';
        droneBody.style.transform = `
            rotate(${rollRotation}deg) 
            scale(${pitchScale}, 1)
        `;
    }
}

// Update propeller animation
function updatePropellerAnimation(armed) {
    const propellers = document.querySelectorAll('.propeller');
    propellers.forEach(prop => {
        if (armed) {
            prop.style.animationPlayState = 'running';
        } else {
            prop.style.animationPlayState = 'paused';
        }
    });
}

// Update thrust arrow based on motor outputs
function updateThrustArrow(motors) {
    const avgMotor = (motors.fl + motors.fr + motors.rl + motors.rr) / 4;
    const thrustArrow = document.getElementById('thrust-arrow');
    
    if (thrustArrow) {
        // Calculate thrust intensity (1000 = no thrust, 2000 = max thrust)
        const thrustIntensity = Math.max(0, (avgMotor - 1000) / 1000);
        
        // Adjust arrow opacity and length based on thrust
        const line = thrustArrow.querySelector('line');
        if (line) {
            const baseY = 200;
            const maxLength = 80;
            const length = maxLength * thrustIntensity;
            line.setAttribute('y2', baseY - length);
        }
        
        thrustArrow.style.opacity = 0.3 + (thrustIntensity * 0.7);
    }
}

// Helper function to update element text
function updateElement(id, value) {
    const element = document.getElementById(id);
    if (element) {
        element.textContent = value;
    }
}

// Update battery bar
function updateBatteryBar(voltage) {
    const minVoltage = 10.0;
    const maxVoltage = 12.6;
    const percentage = Math.max(0, Math.min(100, ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100));
    
    const batteryLevel = document.getElementById('battery-level');
    if (batteryLevel) {
        batteryLevel.style.width = `${percentage}%`;
    }
}

// Update motor display
function updateMotor(motor, value) {
    const fillElement = document.getElementById(`motor-${motor}`);
    const valueElement = document.getElementById(`motor-${motor}-value`);
    
    if (fillElement && valueElement) {
        const percentage = Math.max(0, Math.min(100, ((value - 1000) / 1000) * 100));
        fillElement.style.width = `${percentage}%`;
        valueElement.textContent = value;
    }
}

// Update RC input display
function updateRCDisplay(input) {
    // Throttle (0-1)
    if (input.throttle !== undefined) {
        const throttlePct = (input.throttle * 100).toFixed(0);
        updateElement('rc-throttle', `${throttlePct}%`);
        const throttleBar = document.getElementById('rc-throttle-bar');
        if (throttleBar) {
            throttleBar.style.width = `${throttlePct}%`;
        }
    }
    
    // Roll (-1 to 1)
    if (input.roll !== undefined) {
        updateElement('rc-roll', input.roll.toFixed(2));
        updateCenteredBar('rc-roll-bar', input.roll);
    }
    
    // Pitch (-1 to 1)
    if (input.pitch !== undefined) {
        updateElement('rc-pitch', input.pitch.toFixed(2));
        updateCenteredBar('rc-pitch-bar', input.pitch);
    }
    
    // Yaw (-1 to 1)
    if (input.yaw !== undefined) {
        updateElement('rc-yaw', input.yaw.toFixed(2));
        updateCenteredBar('rc-yaw-bar', input.yaw);
    }
}

// Update centered progress bar for -1 to 1 values
function updateCenteredBar(barId, value) {
    const bar = document.getElementById(barId);
    if (bar) {
        const percentage = value * 50; // -1 to 1 => -50% to 50%
        const width = Math.abs(percentage);
        
        if (value >= 0) {
            bar.style.left = '50%';
            bar.style.width = `${width}%`;
        } else {
            bar.style.left = `${50 - width}%`;
            bar.style.width = `${width}%`;
        }
    }
}

// Send command to drone
function sendCommand(command) {
    const payload = JSON.stringify({ command: command });
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(payload);
    } else {
        // Fallback to HTTP POST
        fetch('/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: payload
        }).catch(err => console.error('Command error:', err));
    }
    
    // Visual feedback
    console.log(`Command sent: ${command}`);
}

// Toggle RC control
function toggleRC() {
    rcEnabled = !rcEnabled;
    sendCommand(rcEnabled ? 'ENABLE_WEB_RC' : 'DISABLE_WEB_RC');
    updateRCPill();
}

// Update RC pill display
function updateRCPill() {
    const pill = document.getElementById('rc-pill');
    if (pill) {
        pill.textContent = rcEnabled ? 'RC ON' : 'RC OFF';
        pill.classList.toggle('active', rcEnabled);
    }
}

// Dual Stick Control Implementation
class VirtualStick {
    constructor(containerId, knobId, onUpdate) {
        this.container = document.getElementById(containerId);
        this.knob = document.getElementById(knobId);
        this.onUpdate = onUpdate;
        
        this.active = false;
        this.centerX = 0;
        this.centerY = 0;
        this.knobX = 0;
        this.knobY = 0;

        this.touchId = null; // 멀티터치 식별자

        // 모바일 감지 및 maxRadius 설정
        this.updateMaxRadius();
        
        this.init();
    }

    updateMaxRadius() {
        this.isMobile = window.innerWidth <= 768;
        const isLandscape = window.innerWidth > window.innerHeight;
        
        if (this.isMobile && isLandscape) {
            this.maxRadius = 35; // 가로 모드: 70px의 절반
        } else if (this.isMobile) {
            this.maxRadius = 70; // 세로 모드: 140px의 절반
        } else {
            this.maxRadius = 140; // PC
        }
    }
    
    init() {
        // Touch events
        this.container.addEventListener('touchstart', this.onStart.bind(this), { passive: false });
        this.container.addEventListener('touchmove', this.onMove.bind(this), { passive: false });
        this.container.addEventListener('touchend', this.onEnd.bind(this));
        this.container.addEventListener('touchcancel', this.onEnd.bind(this));
        
        // Mouse events
        this.container.addEventListener('mousedown', this.onStart.bind(this));
        this.container.addEventListener('mousemove', this.onMove.bind(this));
        this.container.addEventListener('mouseup', this.onEnd.bind(this));
        this.container.addEventListener('mouseleave', this.onEnd.bind(this));
        
        // Calculate center
        this.updateCenter();
        window.addEventListener('resize', () => {
            this.updateMaxRadius();
            this.updateCenter();
        });
    }
    
    updateCenter() {
        const rect = this.container.getBoundingClientRect();
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
    }
    
    onStart(e) {
        e.preventDefault();

        // 이 컨테이너에서 "새로" 발생한 터치만 캡처
        if (e.changedTouches && e.changedTouches.length > 0) {
            if (this.touchId !== null) return;
            this.touchId = e.changedTouches[0].identifier;
        }

        this.active = true;
        this.onMove(e);
    }
    
    onMove(e) {
        if (!this.active) return;
        
        e.preventDefault();
        const rect = this.container.getBoundingClientRect();
        
        let clientX, clientY;
        if (e.touches) {
            // 멀티터치: 자신의 touchId와 일치하는 터치만 처리
            let touch = null;
            for (let i = 0; i < e.touches.length; i++) {
                if (e.touches[i].identifier === this.touchId) {
                    touch = e.touches[i];
                    break;
                }
            }
            if (!touch) return; // 자신의 터치가 아니면 무시
            
            clientX = touch.clientX;
            clientY = touch.clientY;
        } else {
            clientX = e.clientX;
            clientY = e.clientY;
        }
        
        const x = clientX - rect.left - this.centerX;
        const y = clientY - rect.top - this.centerY;
        
        // Limit to circular area
        const distance = Math.sqrt(x * x + y * y);
        if (distance <= this.maxRadius) {
            this.knobX = x;
            this.knobY = y;
        } else {
            const angle = Math.atan2(y, x);
            this.knobX = Math.cos(angle) * this.maxRadius;
            this.knobY = Math.sin(angle) * this.maxRadius;
        }
        
        this.updateKnobPosition();
        
        // Calculate normalized values (-1 to 1)
        const normalX = this.knobX / this.maxRadius;
        const normalY = -this.knobY / this.maxRadius; // Invert Y axis
        
        if (this.onUpdate) {
            this.onUpdate(normalX, normalY);
        }
    }
    
    onEnd() {
        // 터치 이벤트인 경우, 자신의 터치가 끝났는지 확인
        if (e.changedTouches) {
            let isMine = false;
            for (let i = 0; i < e.changedTouches.length; i++) {
                if (e.changedTouches[i].identifier === this.touchId) {
                    isMine = true;
                    break;
                }
            }
            if (!isMine) return; // 다른 터치가 끝난 것이면 무시
            e.preventDefault();
        }
        
        this.active = false;
        this.touchId = null; // 터치 ID 초기화
        this.knobX = 0;
        this.knobY = 0;
        this.updateKnobPosition();
        
        if (this.onUpdate) {
            this.onUpdate(0, 0);
        }
    }
    
    updateKnobPosition() {
        // 중앙 정렬(-50%, -50%)을 유지하면서 knobX, knobY만큼 이동
        this.knob.style.transform = `translate(calc(-50% + ${this.knobX}px), calc(-50% + ${this.knobY}px))`;
    }
}

// Initialize dual stick controls
let leftStick = null;
let rightStick = null;

function initStickControls() {
    // Left stick - Throttle (Y) and Yaw (X)
    leftStick = new VirtualStick('left-stick', 'left-knob', (x, y) => {
        // Y axis: -1 (bottom) to 1 (top) -> 0 to 1 throttle
        rcData.throttle = Math.max(0, Math.min(1, (y + 1) / 2));
        rcData.yaw = Math.max(-1, Math.min(1, x));
        
        // Update display
        updateElement('left-throttle', `${(rcData.throttle * 100).toFixed(0)}%`);
        updateElement('left-yaw', rcData.yaw.toFixed(2));
        
        if (rcEnabled) {
            sendRCData();
        }
    });
    
    // Right stick - Roll (X) and Pitch (Y)
    rightStick = new VirtualStick('right-stick', 'right-knob', (x, y) => {
        rcData.roll = Math.max(-1, Math.min(1, x));
        rcData.pitch = Math.max(-1, Math.min(1, y));
        
        // Update display
        updateElement('right-roll', rcData.roll.toFixed(2));
        updateElement('right-pitch', rcData.pitch.toFixed(2));
        
        if (rcEnabled) {
            sendRCData();
        }
    });
}

// Send RC data to server
function sendRCData() {
    const payload = JSON.stringify({ rc: rcData });
    
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(payload);
    } else {
        // Fallback to HTTP POST
        fetch('/command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: payload
        }).catch(err => console.error('RC data error:', err));
    }
}

// Periodic RC data sending (50Hz)
let rcInterval = null;

function startRCStream() {
    if (rcInterval) return;
    
    rcInterval = setInterval(() => {
        if (rcEnabled) {
            sendRCData();
        }
    }, 20); // 50Hz
}

function stopRCStream() {
    if (rcInterval) {
        clearInterval(rcInterval);
        rcInterval = null;
    }
}

// Fallback telemetry update via HTTP
function fetchTelemetry() {
    fetch('/telemetry')
        .then(response => response.json())
        .then(data => updateDashboard(data))
        .catch(err => console.error('Telemetry fetch error:', err));
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    console.log('Initializing Shane Drone Control...');
    
    // Connect WebSocket
    connectWebSocket();
    
    // Initialize stick controls
    initStickControls();
    
    // Start RC data stream
    startRCStream();
    
    // Fallback telemetry polling (when WebSocket is disconnected)
    setInterval(() => {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            fetchTelemetry();
        }
    }, 1000);
    
    // Add click event to RC pill
    document.getElementById('rc-pill').addEventListener('click', toggleRC);
});

// Keyboard controls (for testing)
document.addEventListener('keydown', function(e) {
    if (!rcEnabled) return;
    
    const step = 0.1;
    
    switch(e.key) {
        case 'w': // Throttle up
            rcData.throttle = Math.min(1, rcData.throttle + step);
            break;
        case 's': // Throttle down
            rcData.throttle = Math.max(0, rcData.throttle - step);
            break;
        case 'a': // Yaw left
            rcData.yaw = Math.max(-1, rcData.yaw - step);
            break;
        case 'd': // Yaw right
            rcData.yaw = Math.min(1, rcData.yaw + step);
            break;
        case 'ArrowLeft': // Roll left
            rcData.roll = Math.max(-1, rcData.roll - step);
            break;
        case 'ArrowRight': // Roll right
            rcData.roll = Math.min(1, rcData.roll + step);
            break;
        case 'ArrowUp': // Pitch forward
            rcData.pitch = Math.min(1, rcData.pitch + step);
            break;
        case 'ArrowDown': // Pitch backward
            rcData.pitch = Math.max(-1, rcData.pitch - step);
            break;
        case ' ': // Emergency stop
            e.preventDefault();
            sendCommand('EMERGENCY_STOP');
            break;
    }
    
    if (rcEnabled) {
        sendRCData();
    }
});

console.log('Shane Drone Control script loaded');