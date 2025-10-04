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
    }
    
    // Gyro
    if (data.gyro) {
        updateElement('gyro-x', `${data.gyro.x.toFixed(1)}°/s`);
        updateElement('gyro-y', `${data.gyro.y.toFixed(1)}°/s`);
        updateElement('gyro-z', `${data.gyro.z.toFixed(1)}°/s`);
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
    }
    
    // Motors
    if (data.motors) {
        updateMotor('fl', data.motors.fl);
        updateMotor('fr', data.motors.fr);
        updateMotor('rl', data.motors.rl);
        updateMotor('rr', data.motors.rr);
    }
    
    // RC Input
    if (data.input) {
        updateRCDisplay(data.input);
    }
    
    // RC Enabled status
    if (data.rcEnabled !== undefined) {
        rcEnabled = data.rcEnabled;
        updateRCPill();
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
function updateRCPillRCPill() {
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
        this.maxRadius = 70;
        
        this.init();
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
        window.addEventListener('resize', this.updateCenter.bind(this));
    }
    
    updateCenter() {
        const rect = this.container.getBoundingClientRect();
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
    }
    
    onStart(e) {
        e.preventDefault();
        this.active = true;
        this.onMove(e);
    }
    
    onMove(e) {
        if (!this.active) return;
        
        e.preventDefault();
        const rect = this.container.getBoundingClientRect();
        
        let clientX, clientY;
        if (e.touches) {
            clientX = e.touches[0].clientX;
            clientY = e.touches[0].clientY;
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
        this.active = false;
        this.knobX = 0;
        this.knobY = 0;
        this.updateKnobPosition();
        
        if (this.onUpdate) {
            this.onUpdate(0, 0);
        }
    }
    
    updateKnobPosition() {
        this.knob.style.transform = `translate(${this.knobX}px, ${this.knobY}px)`;
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