#!/usr/bin/env node
// auto-monitor.js (Node >=16, CommonJS)
// npm i -D serialport
const { SerialPort } = require('serialport');
const { spawn } = require('child_process');

const args = process.argv.slice(2);
const getArg = (k, def) => {
  const i = args.findIndex(a => a === k || a.startsWith(k + '='));
  if (i === -1) return def;
  return args[i].includes('=') ? args[i].split('=').slice(1).join('=') : args[i + 1] ?? def;
};

const baud = parseInt(getArg('-b', getArg('--baud', '115200')), 10);
const timeoutSec = parseInt(getArg('-t', getArg('--timeout', '20')), 10);

// 기본은 "CDC만": Arduino Nano ESP32 = 2341:0070
const candStr = getArg('-c', getArg('--candidates', '2341:0070')).toLowerCase();
const candidates = candStr.split(',').map(s => s.trim()).filter(Boolean);

// 업로드에 사용한 포트를 제외
const excludeStr = getArg('--exclude', '');
const excludePorts = excludeStr.split(',').map(s => s.trim().toUpperCase()).filter(Boolean);

// 키워드 보조(윈도우에서 "USB 직렬 장치" 등)
const kwStr = getArg('--keywords', 'usb serial device,cdc,arduino');
const keywords = kwStr.split(',').map(s => s.trim().toLowerCase()).filter(Boolean);

// 업로드 직후 포트 전환 시간 확보(기본 800ms)
const initialDelayMs = parseInt(getArg('--initial-delay-ms', '800'), 10);

function matchByVidPid(p) {
  const vid = (p.vendorId || '').toLowerCase();
  const pid = (p.productId || '').toLowerCase();
  if (!vid || !pid) return false;
  return candidates.includes(`${vid}:${pid}`);
}
function matchByDesc(p) {
  const s = [
    p.path, p.friendlyName, p.manufacturer, p.serialNumber, p.pnpId, p.locationId, p.productId, p.vendorId
  ].filter(Boolean).join(' ').toLowerCase();
  return keywords.some(k => s.includes(k));
}

async function findCdcPort(deadline) {
  let lastSeen = null;
  while (Date.now() < deadline) {
    const ports = await SerialPort.list();
    for (const p of ports) {
      const path = (p.path || '').toUpperCase();
      if (excludePorts.includes(path)) continue;           // 업로드 포트 제외
      if (matchByVidPid(p)) return p.path;                 // VID/PID 우선
      if (matchByDesc(p)) lastSeen = p.path;               // 키워드 보조
    }
    await new Promise(r => setTimeout(r, 200));
  }
  return lastSeen;
}

(async () => {
  if (initialDelayMs > 0) await new Promise(r => setTimeout(r, initialDelayMs));
  const deadline = Date.now() + timeoutSec * 1000;
  console.log(`[auto-monitor] waiting CDC port (timeout: ${timeoutSec}s, baud: ${baud})`);
  if (excludePorts.length) console.log(`[auto-monitor] excluding: ${excludePorts.join(', ')}`);
  console.log(`[auto-monitor] candidates (VID:PID): ${candidates.join(', ')}`);

  const port = await findCdcPort(deadline);
  if (!port) {
    console.error('[auto-monitor] CDC port not found. Try `pio device list`.');
    process.exit(1);
  }
  console.log(`[auto-monitor] opening monitor on ${port} @ ${baud}`);
  const cmd = `pio device monitor -p "${port}" -b ${baud}`;
  const child = spawn(cmd, { stdio: 'inherit', shell: true });
  child.on('exit', code => process.exit(code ?? 0));
})();
