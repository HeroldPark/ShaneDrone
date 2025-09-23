// data/app.js
(function(){
  const $ = (id) => document.getElementById(id);

  // 상태 표시 엘리먼트
  const el = {
    armed: $("armed"), mode: $("mode"), battery: $("battery"), rc: $("rc"),
    roll: $("roll"), pitch: $("pitch"), yaw: $("yaw"),
    gx: $("gx"), gy: $("gy"), gz: $("gz"),
    mfl: $("mfl"), mfr: $("mfr"), mrl: $("mrl"), mrr: $("mrr"),
    throttle: $("throttle"), rollIn: $("rollIn"), pitchIn: $("pitchIn"), yawIn: $("yawIn"),
    rcOn: $("rcOn"), rcOff: $("rcOff"),
  };

  // WebSocket 연결
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.addEventListener("message", (e) => {
    try {
      const d = JSON.parse(e.data);
      // 상단 상태
      el.armed.textContent = `ARMED: ${d.armed ? "YES" : "NO"}`;
      el.mode.textContent = `MODE: ${d.mode ?? "-"}`;
      el.battery.textContent = `BAT: ${Number(d.battery ?? 0).toFixed(2)} V`;
      el.rc.textContent = `RC: ${d.rcEnabled ? "on" : "off"}`;

      // 자세/자이로
      el.roll.textContent  = (d.attitude?.roll  ?? "-").toFixed?.(2) ?? d.attitude?.roll ?? "-";
      el.pitch.textContent = (d.attitude?.pitch ?? "-").toFixed?.(2) ?? d.attitude?.pitch ?? "-";
      el.yaw.textContent   = (d.attitude?.yaw   ?? "-").toFixed?.(2) ?? d.attitude?.yaw ?? "-";

      el.gx.textContent = (d.gyro?.x ?? "-");
      el.gy.textContent = (d.gyro?.y ?? "-");
      el.gz.textContent = (d.gyro?.z ?? "-");

      // 모터
      el.mfl.textContent = d.motors?.fl ?? "-";
      el.mfr.textContent = d.motors?.fr ?? "-";
      el.mrl.textContent = d.motors?.rl ?? "-";
      el.mrr.textContent = d.motors?.rr ?? "-";
    } catch {}
  });

  // 명령 전송(HTTP)
  function sendCommand(command) {
    return fetch("/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command })
    }).then(r => r.text()).then(console.log).catch(console.error);
  }

  // RC On/Off
  el.rcOn.addEventListener("click", () => sendCommand("ENABLE_WEB_RC"));
  el.rcOff.addEventListener("click", () => sendCommand("DISABLE_WEB_RC"));

  // 버튼(섹션 하단)
  document.querySelectorAll("button[data-cmd]").forEach(btn => {
    btn.addEventListener("click", () => sendCommand(btn.dataset.cmd));
  });

  // RC 값 변경 시 즉시 전송(HTTP, 필요 시 WS로 바꿔도 OK)
  function sendRc() {
    const rc = {
      throttle: Number(el.throttle.value)/100,
      roll: Number(el.rollIn.value)/100,
      pitch: Number(el.pitchIn.value)/100,
      yaw: Number(el.yawIn.value)/100
    };
    fetch("/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ rc })
    }).catch(console.error);
  }
  ["input","change"].forEach(evt => {
    [el.throttle, el.rollIn, el.pitchIn, el.yawIn].forEach(x => x.addEventListener(evt, sendRc));
  });
})();
