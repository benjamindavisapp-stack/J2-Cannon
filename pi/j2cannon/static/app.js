"use strict";

const $ = (id) => document.getElementById(id);
let S = null;              // latest /api/state
let shotsVersion = -1;
let strategyShown = false;
let lastClick = null;      // {nx, ny, t}
let jogVec = [0, 0];
let jogActive = false;
let streamStarted = false;

// ------------------------------------------------------------------ api

async function api(path, body, method = "POST") {
  try {
    const r = await fetch(path, {
      method,
      headers: { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    return await r.json();
  } catch (e) {
    return { ok: false, error: String(e) };
  }
}

async function poll() {
  try {
    const r = await fetch("/api/state");
    S = await r.json();
    render();
  } catch (e) {
    $("phase").textContent = "Lost connection to the Pi app";
    $("phase").className = "phase bad";
  }
  setTimeout(poll, 200);
}

// ------------------------------------------------------------------ rendering

const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v) ? "--" : Number(v).toFixed(d));

function chip(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = "chip " + (cls || "");
}

function ledState(fw, now) {
  if (!fw || fw.f === undefined) return "";
  if (fw.f) return "red";
  if (fw.c || fw.pst === "FILL") return Math.floor(now / 250) % 2 ? "red" : "";
  if (fw.tgt > 0 && fw.psi >= fw.tgt - S.hysteresis) return "green";
  return "";
}

const FAULTS = { 1: "Overpressure", 2: "Pressure sensor reads too low (unplugged?)", 3: "Pressure sensor reads too high",
  4: "Compressor ran but pressure did not rise", 5: "Compressor ran too long" };

function phaseText() {
  const fw = S.fw || {};
  if (!S.link.connected) return ["RedBoard not connected", "bad"];
  if (fw.f) return ["FAULT: " + (FAULTS[fw.f] || fw.f), "bad"];
  if (fw.z !== 3 && S.mode === "manual") return ["Not zeroed: jog to straight ahead + level, then Zero both", "busy"];
  if (S.mode === "aim") {
    return {
      moving: ["Aiming...", "busy"],
      pressurizing: ["Pressurizing...", "busy"],
      ready: ["READY: fire when clear", "ready"],
      aimed: ["Aimed (not pressurized)", ""],
      fired: ["Fired. Log the landing spot, or Re-arm", ""],
    }[S.phase] || ["Aim", ""];
  }
  if (S.mode === "track") {
    return {
      searching: ["Tracking: looking for target", "busy"],
      tracking: ["Tracking", "busy"],
      ready: ["READY (tracking): fire when clear", "ready"],
      lost: ["Tracking: target lost, holding", "busy"],
      "out of range": ["Tracking: out of range", "bad"],
      "no range": ["Tracking: can't range target", "bad"],
      fired: ["Fired. Re-arm to refill", ""],
    }[S.phase] || ["Tracking", "busy"];
  }
  if (fw.j) return ["Joystick", ""];
  return ["Manual", ""];
}

function render() {
  const fw = S.fw || {};
  $("simBadge").hidden = !S.sim;
  $("btnSimFire").hidden = !S.sim;

  chip("chipLink", S.link.connected ? `RedBoard ${S.link.port || ""}` : "RedBoard offline", S.link.connected ? (S.link.fresh ? "ok" : "warn") : "bad");
  const cam = S.camera;
  chip("chipCam", cam.online ? (cam.detection.valid ? "Limelight: target" : "Limelight") : "Limelight offline", cam.online ? "ok" : "bad");
  chip("chipZero", fw.z === 3 ? "Zeroed" : "Not zeroed", fw.z === 3 ? "ok" : "warn");
  chip("chipMotors", fw.en ? "Motors on" : "Motors off", fw.en ? "ok" : "warn");
  chip("chipMode", S.mode[0].toUpperCase() + S.mode.slice(1) + (S.armed ? " (armed)" : ""), S.mode === "manual" ? "" : "warn");

  const [ptext, pcls] = phaseText();
  $("phase").textContent = ptext;
  $("phase").className = "phase " + pcls;

  $("psi").textContent = fmt(fw.psi);
  const max = 80;
  $("gaugeFill").style.width = Math.max(0, Math.min(100, (fw.psi || 0) / max * 100)) + "%";
  $("gaugeFill").style.background = fw.f ? "var(--bad)" : (fw.c ? "var(--warn)" : "var(--accent)");
  const mark = $("gaugeMark");
  if (fw.tgt > 0) {
    mark.style.display = "block";
    mark.style.left = Math.min(100, fw.tgt / max * 100) + "%";
  } else mark.style.display = "none";
  $("pstate").textContent = { IDLE: "Idle", FILL: "Filling", HOLD: "Holding", MAN: "Manual fill" }[fw.pst] || "--";
  $("ptarget").textContent = fw.tgt > 0 ? `set ${fmt(fw.tgt)} psi` : "";
  $("led").className = "led " + ledState(fw, Date.now());
  $("fault").hidden = !fw.f;
  $("faultText").textContent = fw.f ? "Fault: " + (FAULTS[fw.f] || fw.f) : "";

  $("roYaw").textContent = fmt(fw.ya, 1) + "°";
  $("roPitch").textContent = fmt(fw.pa, 1) + "°";
  $("roYawT").textContent = fw.md && fw.md[0] === "P" ? "→ " + fmt(fw.yt, 1) + "°" : "";
  $("roPitchT").textContent = fw.md && fw.md[1] === "P" ? "→ " + fmt(fw.pt, 1) + "°" : "";
  $("roTarget").textContent = S.target_ft ? `${S.target_ft[0]}, ${S.target_ft[1]}, ${S.target_ft[2]} ft` : "none";

  renderSolution();
  renderStrategy();
  renderLog();
  renderCal();
  drawOverlay();
  startStream();
}

function renderSolution() {
  const sol = S.solution;
  const el = $("solution");
  if (!sol) return;
  if (!sol.feasible) {
    el.innerHTML = `<div class="err">No solution: ${sol.reason}</div>`;
    drawTraj(null);
    return;
  }
  const cells = [
    ["Yaw", fmt(sol.yaw_deg, 2) + "°"], ["Pitch", fmt(sol.pitch_deg, 2) + "°"],
    ["Pressure", fmt(sol.psi, 1) + " psi"], ["Distance", fmt(sol.distance_ft, 0) + " ft"],
    ["Muzzle speed", fmt(sol.v0_fps, 0) + " ft/s"], ["Flight", fmt(sol.flight_time_s, 2) + " s"],
    ["Apex", fmt(sol.apex_ft, 1) + " ft"], ["Arc", sol.arc + (sol.strategy === "fixed_pressure" ? " / fixed" : "")],
  ];
  let html = cells.map(([k, v]) => `<div class="cell"><div class="k">${k}</div><div class="v">${v}</div></div>`).join("");
  if (sol.reused_pressure) html += `<div class="note">Using the pressure already in the chamber (there is no vent valve).</div>`;
  if (S.target_source) html += `<div class="note">Target from ${S.target_source}.</div>`;
  el.innerHTML = html;
  drawTraj(sol);
}

function renderStrategy() {
  if (strategyShown) return;
  strategyShown = true;
  const s = S.strategy;
  document.querySelectorAll("input[name=strategy]").forEach((r) => (r.checked = r.value === s.strategy));
  document.querySelectorAll("input[name=arc]").forEach((r) => (r.checked = r.value === s.arc));
  $("fixedPsi").value = s.fixed_psi;
  $("fixedPsi").max = S.max_psi;
  $("manPsi").max = S.max_psi;
  $("optAuto").checked = s.auto_pressurize;
  $("optReuse").checked = s.reuse;
  $("optTrim").checked = s.trim;
}

function renderLog() {
  const el = $("log");
  const html = S.messages.slice().reverse().map((m) => {
    const t = new Date(m.t * 1000).toLocaleTimeString();
    return `<div class="${m.level}"><span class="time">${t}</span>${escapeHtml(m.text)}</div>`;
  }).join("");
  if (el.dataset.html !== html) {
    el.innerHTML = html;
    el.dataset.html = html;
  }
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

function renderCal() {
  const m = S.model;
  $("ratios").textContent = `Turret ${m.turret_gear_ratio} : 1 (${m.turret_steps_per_deg} steps/deg), ` +
    `pitch ${m.pitch_gear_ratio} : 1 (${m.pitch_steps_per_deg} steps/deg)` +
    (S.axis_test ? ` — test move: ${S.axis_test.axis} ${S.axis_test.commanded} deg` : "");
  $("modelInfo").textContent = `Model: efficiency ${m.efficiency}, drag coefficient ${m.drag_coefficient}, ` +
    `max muzzle speed ${m.max_speed_fps} ft/s at ${S.max_psi} psi. Camera offsets yaw ${m.camera_yaw_offset_deg}°, ` +
    `pitch ${m.camera_pitch_offset_deg}°.`;
  if (S.shots_version !== shotsVersion) {
    shotsVersion = S.shots_version;
    loadShots();
  }
  const fr = S.fit_result;
  const el = $("fitResult");
  if (!fr) {
    el.innerHTML = "";
    return;
  }
  const key = JSON.stringify(fr);
  if (el.dataset.key === key) return;
  el.dataset.key = key;
  const p = Object.entries(fr.params).map(([k, v]) => `${k} = ${v}`).join(", ");
  el.innerHTML = `<p>Fit: ${p}. RMS error ${fr.rms_ft} ft over ${fr.residuals.length} shot(s).</p>` +
    (fr.note ? `<p class="note">${escapeHtml(fr.note)}</p>` : "") +
    `<p class="muted">${fr.residuals.map((r) => `#${r.id}: ${r.error_ft >= 0 ? "+" : ""}${r.error_ft} ft`).join(", ")}</p>` +
    `<button id="btnFitApply" class="primary">Save this model</button>`;
  $("btnFitApply").onclick = () => api("/api/fit/apply", {});
}

async function loadShots() {
  const r = await fetch("/api/shots");
  const data = await r.json();
  const body = document.querySelector("#shots tbody");
  body.innerHTML = "";
  for (const s of data.shots.slice().reverse()) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td title="${s.time}">${s.id}</td><td>${fmt(s.pitch_deg, 1)}°</td><td>${fmt(s.psi, 1)}</td>` +
      `<td>${fmt(s.predicted_ft, 1)}</td>` +
      `<td><input type="number" step="0.5" value="${s.measured_ft ?? ""}" data-f="measured_ft"></td>` +
      `<td><input type="number" step="0.5" value="${s.landing_height_ft ?? 0}" data-f="landing_height_ft"></td>` +
      `<td><input type="checkbox" ${s.use !== false ? "checked" : ""} data-f="use"></td>` +
      `<td><button title="Delete">✕</button></td>`;
    tr.querySelectorAll("input").forEach((inp) => {
      inp.onchange = () => {
        const v = inp.type === "checkbox" ? inp.checked : (inp.value === "" ? null : Number(inp.value));
        api(`/api/shots/${s.id}`, { [inp.dataset.f]: v });
      };
    });
    tr.querySelector("button").onclick = () => {
      if (confirm(`Delete shot #${s.id}?`)) api(`/api/shots/${s.id}`, undefined, "DELETE");
    };
    body.appendChild(tr);
  }
}

// ------------------------------------------------------------------ video + overlay

function startStream() {
  if (streamStarted || !S.camera.stream) {
    $("videoNote").hidden = S.camera.stream && streamStarted && $("stream").dataset.ok === "1";
    return;
  }
  streamStarted = true;
  const img = $("stream");
  const local = ["localhost", "127.0.0.1"].includes(location.hostname);
  const src = local && S.camera.stream_url ? S.camera.stream_url : "/stream";
  img.onload = () => (img.dataset.ok = "1");
  img.onerror = () => {
    img.dataset.ok = "0";
    setTimeout(() => (img.src = src + (src.includes("?") ? "&" : "?") + "t=" + Date.now()), 5000);
  };
  img.src = src;
}

function drawOverlay() {
  const c = $("overlay");
  const ctx = c.getContext("2d");
  const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  const fw = S.fw || {};
  const noVideo = !S.camera.stream || $("stream").dataset.ok !== "1";
  const toPx = (p) => [(p[0] + 1) / 2 * W, (p[1] + 1) / 2 * H];

  if (noVideo && fw.pa !== undefined) {
    // Synthetic view: sky, ground and horizon from the barrel pitch.
    const pitch = fw.pa + S.model.camera_pitch_offset_deg;
    const ny = Math.tan(pitch * Math.PI / 180) / Math.tan(S.camera.vfov / 2 * Math.PI / 180);
    const hy = Math.max(-10, Math.min(10, ny));
    const y = (hy + 1) / 2 * H;
    ctx.fillStyle = "#0b1620";
    ctx.fillRect(0, 0, W, Math.max(0, Math.min(H, y)));
    ctx.fillStyle = "#14200f";
    ctx.fillRect(0, Math.max(0, Math.min(H, y)), W, H);
    ctx.strokeStyle = "#3b5d3a";
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(W, y);
    ctx.stroke();
  }

  // Crosshair = camera centre.
  ctx.strokeStyle = "rgba(255,255,255,0.6)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(W / 2 - 14, H / 2); ctx.lineTo(W / 2 + 14, H / 2);
  ctx.moveTo(W / 2, H / 2 - 14); ctx.lineTo(W / 2, H / 2 + 14);
  ctx.stroke();

  if (S.camera.det_px) {
    const [x, y] = toPx(S.camera.det_px);
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 2;
    ctx.strokeRect(x - 16, y - 16, 32, 32);
    ctx.fillStyle = "#22c55e";
    ctx.font = "13px system-ui";
    const d = S.camera.detection;
    ctx.fillText(d.tag_id !== null && d.tag_id !== undefined ? `tag ${d.tag_id}` + (d.tag_range_ft ? ` ${d.tag_range_ft} ft` : "") : "target", x + 20, y - 8);
  }
  if (S.target_px) {
    const [x, y] = toPx(S.target_px);
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 10, 0, Math.PI * 2);
    ctx.moveTo(x - 16, y); ctx.lineTo(x + 16, y);
    ctx.moveTo(x, y - 16); ctx.lineTo(x, y + 16);
    ctx.stroke();
  }
  if (lastClick && Date.now() - lastClick.t < 2000) {
    const [x, y] = toPx([lastClick.nx, lastClick.ny]);
    ctx.strokeStyle = "#60a5fa";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, Math.PI * 2);
    ctx.stroke();
  }
  if (noVideo) {
    $("videoNote").hidden = false;
    $("videoNote").textContent = S.sim ? "Simulated view (green box = simulated target)" :
      (S.camera.online ? "Video stream not loading" : "Limelight offline: " + (S.camera.error || ""));
  }
}

$("overlay").addEventListener("click", async (e) => {
  const rect = e.currentTarget.getBoundingClientRect();
  const nx = (e.clientX - rect.left) / rect.width * 2 - 1;
  const ny = (e.clientY - rect.top) / rect.height * 2 - 1;
  lastClick = { nx, ny, t: Date.now() };
  const mode = $("clickMode").value;
  if (mode === "boresight") {
    await api("/api/calibrate/boresight", { nx, ny, forward_ft: $("bsFwd").value, right_ft: $("bsRight").value, height_ft: $("bsHeight").value });
    return;
  }
  await api("/api/aim/click", { nx, ny, method: mode, distance_ft: $("clickDist").value, height_ft: $("clickHeight").value });
});

$("clickMode").onchange = () => {
  const m = $("clickMode").value;
  $("lblDist").hidden = m !== "distance";
  $("lblHeight").hidden = m !== "ground";
};

function drawTraj(sol) {
  const c = $("traj");
  const ctx = c.getContext("2d");
  const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  if (!sol || !sol.path_ft || sol.path_ft.length < 2) return;
  const pts = sol.path_ft;
  const tgt = S.target_ft;
  const xmax = Math.max(...pts.map((p) => p[0]), 1) * 1.05;
  const ymin = Math.min(0, ...pts.map((p) => p[1]));
  const ymax = Math.max(...pts.map((p) => p[1]), 1) * 1.15;
  const pad = 22;
  const sx = (W - 2 * pad) / xmax;
  const sy = (H - 2 * pad) / (ymax - ymin);  // separate scales: a side view, not to scale
  const X = (x) => pad + x * sx;
  const Y = (y) => H - pad - (y - ymin) * sy;
  ctx.strokeStyle = "#3b5d3a";
  ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(W, Y(0)); ctx.stroke();
  ctx.strokeStyle = "#60a5fa";
  ctx.lineWidth = 2;
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
  ctx.stroke();
  const end = pts[pts.length - 1];
  ctx.fillStyle = "#f59e0b";
  ctx.beginPath(); ctx.arc(X(end[0]), Y(end[1]), 5, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = "#8b98a7";
  ctx.font = "12px system-ui";
  ctx.fillText(`${fmt(sol.distance_ft, 0)} ft`, Math.min(X(end[0]) - 20, W - 60), Y(0) + 15);
  ctx.fillText(`apex ${fmt(sol.apex_ft, 1)} ft`, X(xmax / 2) - 30, 14);
  if (tgt) ctx.fillText(`target height ${tgt[2]} ft`, pad, 14);
}

// ------------------------------------------------------------------ controls

document.querySelectorAll(".tabs button").forEach((b) => {
  b.onclick = () => {
    document.querySelectorAll(".tabs button").forEach((x) => x.classList.toggle("active", x === b));
    document.querySelectorAll(".tab").forEach((t) => (t.hidden = t.id !== "tab-" + b.dataset.tab));
  };
});

$("btnStop").onclick = () => api("/api/stop", {});
$("btnAim").onclick = () => api("/api/aim/coords", { forward_ft: $("tFwd").value, right_ft: $("tRight").value, height_ft: $("tHeight").value });
$("btnLook").onclick = () => api("/api/look", {});
$("btnRearm").onclick = () => api("/api/rearm", {});
$("btnPStop").onclick = $("btnPStop2").onclick = () => api("/api/pressure/stop", {});
$("btnTrack").onclick = () => api("/api/track/start", { method: $("trackMethod").value });
$("btnTrackStop").onclick = () => api("/api/track/stop", {});
$("btnClear").onclick = $("btnClear2").onclick = () => api("/api/clear", {});
$("btnEnable").onclick = () => api("/api/enable", { on: true });
$("btnDisable").onclick = () => api("/api/enable", { on: false });
$("btnFill").onclick = () => api("/api/pressure", { psi: $("manPsi").value });
document.querySelectorAll("[data-zero]").forEach((b) => (b.onclick = () => api("/api/zero", { axis: b.dataset.zero })));
$("btnCalMove").onclick = () => api("/api/calibrate/axis/move", { axis: $("calAxis").value, degrees: $("calDeg").value });
$("btnCalApply").onclick = () => api("/api/calibrate/axis/apply", { axis: $("calAxis").value, measured: $("calMeasured").value });
$("btnFitE").onclick = () => api("/api/fit", { params: ["efficiency"] });
$("btnFitED").onclick = () => api("/api/fit", { params: ["efficiency", "drag_coefficient"] });
$("btnSimFire").onclick = () => api("/api/sim/fire", {});

function sendStrategy() {
  api("/api/strategy", {
    strategy: document.querySelector("input[name=strategy]:checked")?.value,
    arc: document.querySelector("input[name=arc]:checked")?.value,
    fixed_psi: $("fixedPsi").value,
    auto_pressurize: $("optAuto").checked,
    reuse: $("optReuse").checked,
    trim: $("optTrim").checked,
  });
}
document.querySelectorAll("input[name=strategy], input[name=arc], #fixedPsi, #optAuto, #optReuse, #optTrim")
  .forEach((el) => (el.onchange = sendStrategy));

// Jogging: buttons and arrow keys. The firmware stops by itself if these stop coming.
function setJog(y, p) {
  jogVec = [y, p];
  const speed = Number($("jogSpeed").value);
  api("/api/jog", { yaw: y * speed, pitch: p * speed });
  if ((y || p) && !jogActive) {
    jogActive = true;
    jogLoop();
  }
  if (!y && !p) jogActive = false;
}
function jogLoop() {
  if (!jogActive) return;
  const speed = Number($("jogSpeed").value);
  api("/api/jog", { yaw: jogVec[0] * speed, pitch: jogVec[1] * speed });
  setTimeout(jogLoop, 100);
}
document.querySelectorAll(".jogbtn").forEach((b) => {
  const start = (e) => { e.preventDefault(); setJog(Number(b.dataset.y), Number(b.dataset.p)); };
  const end = () => { if (jogActive) setJog(0, 0); };
  b.addEventListener("pointerdown", start);
  b.addEventListener("pointerup", end);
  b.addEventListener("pointerleave", end);
  b.addEventListener("pointercancel", end);
});

const keys = new Set();
function keyJog() {
  const y = (keys.has("ArrowRight") ? 1 : 0) - (keys.has("ArrowLeft") ? 1 : 0);
  const p = (keys.has("ArrowUp") ? 1 : 0) - (keys.has("ArrowDown") ? 1 : 0);
  if (y !== jogVec[0] || p !== jogVec[1]) setJog(y, p);
}
document.addEventListener("keydown", (e) => {
  if (["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement.tagName)) return;
  if (e.code === "Space") {
    e.preventDefault();
    api("/api/stop", {});
    return;
  }
  if (e.key.startsWith("Arrow")) {
    e.preventDefault();
    keys.add(e.key);
    keyJog();
  }
});
document.addEventListener("keyup", (e) => {
  if (e.key.startsWith("Arrow")) {
    keys.delete(e.key);
    keyJog();
  }
});
window.addEventListener("blur", () => { keys.clear(); if (jogActive) setJog(0, 0); });

poll();
