"use strict";

const state = {
  samples: [], events: [], params: {}, groups: {}, dirty: new Set(),
  lastSampleId: 0, lastEventId: 0, paused: false, windowSeconds: 30,
  lastRender: 0, renderPending: false, status: null, sampleTimes: [],
  viewMode: "live", sessionSamples: [], sessionAnalysis: null,
  apiToken: "", portPickerBusy: false,
  steeringPendingAngle: null, steeringCommandActive: false,
  steeringDebounce: null,
};

const colors = { green: "#56d5ee", cyan: "#67d4ff", amber: "#f5c870", violet: "#b8a5ff", red: "#ff7e8c", gray: "#aab9c9" };
const dangerousFlags = new Set(["BIAS_SAT", "VELOCITY_SAT", "ACCEL_SAT", "ZERO_LIMIT", "ZERO_PERSIST_ERROR", "ODRIVE_TIMEOUT", "ODRIVE_FAULT", "ODRIVE_RECOVERY_FAILED", "UART7_RX_OVERFLOW", "RATE_TARGET_SAT", "FALL_DISARM"]);

const $ = (selector) => document.querySelector(selector);
const fmt = (value, digits = 2) => value === undefined || value === null || value === "" ? "--" : Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";

class TelemetryChart {
  constructor(canvasId, series, options = {}) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.series = series;
    this.options = options;
  }
  render(samples, seconds, sessionMode = false) {
    const rect = this.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(300, Math.floor(rect.width * ratio));
    const height = Math.max(150, Math.floor(rect.height * ratio));
    if (this.canvas.width !== width || this.canvas.height !== height) { this.canvas.width = width; this.canvas.height = height; }
    const ctx = this.ctx; ctx.clearRect(0, 0, width, height);
    const pad = { l: 48 * ratio, r: 12 * ratio, t: 28 * ratio, b: 24 * ratio };
    const plotW = width - pad.l - pad.r, plotH = height - pad.t - pad.b;
    const now = samples.length ? samples[samples.length - 1].host_time_s : performance.now() / 1000;
    const start = sessionMode && samples.length ? samples[0].host_time_s : now - seconds;
    const visible = samples.filter(s => s.host_time_s >= start);
    let values = [];
    for (const s of visible) for (const line of this.series) { const v = line.value(s); if (Number.isFinite(v)) values.push(v); }
    let min = this.options.min ?? (values.length ? Math.min(...values) : -1);
    let max = this.options.max ?? (values.length ? Math.max(...values) : 1);
    if (this.options.symmetric) { const extent = Math.max(Math.abs(min), Math.abs(max), this.options.minExtent || 0.1); min = -extent; max = extent; }
    if (min === max) { min -= 1; max += 1; }
    const span = max - min, margin = this.options.fixedRange ? 0 : span * .1;
    if (!this.options.fixedRange && !this.options.symmetric) { min -= margin; max += margin; }
    const x = t => pad.l + ((t - start) / seconds) * plotW;
    const y = v => pad.t + (1 - (v - min) / (max - min)) * plotH;
    ctx.font = `${10 * ratio}px SFMono-Regular, Consolas, monospace`;
    ctx.lineWidth = ratio;
    for (let i = 0; i <= 4; i++) {
      const yy = pad.t + (plotH * i / 4); const value = max - (max - min) * i / 4;
      ctx.strokeStyle = "rgba(148,163,184,.12)"; ctx.beginPath(); ctx.moveTo(pad.l, yy); ctx.lineTo(width - pad.r, yy); ctx.stroke();
      ctx.fillStyle = "#667382"; ctx.textAlign = "right"; ctx.fillText(value.toFixed(Math.abs(value) < 2 ? 2 : 1), pad.l - 7 * ratio, yy + 3 * ratio);
    }
    for (let i = 0; i <= 3; i++) {
      const xx = pad.l + plotW * i / 3; ctx.strokeStyle = "rgba(148,163,184,.07)"; ctx.beginPath(); ctx.moveTo(xx, pad.t); ctx.lineTo(xx, height - pad.b); ctx.stroke();
      const tick = sessionMode ? seconds * i / 3 : -seconds + seconds * i / 3;
      ctx.fillStyle = "#667382"; ctx.textAlign = "center"; ctx.fillText(`${tick.toFixed(seconds < 10 ? 1 : 0)}s`, xx, height - 7 * ratio);
    }
    if (min < 0 && max > 0) { ctx.strokeStyle = "rgba(230,237,243,.22)"; ctx.beginPath(); ctx.moveTo(pad.l, y(0)); ctx.lineTo(width - pad.r, y(0)); ctx.stroke(); }
    for (const line of this.series) {
      ctx.strokeStyle = line.color; ctx.lineWidth = (line.width || 1.5) * ratio; ctx.setLineDash((line.dash || []).map(n => n * ratio)); ctx.beginPath();
      let started = false;
      for (const sample of visible) { const value = line.value(sample); if (!Number.isFinite(value)) continue; const xx = x(sample.host_time_s), yy = y(value); if (!started) { ctx.moveTo(xx, yy); started = true; } else ctx.lineTo(xx, yy); }
      ctx.stroke(); ctx.setLineDash([]);
    }
    let legendX = pad.l;
    for (const line of this.series) { ctx.fillStyle = line.color; ctx.fillRect(legendX, 8 * ratio, 14 * ratio, 2 * ratio); ctx.fillStyle = "#94a3b8"; ctx.textAlign = "left"; ctx.fillText(line.label, legendX + 19 * ratio, 12 * ratio); legendX += (line.label.length * 8 + 42) * ratio; }
  }
}

const bv = (key) => s => Number(s.balance?.[key]);
const ov = (key) => s => Number(s.odrive?.[key]);
const charts = [
  new TelemetryChart("chart-roll", [
    { label: "误差", color: colors.green, width: 2, value: bv("roll_error_deg") },
    { label: "横滚", color: colors.cyan, value: bv("roll_deg") },
    { label: "零点", color: colors.amber, dash: [5, 4], value: bv("zero_control_deg") },
  ]),
  new TelemetryChart("chart-rate", [
    { label: "实际", color: colors.cyan, width: 2, value: bv("roll_rate_dps") },
    { label: "目标", color: colors.amber, dash: [5, 4], value: bv("rate_target_dps") },
  ], { symmetric: true, minExtent: 1 }),
  new TelemetryChart("chart-wheel", [
    { label: "轮速", color: colors.green, width: 2, value: bv("wheel_tps") },
    { label: "速度指令", color: colors.violet, dash: [5, 4], value: bv("vel_cmd_tps") },
  ], { symmetric: true, minExtent: 5 }),
  new TelemetryChart("chart-accel", [
    { label: "原始需求", color: colors.red, value: bv("accel_raw_tps2") },
    { label: "限幅输出", color: colors.amber, width: 2, value: bv("accel_cmd_tps2") },
  ], { symmetric: true, minExtent: 20 }),
  new TelemetryChart("chart-pid", [
    { label: "P", color: colors.cyan, value: bv("rate_p_tps") },
    { label: "I", color: colors.green, value: bv("rate_i_tps") },
    { label: "D", color: colors.violet, value: bv("rate_d_tps") },
  ], { symmetric: true, minExtent: 5 }),
  new TelemetryChart("chart-current", [
    { label: "Iq 指令", color: colors.amber, dash: [5, 4], value: ov("iq_setpoint_a") },
    { label: "Iq 实测", color: colors.green, width: 2, value: ov("iq_measured_a") },
    { label: "母线电流", color: colors.cyan, value: ov("ibus_a") },
  ], { symmetric: true, minExtent: 10 }),
];

function makePidControls() {
  const names = { rate: ["角速度环", "内环 · 加速度"], angle: ["角度环", "外环 · 目标角速度"], wheel: ["轮速回正环", "零均速偏置"], steer: ["转向环", "航向 PID"] };
  const root = $("#pid-groups"); root.innerHTML = "";
  for (const [key, params] of Object.entries(state.groups)) {
    const section = document.createElement("section"); section.className = "pid-group";
    section.innerHTML = `<div class="pid-group-title"><h3>${names[key][0]}</h3><span>${names[key][1]}</span></div>`;
    for (const p of params) {
      const row = document.createElement("div"); row.className = "pid-row"; row.dataset.name = p.name;
      row.innerHTML = `<label for="input-${p.name}">${p.label}</label><span class="live-value" id="live-${p.name}">--</span><input class="pid-input" id="input-${p.name}" type="number" min="${p.min}" max="${p.max}" step="${p.step}" aria-label="${p.name} 编辑值"><button class="step-button" type="button" data-step="-${p.step}" aria-label="减小 ${p.name}">−</button><button class="step-button" type="button" data-step="${p.step}" aria-label="增大 ${p.name}">+</button><button class="apply-one" type="button">应用</button>`;
      const input = row.querySelector("input");
      input.addEventListener("input", () => { state.dirty.add(p.name); input.classList.add("dirty"); });
      row.querySelectorAll(".step-button").forEach(button => button.addEventListener("click", () => {
        const current = Number(input.value || state.params[p.name] || 0); input.value = String(Number((current + Number(button.dataset.step)).toFixed(6))); input.dispatchEvent(new Event("input"));
      }));
      row.querySelector(".apply-one").addEventListener("click", () => applyParam(p.name));
      section.appendChild(row);
    }
    root.appendChild(section);
  }
}

async function api(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options, headers: { "Content-Type": "application/json", ...(state.apiToken ? { "X-Bike-Token": state.apiToken } : {}), ...(options.headers || {}) } });
  const data = await response.json();
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

async function post(path, payload) { return api(path, { method: "POST", body: JSON.stringify(payload) }); }

async function applyParam(name) {
  const input = document.getElementById(`input-${name}`); const value = Number(input.value);
  setFeedback(`正在设置 ${name} = ${value}…`);
  try { const result = await post("/api/tune", { action: "set", name, value }); state.dirty.delete(name); input.classList.remove("dirty"); setFeedback(result.reply, true); showToast(`${name} 已生效`); }
  catch (error) { setFeedback(error.message, false); showToast(error.message, true); }
}

async function applyAll() {
  const names = [...state.dirty];
  if (!names.length) return showToast("没有待应用的改动");
  for (const name of names) { await applyParam(name); if (state.dirty.has(name)) break; }
}

function setFeedback(message, ok = null) { const el = $("#command-feedback"); el.textContent = message; el.className = `command-feedback ${ok === true ? "ok" : ok === false ? "error" : ""}`; }
function setSteeringFeedback(message, ok = null) { const el = $("#steering-feedback"); el.textContent = message; el.className = `command-feedback ${ok === true ? "ok" : ok === false ? "error" : ""}`; }
let toastTimer;
function showToast(message, error = false) { const el = $("#toast"); el.textContent = message; el.className = `toast visible${error ? " error" : ""}`; clearTimeout(toastTimer); toastTimer = setTimeout(() => el.className = "toast", 2600); }

function validateSteeringAngle(value) {
  const rounded = Math.round(Number(value) * 2) / 2;
  if (!Number.isFinite(rounded) || Math.abs(rounded) > 15) throw new Error("转向角范围为 -15° … +15°");
  if (rounded !== 0 && Math.abs(rounded) < 5) throw new Error("机械死区内不可用；请选择回中或至少 ±5°");
  return rounded;
}

function steeringIndexToAngle(index) {
  const step = Math.max(0, Math.min(42, Math.round(Number(index))));
  if (step <= 20) return -15 + step * 0.5;
  if (step === 21) return 0;
  return 5 + (step - 22) * 0.5;
}

function steeringAngleToIndex(angle) {
  const value = validateSteeringAngle(angle);
  if (value < 0) return (value + 15) / 0.5;
  if (value === 0) return 21;
  return 22 + (value - 5) / 0.5;
}

function updateSteeringSliderLabel() {
  const angle = steeringIndexToAngle($("#steering-slider").value);
  $("#steering-slider-value").textContent = angle === 0 ? "回中 · 0.0°" : `${angle > 0 ? "左" : "右"} ${Math.abs(angle).toFixed(1)}°`;
}

function setSteeringBusy(busy) {
  const panel = document.querySelector(".steering-panel");
  panel.setAttribute("aria-busy", String(busy));
}

async function flushSteeringCommand() {
  if (state.steeringCommandActive || state.steeringPendingAngle === null) return;
  const angle = state.steeringPendingAngle;
  state.steeringPendingAngle = null;
  state.steeringCommandActive = true;
  setSteeringFeedback(angle === 0 ? "正在平滑回中…" : `正在平滑转向至${angle > 0 ? "左" : "右"} ${Math.abs(angle).toFixed(1)}°…`);
  setSteeringBusy(true);
  try {
    await post("/api/steering", { angle_deg: angle });
    if (state.status) {
      state.status.steering_target_deg = angle;
      state.status.steering_target_us = angle * 10;
    }
    setSteeringFeedback(angle === 0 ? "正在平滑回中" : `目标已更新：${angle > 0 ? "左" : "右"} ${Math.abs(angle).toFixed(1)}°`, true);
  } catch (error) {
    setSteeringFeedback(error.message, false);
    showToast(error.message, true);
  } finally {
    state.steeringCommandActive = false;
    setSteeringBusy(false);
    if (state.steeringPendingAngle !== null && Math.abs(state.steeringPendingAngle - angle) < 0.01) {
      state.steeringPendingAngle = null;
    }
    if (state.steeringPendingAngle !== null) flushSteeringCommand();
  }
}

function commandSteering(value, debounce = false) {
  let angle;
  try { angle = validateSteeringAngle(value); }
  catch (error) { setSteeringFeedback(error.message, false); showToast(error.message, true); return; }
  $("#steering-slider").value = String(steeringAngleToIndex(angle));
  updateSteeringSliderLabel();
  state.steeringPendingAngle = angle;
  clearTimeout(state.steeringDebounce);
  if (debounce) state.steeringDebounce = setTimeout(flushSteeringCommand, 140);
  else flushSteeringCommand();
}

function setDriveBusy(busy) {
  document.querySelectorAll(".drive-command,#drive-demo,#drive-speed-slider").forEach(control => { control.disabled = busy; });
  $("#drive-stop").disabled = false;
}

function setDriveFeedback(message, ok = null) {
  const el = $("#drive-feedback"); el.textContent = message;
  el.className = `command-feedback ${ok === true ? "ok" : ok === false ? "error" : ""}`;
}

async function commandDrive(action) {
  const speed = Number($("#drive-speed-slider").value);
  const payload = { action };
  if (action === "manual") payload.speed_mps = speed;
  if (action === "reverse") { payload.action = "manual"; payload.speed_mps = -speed; }
  if (action === "demo") payload.speed_mps = speed;
  setDriveBusy(true);
  setDriveFeedback(action === "stop" ? "正在停止后轮…" : "正在发送后轮指令…");
  try {
    const result = await post("/api/drive", payload);
    setDriveFeedback(result.reply, true);
    showToast(action === "stop" ? "后轮正在平滑停止" : "后轮指令已生效");
  } catch (error) { setDriveFeedback(error.message, false); showToast(error.message, true); }
  finally { setDriveBusy(false); }
}

async function setTelemetryMode(mode) {
  const button = $("#telemetry-button"); button.disabled = true;
  try {
    const result = await post("/api/telemetry", { mode });
    if (state.status) state.status.telemetry_mode = result.mode;
    showToast(mode === "live" ? "已开启详细 IMU 遥测" : "已切换到低带宽基础状态");
  } catch (error) { showToast(error.message, true); throw error; }
  finally { button.disabled = false; }
}

async function recoverOdrive() {
  if (!confirm("确认车体已扶正、后轮已停止，并且有人扶稳车辆？恢复成功后将重新进入 2 秒启动计时。")) return;
  const button = $("#odrive-recover");
  const feedback = $("#odrive-recovery-feedback");
  button.disabled = true;
  button.setAttribute("aria-busy", "true");
  feedback.textContent = "正在请求安全恢复…";
  feedback.className = "command-feedback";
  try {
    const result = await post("/api/odrive", { action: "recover" });
    feedback.textContent = result.reply;
    feedback.className = "command-feedback ok";
    showToast("恢复流程已请求，请继续扶稳 2 秒");
  } catch (error) {
    feedback.textContent = error.message;
    feedback.className = "command-feedback error";
    showToast(error.message, true);
  } finally {
    button.disabled = false;
    button.removeAttribute("aria-busy");
  }
}

function ingestSamples(samples) {
  for (const sample of samples) { if (sample.id <= state.lastSampleId) continue; state.samples.push(sample); state.lastSampleId = sample.id; state.sampleTimes.push(sample.host_time_s); }
  const cutoff = (state.samples.at(-1)?.host_time_s || 0) - 125;
  while (state.samples.length && state.samples[0].host_time_s < cutoff) state.samples.shift();
  while (state.sampleTimes.length && state.sampleTimes[0] < cutoff) state.sampleTimes.shift();
  if (samples.length) scheduleRender();
}
function ingestEvents(events) { for (const event of events) { if (event.id <= state.lastEventId) continue; state.events.push(event); state.lastEventId = event.id; } state.events = state.events.slice(-100); renderEvents(); }

function updateParams(params) {
  state.params = { ...state.params, ...params };
  for (const [name, value] of Object.entries(state.params)) {
    const live = document.getElementById(`live-${name}`); const input = document.getElementById(`input-${name}`);
    if (live) live.textContent = fmt(value, Math.abs(value) < .1 ? 4 : 2);
    if (input && !state.dirty.has(name) && document.activeElement !== input) input.value = Number(value).toString();
  }
}

function renderStatus(status) {
  state.status = status; updateParams(status.params || {});
  const uartFresh = status.uart_connected && status.uart_age_ms < 800;
  setPill("#uart-status", uartFresh ? "UART7 在线" : "UART7 断开", uartFresh ? "online" : "danger");
  setPill("#odrive-status", status.odrive_connected ? "ODrive 在线" : "ODrive 断开", status.odrive_connected ? "online" : "danger");
  const basicFlags = Number(status.basic?.balance_flags || 0);
  const armed = status.telemetry_mode === "basic" ? Boolean(basicFlags & (1 << 7)) : Boolean((status.balance?.flag_names || []).includes("CONTROL_ARMED"));
  const fall = status.telemetry_mode === "basic" ? Boolean(basicFlags & (1 << 20)) : Boolean((status.balance?.flag_names || []).includes("FALL_DISARM"));
  setPill("#control-status", fall ? "已倒车保护" : armed ? "平衡已启动" : "等待启动", fall ? "danger" : armed ? "online" : "warning");
  $("#freshness").textContent = `UART ${fmt(status.uart_age_ms,0)} ms · ODrive ${fmt(status.odrive_age_ms,0)} ms`;
  const b = status.balance || {}, o = status.odrive || {};
  const detailed = status.telemetry_mode === "live";
  $("#metric-roll-error").textContent = detailed ? signed(b.roll_error_deg, 3) : "--"; $("#metric-roll-rate").textContent = detailed ? signed(b.roll_rate_dps, 2) : "--";
  $("#metric-wheel").textContent = detailed ? signed(b.wheel_tps, 2) : "--"; $("#metric-accel").textContent = detailed ? signed(b.accel_cmd_tps2, 1) : "--";
  $("#metric-current").textContent = signed(o.iq_measured_a, 1); $("#metric-vbus").textContent = fmt(o.vbus_v, 1);
  renderFlags(detailed ? (b.flag_names || []) : [armed ? "CONTROL_ARMED" : "CONTROL_IDLE", fall ? "FALL_DISARM" : "BASIC_TELEMETRY"]); renderHardware(o);
  const steering = status.steering || {};
  const steeringTarget = Number(status.steering_target_deg ?? (Number(status.steering_target_us ?? 0) / 10));
  const steeringOutput = Number(steering.commanded_offset_us) / 10;
  const steeringFlags = Number(steering.flags || 0);
  $("#steering-target").textContent = signed(steeringTarget, 1);
  $("#steering-output").textContent = Number.isFinite(steeringOutput) ? signed(steeringOutput, 1) : "--";
  $("#steering-motion").textContent = (steeringFlags & (1 << 9)) ? "消隙回中" : (steeringFlags & (1 << 4)) ? "平滑转向" : Number.isFinite(steeringOutput) ? "已到位" : "--";
  $("#steering-direction").textContent = steeringTarget > 0 ? "左转" : steeringTarget < 0 ? "右转" : "回中";
  if (document.activeElement !== $("#steering-slider")) {
    try { $("#steering-slider").value = String(steeringAngleToIndex(steeringTarget)); updateSteeringSliderLabel(); } catch (_) {}
  }
  const drive = status.drive || {};
  const driveMode = Number(drive.mode ?? 0);
  const recoverButton = $("#odrive-recover");
  if (!recoverButton.hasAttribute("aria-busy")) recoverButton.disabled = !uartFresh || armed || driveMode !== 0;
  const driveModeNames = ["静止", "手动", "循环演示"];
  $("#drive-mode").textContent = driveModeNames[driveMode] || "未知";
  $("#drive-target").textContent = signed(drive.requested_speed_mps, 2);
  $("#drive-output").textContent = signed(drive.output_speed_mps, 2);
  $("#drive-actual").textContent = signed(drive.actual_speed_mps, 2);
  $("#drive-odometry").textContent = signed(drive.odometry_m, 2);
  const driveFlags = Number(drive.flags || 0);
  const balanceArmed = Boolean(driveFlags & (1 << 3));
  const rearFeedbackReady = Boolean(driveFlags & (1 << 4));
  const positionValid = Boolean(driveFlags & (1 << 7));
  const encoderOdometry = Boolean(driveFlags & (1 << 9));
  let odometryState = "等待车辆连接";
  if (uartFresh) {
    if (!rearFeedbackReady) odometryState = "Axis 1 无反馈";
    else if (!positionValid || !encoderOdometry) odometryState = "等待编码器首帧";
    else odometryState = "后轮编码器里程";
  }
  $("#drive-fusion").textContent = odometryState;
  let driveSafetyState = "运动就绪";
  if (!uartFresh) driveSafetyState = "等待车辆连接";
  else if (!balanceArmed) driveSafetyState = "等待平衡启动";
  else if (!rearFeedbackReady) driveSafetyState = "Axis 1 未进入闭环";
  else if (!positionValid || !encoderOdometry) driveSafetyState = "等待编码器首帧";
  else if (driveFlags & (1 << 8)) driveSafetyState = "后轮方向异常";
  else if (driveFlags & (1 << 10)) driveSafetyState = "检测到打滑";
  else if (driveFlags & (1 << 2)) driveSafetyState = "安全状态锁定";
  $("#drive-safety").textContent = driveSafetyState;
  const phaseNames = ["演示未运行", "直行 2 m", "停稳", "右转 15°", "右转等待 5 s", "右转回中", "左转 15°", "左转等待 5 s", "前进 1 m", "后退 1 m", "回中", "倒车 2 m"];
  $("#demo-phase").textContent = phaseNames[Number(drive.demo_phase ?? 0)] || "未知阶段";
  $("#demo-cycles").textContent = `${Number(drive.demo_cycle_count || 0)} 组`;
  const driveFresh = status.uart_connected && status.uart_age_ms < 1200;
  const demoActive = driveFresh ? driveMode === 2 : status.drive_mode === "demo";
  $("#drive-demo").textContent = demoActive ? "停止循环演示" : "开始循环演示";
  $("#drive-demo").classList.toggle("active", demoActive);
  const telemetryButton = $("#telemetry-button");
  telemetryButton.textContent = detailed ? "关闭详细遥测" : "开启详细遥测";
  telemetryButton.classList.toggle("active", detailed);
  const rec = $("#record-button"); rec.textContent = status.recording ? "停止记录" : "开始记录"; rec.classList.toggle("active", status.recording);
  const times = state.sampleTimes.filter(t => t >= (state.sampleTimes.at(-1) || 0) - 3); $("#sample-rate").textContent = detailed && times.length > 1 ? `${((times.length - 1)/(times.at(-1)-times[0])).toFixed(1)} Hz` : "基础 2 Hz";
}

function setPill(selector, text, className) { const el = $(selector); el.childNodes[1].textContent = text; el.className = `status-pill ${className}`; }
function signed(value, digits) { if (value === undefined || value === null || value === "") return "--"; const n = Number(value); return Number.isFinite(n) ? `${n >= 0 ? "+" : ""}${n.toFixed(digits)}` : "--"; }
function renderFlags(flags) { const root = $("#flag-list"); root.innerHTML = ""; const visible = flags.length ? flags : ["NO FLAGS"]; for (const name of visible) { const span = document.createElement("span"); span.className = `flag ${dangerousFlags.has(name) ? "alert" : ["CONTROL_ARMED","IMU_READY","ODRIVE_READY"].includes(name) ? "good" : ""}`; span.textContent = name; root.appendChild(span); } }
function renderHardware(o) {
  const values = [
    ["Axis 0 状态", o.axis_state === 8 ? "8 · CLOSED LOOP" : String(o.axis_state ?? "--")],
    ["Axis 0 错误", o.axis_error || "--"],
    ["Axis 1 状态", o.axis1_state === 8 ? "8 · CLOSED LOOP" : String(o.axis1_state ?? "--")],
    ["Axis 1 错误", o.axis1_error || "--"],
    ["Axis 1 编码器", o.axis1_encoder_error || "--"],
    ["电流限制", `${fmt(o.current_lim_a,1)} A`],
    ["加速度斜坡", `${fmt(o.vel_ramp_tps2,1)} tps²`],
    ["速度限制", `${fmt(o.vel_limit_tps,1)} tps`],
  ];
  const root = $("#hardware-list"); root.innerHTML = values.map(([k,v]) => `<div><dt>${k}</dt><dd>${v}</dd></div>`).join("");
}

function renderEvents() {
  const root = $("#event-list"); root.innerHTML = "";
  for (const event of [...state.events].reverse().slice(0, 40)) { const li = document.createElement("li"); li.className = event.level; const date = new Date(event.time * 1000); li.innerHTML = `<time>${date.toLocaleTimeString("zh-CN", {hour12:false})}</time><span class="level">${event.level}</span><span></span>`; li.lastElementChild.textContent = event.message; root.appendChild(li); }
}

function renderAnalysis(analysis) {
  if (!analysis || !Object.keys(analysis).length) return;
  state.sessionAnalysis = analysis;
  const panel = $("#analysis-panel"); panel.hidden = false;
  const hasScore = Boolean(analysis.disturbance_detected);
  const badge = $("#quality-badge"); badge.className = `quality-badge ${analysis.quality_key || "unknown"}${hasScore ? "" : " no-score"}`;
  $("#quality-text").textContent = analysis.quality || "无法评估";
  $("#quality-score").textContent = hasScore ? fmt(analysis.score, 0) : "--";
  $("#assessment-duration").textContent = fmt(analysis.duration_s, 2);
  $("#assessment-peak").textContent = signed(analysis.peak_signed_deg, 2);
  $("#assessment-overshoot").textContent = Number.isFinite(Number(analysis.overshoot_ratio)) ? fmt(Number(analysis.overshoot_ratio) * 100, 0) : "--";
  $("#assessment-settle-03").textContent = analysis.settle_03_s == null ? "未恢复" : fmt(analysis.settle_03_s, 2);
  $("#assessment-settle-02").textContent = analysis.settle_02_s == null ? "未恢复" : fmt(analysis.settle_02_s, 2);
  $("#assessment-settle-03-unit").hidden = analysis.settle_03_s == null;
  $("#assessment-settle-02-unit").hidden = analysis.settle_02_s == null;
  $("#assessment-rate").textContent = fmt(analysis.roll_rate_peak_dps, 2);
  $("#assessment-current").textContent = fmt(analysis.current_peak_a, 1);
  $("#assessment-saturation").textContent = fmt(analysis.accel_sat_pct, 1);
  $("#assessment-boost").textContent = fmt(analysis.accel_boost_120_pct, 1);
  $("#assessment-envelope").textContent = fmt(analysis.speed_envelope_pct, 1);
  $("#assessment-torque-control").textContent = fmt(analysis.torque_control_pct, 1);

  const peaks = $("#peak-sequence"); peaks.innerHTML = "";
  const sequence = analysis.peak_sequence || [];
  if (!sequence.length) peaks.textContent = "没有可辨识的衰减峰";
  for (const peak of sequence) {
    const chip = document.createElement("span"); chip.className = "peak-chip";
    chip.textContent = `${fmt(peak.time_s, 2)}s  ${signed(peak.value_deg, 2)}°`;
    peaks.appendChild(chip);
  }

  const flags = $("#analysis-flags"); flags.innerHTML = "";
  const triggered = analysis.triggered_flags || [];
  const visibleFlags = triggered.length ? triggered : ["无危险状态"];
  for (const name of visibleFlags) {
    const item = document.createElement("span");
    item.className = `flag ${dangerousFlags.has(name) ? "alert" : "good"}`;
    item.textContent = name; flags.appendChild(item);
  }

}

function showLiveView() {
  state.viewMode = "live";
  $("#view-mode-label").textContent = "实时曲线";
  $("#return-live").hidden = true;
  $("#window-select").disabled = false;
  renderCharts();
}

function showSessionView(samples, analysis) {
  state.sessionSamples = samples || [];
  state.viewMode = "session";
  $("#view-mode-label").textContent = "已记录区间 · 冻结视图";
  $("#return-live").hidden = false;
  $("#window-select").disabled = true;
  renderAnalysis(analysis);
  renderCharts();
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  $("#analysis-panel").scrollIntoView({ behavior: reducedMotion ? "auto" : "smooth", block: "start" });
}

function scheduleRender() { if (state.paused || state.renderPending) return; state.renderPending = true; requestAnimationFrame(() => { state.renderPending = false; renderCharts(); }); }
function renderCharts() {
  const sessionMode = state.viewMode === "session";
  const samples = sessionMode ? state.sessionSamples : state.samples;
  if (!samples.length) return;
  const latest = samples.at(-1), b = latest.balance || {}, o = latest.odrive || {};
  const sessionDuration = Math.max(.1, Number(state.sessionAnalysis?.duration_s) || (latest.host_time_s - samples[0].host_time_s));
  charts.forEach(chart => chart.render(samples, sessionMode ? sessionDuration : state.windowSeconds, sessionMode));
  $("#chart-roll-value").textContent = `${signed(b.roll_error_deg,3)}°`;
  $("#chart-rate-value").textContent = `${signed(b.roll_rate_dps,2)}°/s`;
  $("#chart-wheel-value").textContent = `${signed(b.wheel_tps,2)} tps`;
  $("#chart-accel-value").textContent = `${signed(b.accel_cmd_tps2,1)} tps²`;
  $("#chart-pid-value").textContent = `P ${signed(b.rate_p_tps,1)} · D ${signed(b.rate_d_tps,1)}`;
  $("#chart-current-value").textContent = `${signed(o.iq_measured_a,1)} A`;
}

async function poll() {
  try { const [sampleData, status, eventData] = await Promise.all([api(`/api/samples?after=${state.lastSampleId}&limit=500`), api("/api/status"), api(`/api/events?after=${state.lastEventId}`)]); ingestSamples(sampleData.samples); renderStatus(status); ingestEvents(eventData.events); }
  catch (error) { setPill("#uart-status", "服务断开", "danger"); }
  setTimeout(poll, 150);
}

async function refreshPorts() {
  if (state.portPickerBusy) return;
  try {
    const data = await api("/api/ports");
    const select = $("#serial-port");
    const current = select.value;
    const ports = data.ports || [];
    select.replaceChildren();
    const empty = new Option("选择设备", ""); select.add(empty);
    for (const port of ports) select.add(new Option(`${port.device} · ${port.description}`, port.device));
    select.value = data.selected || current || "";
  } catch (_) {}
}

async function init() {
  try {
    const data = await api("/api/bootstrap"); state.apiToken = data.api_token || ""; state.groups = data.parameter_groups; makePidControls(); ingestSamples(data.samples); ingestEvents(data.events); renderStatus(data.status); updateParams(data.status.params || {}); if (Object.keys(data.status.latest_analysis || {}).length) renderAnalysis(data.status.latest_analysis); renderCharts(); await refreshPorts(); setInterval(refreshPorts, 5000); poll();
  } catch (error) { showToast(`初始化失败：${error.message}`, true); }
}

$("#pause-button").addEventListener("click", () => { state.paused = !state.paused; $("#pause-button").textContent = state.paused ? "继续曲线" : "暂停曲线"; if (!state.paused) renderCharts(); });
$("#window-select").addEventListener("change", event => { state.windowSeconds = Number(event.target.value); showLiveView(); });
$("#return-live").addEventListener("click", showLiveView);
$("#refresh-params").addEventListener("click", async () => { try { const r = await post("/api/tune", {action:"get"}); setFeedback(r.reply, true); } catch (e) { setFeedback(e.message, false); } });
$("#apply-all").addEventListener("click", applyAll);
$("#revert-button").addEventListener("click", async () => { if (!confirm("恢复全部 PID 为固件编译默认值？该操作会立即影响控制。")) return; try { const r = await post("/api/tune", {action:"revert"}); state.dirty.clear(); document.querySelectorAll(".pid-input").forEach(el => el.classList.remove("dirty")); setFeedback(r.reply, true); } catch(e) { setFeedback(e.message, false); } });
document.querySelectorAll("[data-steer-angle]").forEach(button => button.addEventListener("click", () => commandSteering(Number(button.dataset.steerAngle))));
$("#steering-slider").addEventListener("input", () => {
  updateSteeringSliderLabel();
  commandSteering(steeringIndexToAngle($("#steering-slider").value), true);
});
$("#drive-speed-slider").addEventListener("input", event => { $("#drive-speed-value").textContent = `${Number(event.target.value).toFixed(2)} m/s`; });
$("#drive-forward").addEventListener("click", () => commandDrive("manual"));
$("#drive-reverse").addEventListener("click", () => commandDrive("reverse"));
$("#drive-stop").addEventListener("click", () => commandDrive("stop"));
$("#drive-demo").addEventListener("click", () => commandDrive((Number(state.status?.drive?.mode ?? 0) === 2 || state.status?.drive_mode === "demo") ? "stop" : "demo"));
$("#telemetry-button").addEventListener("click", () => { setTelemetryMode(state.status?.telemetry_mode === "live" ? "basic" : "live").catch(() => {}); });
$("#odrive-recover").addEventListener("click", recoverOdrive);
$("#serial-port").addEventListener("change", async event => {
  state.portPickerBusy = true;
  try { await post("/api/port", { port: event.target.value }); showToast(event.target.value ? "串口已选择，正在连接" : "串口已断开"); }
  catch (error) { showToast(error.message, true); }
  finally { state.portPickerBusy = false; await refreshPorts(); }
});
$("#mark-button").addEventListener("click", async () => { await post("/api/mark", {label:"人工扰动"}); showToast("已标记扰动时刻"); });
$("#record-button").addEventListener("click", async () => {
  const button = $("#record-button"); const action = state.status?.recording ? "stop" : "start";
  button.disabled = true;
  try {
    if (action === "start" && state.status?.telemetry_mode !== "live") await setTelemetryMode("live");
    const result = await post("/api/record", { action });
    if (action === "start") {
      state.status.recording = true; showLiveView(); $("#analysis-panel").hidden = true;
      showToast("记录已开始");
    } else {
      state.status.recording = false; showSessionView(result.samples, result.analysis);
      showToast(`评估完成：${result.analysis?.quality || "数据不足"}`);
      await setTelemetryMode("basic");
    }
    renderStatus(state.status);
  } catch (error) { showToast(error.message, true); }
  finally { button.disabled = false; }
});
$("#export-button").addEventListener("click", () => { window.location.href = `/api/export?seconds=${state.windowSeconds}`; });
$("#clear-events").addEventListener("click", () => { state.events = []; renderEvents(); });
window.addEventListener("resize", () => scheduleRender());
init();
