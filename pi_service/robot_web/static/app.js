const $ = selector => document.querySelector(selector);
const video = $("#video"), save = $("#save");
const auxVideo = $("#auxVideo"), auxContext = auxVideo.getContext("2d", {alpha:false});
let folder, aborter, count = 0, profiles = {}, activeMode = "all", configLoaded = false, keysSending = false, keysQueued = false;
let cameraModeDirty = false, cameraModeBusy = false, streamProfileDirty = false, streamProfileBusy = false, exposureDirty = false, exposureBusy = false, videoRetryTimer;
let servoTimer = null, servoBusy = false;
let receivedFrameCount = 0, receivedFrameWindowAt = performance.now(), browserReceiveFps = 0, statusRttMs = null;
const wheelNames = ["rf", "lf", "lr", "rr"];
const actionKeys = {FL:"q", F:"w", FR:"e", PL:"a", PR:"d", BL:"z", B:"s", BR:"c"};
const keyboardKeys = {w:"w", a:"a", s:"s", d:"d", q:"q", e:"e", z:"z", c:"c", ArrowUp:"w", ArrowDown:"s", ArrowLeft:"a", ArrowRight:"d"};
const heldKeys = new Set();

async function requestJson(url, options = {}, timeoutMs = 500) {
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), timeoutMs);
  try { return await fetch(url, {...options, signal:abort.signal}); } finally { clearTimeout(timer); }
}

const note = text => { save.textContent = text; };
const fileName = () => `robot_${new Date().toISOString().replace(/[:.]/g, "-")}_${++count}.jpg`;
function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
function drawAuxiliaryFrame() {
  const sourceWidth = video.naturalWidth, sourceHeight = video.naturalHeight;
  if (sourceWidth && sourceHeight && video.complete) {
    let sx = 0, sy = 0, cropWidth = sourceWidth, cropHeight = sourceHeight;
    if ($("#auxCropMode").value === "center") {
      cropWidth = clamp(Number($("#auxCropWidth").value) || 640, 1, sourceWidth);
      cropHeight = clamp(Number($("#auxCropHeight").value) || 480, 1, sourceHeight);
      sx = (sourceWidth - cropWidth) / 2;
      sy = (sourceHeight - cropHeight) / 2;
    }
    auxContext.drawImage(video, sx, sy, cropWidth, cropHeight, 0, 0, auxVideo.width, auxVideo.height);
  }
  requestAnimationFrame(drawAuxiliaryFrame);
}
$("#auxCropMode").onchange = () => {
  const enabled = $("#auxCropMode").value === "center";
  $("#auxCropWidth").disabled = !enabled; $("#auxCropHeight").disabled = !enabled;
};
$("#auxCropMode").dispatchEvent(new Event("change"));
drawAuxiliaryFrame();
async function chooseFolder() {
  if (!isSecureContext || !window.showDirectoryPicker) throw Error("请用 Chrome/Edge 通过 HTTPS 或 localhost 打开。");
  folder = await window.showDirectoryPicker({mode:"readwrite"});
  if (await folder.requestPermission({mode:"readwrite"}) !== "granted") throw Error("未获得文件夹权限");
  note(`保存目标：${folder.name}`);
}
async function saveBlob(blob) {
  if (!folder) await chooseFolder();
  const handle = await folder.getFileHandle(fileName(), {create:true});
  const writer = await handle.createWritable(); await writer.write(blob); await writer.close();
}
async function saveFrame() {
  const canvas = document.createElement("canvas"); canvas.width = video.naturalWidth; canvas.height = video.naturalHeight;
  canvas.getContext("2d").drawImage(video, 0, 0);
  const blob = await new Promise(resolve => canvas.toBlob(resolve, "image/jpeg", .9));
  if (!blob) throw Error("视频尚未就绪"); await saveBlob(blob); note(`已保存 ${count} 张到电脑`);
}
$("#choose").onclick = () => chooseFolder().catch(error => note(error.message));
$("#snapshot").onclick = () => saveFrame().catch(error => note(error.message));
$("#record").onclick = async () => {
  try { if (!folder) await chooseFolder(); aborter = new AbortController(); $("#record").disabled = true; $("#stopRecord").disabled = false;
    while (!aborter.signal.aborted) { await saveFrame(); await new Promise(resolve => setTimeout(resolve, 200)); }
  } catch (error) { note(error.message); } finally { $("#record").disabled = false; $("#stopRecord").disabled = true; }
};
$("#stopRecord").onclick = () => aborter?.abort();

function reloadVideo() {
  clearTimeout(videoRetryTimer);
  videoRetryTimer = setTimeout(() => { video.src = `/video_feed?ts=${Date.now()}`; }, 800);
}
video.addEventListener("error", reloadVideo);
video.addEventListener("load", () => {
  const now = performance.now();
  receivedFrameCount += 1;
  const elapsed = now - receivedFrameWindowAt;
  if (elapsed >= 1000) {
    browserReceiveFps = receivedFrameCount * 1000 / elapsed;
    receivedFrameCount = 0;
    receivedFrameWindowAt = now;
  }
});
$("#cameraMode").onchange = () => { cameraModeDirty = true; };
$("#applyCameraMode").onclick = async () => {
  if (cameraModeBusy) return;
  cameraModeBusy = true;
  const button = $("#applyCameraMode"), select = $("#cameraMode");
  button.disabled = true; select.disabled = true; note("正在切换相机分辨率，视频流会短暂重连……");
  try {
    const response = await fetch("/api/camera/mode", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({mode:select.value})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "相机模式切换失败");
    cameraModeDirty = false; note(`相机已切换到 ${data.camera.mode_label}，传感器目标 ${fixed(data.camera.sensor_target_fps)} FPS`); reloadVideo();
  } catch (error) { note(error.message); }
  finally { cameraModeBusy = false; button.disabled = false; select.disabled = false; }
};

$("#streamProfile").onchange = () => { streamProfileDirty = true; };
$("#applyStreamProfile").onclick = async () => {
  if (streamProfileBusy) return;
  streamProfileBusy = true;
  const button = $("#applyStreamProfile"), select = $("#streamProfile");
  button.disabled = true; select.disabled = true;
  try {
    const response = await fetch("/api/camera/stream-profile", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({profile:select.value})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "视频传输档位切换失败");
    streamProfileDirty = false;
    note(`已应用 ${data.camera.stream_profile.label}；采集不重启，网页将在下一帧使用新压缩画面。`);
  } catch (error) { note(error.message); }
  finally { streamProfileBusy = false; button.disabled = false; select.disabled = false; }
};

function updateExposureUi() {
  const automatic = $("#exposureMode").value === "auto";
  $("#cameraEv").disabled = !automatic;
  $("#shutterDenominator").disabled = automatic;
  $("#exposureHint").textContent = automatic
    ? "自动模式下 EV 调整亮度；快门由相机自动决定。"
    : "固定快门使用 1/xx 秒；此时自动 EV 不参与曝光。";
}
for (const input of [$("#exposureMode"), $("#cameraEv"), $("#shutterDenominator")]) {
  input.addEventListener("input", () => { exposureDirty = true; updateExposureUi(); });
  input.addEventListener("change", () => { exposureDirty = true; updateExposureUi(); });
}
updateExposureUi();
$("#applyExposure").onclick = async () => {
  if (exposureBusy) return;
  exposureBusy = true;
  const button = $("#applyExposure"), mode = $("#exposureMode"), ev = $("#cameraEv"), shutter = $("#shutterDenominator");
  button.disabled = true; mode.disabled = true; ev.disabled = true; shutter.disabled = true;
  try {
    const response = await fetch("/api/camera/exposure", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({
      auto: mode.value === "auto", ev:Number(ev.value), shutter_denominator:Number(shutter.value),
    })});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "曝光设置失败");
    exposureDirty = false;
    note(data.camera.exposure.auto ? `已应用自动曝光：EV ${fixed(data.camera.exposure.ev)}` : `已固定快门：1/${data.camera.exposure.shutter_denominator} 秒`);
  } catch (error) { note(error.message); }
  finally { exposureBusy = false; button.disabled = false; mode.disabled = false; updateExposureUi(); }
};

function editing(event) { return ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName); }
async function sendKeys() {
  if (keysSending) { keysQueued = true; return; }
  keysSending = true;
  do { keysQueued = false; try { const response = await requestJson("/api/keys", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({keys:[...heldKeys]}), keepalive:true});
      const data = await response.json(); if (!response.ok) throw Error(data.error || "动作发送失败"); $("#action").textContent = data.action;
    } catch (error) { note(error.message); }
  } while (keysQueued);
  keysSending = false;
}
function setKey(key, pressed) { if (pressed) heldKeys.add(key); else heldKeys.delete(key); sendKeys(); }
function releaseKeys() { heldKeys.clear(); sendKeys(); }
for (const button of document.querySelectorAll("[data-action]")) {
  const key = actionKeys[button.dataset.action];
  if (!key) { button.onclick = releaseKeys; continue; }
  button.addEventListener("pointerdown", event => { event.preventDefault(); button.setPointerCapture(event.pointerId); setKey(key, true); });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(name, event => { event.preventDefault(); setKey(key, false); });
}
addEventListener("keydown", event => { if (editing(event) || event.repeat) return; if (event.code === "Space") { event.preventDefault(); releaseKeys(); return; }
  const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, true); }
});
addEventListener("keyup", event => { if (editing(event)) return; const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, false); } });
addEventListener("blur", releaseKeys); addEventListener("beforeunload", () => navigator.sendBeacon("/api/stop")); setInterval(sendKeys, 180);
async function sendHeartbeat() { try { await requestJson("/api/heartbeat", {method:"POST", keepalive:true}, 500); } catch (_) {} }
setInterval(sendHeartbeat, 180);
$("#stopButton").onclick = releaseKeys;

// Send only after the slider has paused briefly. This keeps camera pan
// responsive without flooding the 9600-baud Arduino serial link.
$("#servoSlider").addEventListener("input", () => {
  const angle = Number($("#servoSlider").value);
  $("#servoAngleDisplay").textContent = `${angle}°`;
  clearTimeout(servoTimer);
  servoTimer = setTimeout(async () => {
    servoBusy = true;
    try {
      const response = await fetch("/api/servo", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({angle})});
      const data = await response.json();
      if (!response.ok || !data.ok) throw Error(data.error || "舵机指令失败");
    } catch (error) { note(error.message); }
    finally { servoBusy = false; }
  }, 80);
});

function profileFor(action) { return profiles[action] || {rf:0, lf:0, lr:0, rr:0}; }
function signedMagnitude(value, sign) { return Math.round(Math.abs(Number(value) || 0)) * (sign < 0 ? -1 : 1); }
function fillProfileEditor() {
  const p = profileFor($("#profileAction").value), abs = wheel => Math.abs(Number(p[wheel]) || 0);
  $("#allValue").value = Math.max(...wheelNames.map(abs));
  $("#leftValue").value = Math.round((abs("lf") + abs("lr")) / 2); $("#rightValue").value = Math.round((abs("rf") + abs("rr")) / 2);
  for (const wheel of wheelNames) $(`#${wheel}Value`).value = p[wheel]; updateProfilePreview(p);
}
function updateProfilePreview(p = profileFor($("#profileAction").value)) { $("#profilePreview").textContent = `M1 ${p.rf} · M2 ${p.lf} · M3 ${p.lr} · M4 ${p.rr}`; }
function profileFromEditor() {
  const p = {...profileFor($("#profileAction").value)};
  if (activeMode === "all") for (const wheel of wheelNames) p[wheel] = signedMagnitude($("#allValue").value, p[wheel] || 1);
  if (activeMode === "sides") { for (const wheel of ["lf", "lr"]) p[wheel] = signedMagnitude($("#leftValue").value, p[wheel] || 1); for (const wheel of ["rf", "rr"]) p[wheel] = signedMagnitude($("#rightValue").value, p[wheel] || 1); }
  if (activeMode === "wheels") for (const wheel of wheelNames) p[wheel] = Math.max(-255, Math.min(255, Number($(`#${wheel}Value`).value) || 0));
  return p;
}
function refreshProfilePreview() { updateProfilePreview(profileFromEditor()); }
for (const input of document.querySelectorAll("#allValue,#leftValue,#rightValue,#rfValue,#lfValue,#lrValue,#rrValue")) input.addEventListener("input", refreshProfilePreview);
$("#profileAction").onchange = fillProfileEditor;
for (const tab of document.querySelectorAll(".mode")) tab.onclick = () => { activeMode = tab.dataset.mode; document.querySelectorAll(".mode").forEach(item => item.classList.toggle("active", item === tab)); $("#allEditor").classList.toggle("hidden", activeMode !== "all"); $("#sidesEditor").classList.toggle("hidden", activeMode !== "sides"); $("#wheelsEditor").classList.toggle("hidden", activeMode !== "wheels"); refreshProfilePreview(); };
$("#applyProfile").onclick = async () => { profiles[$("#profileAction").value] = profileFromEditor();
  try { const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({profiles})}); const data = await response.json(); if (!response.ok) throw Error(data.error || "轮速配置失败"); profiles = data.config.profiles; fillProfileEditor(); note("轮速配置已应用并保存"); } catch (error) { note(error.message); }
};

function fillConfig(config) {
  profiles = config.profiles || profiles; $("#speedMode").checked = !!config.speed_mode; $("#targetSpeed").value = config.target_speed; $("#kp").value = config.kp; $("#ki").value = config.ki; $("#kd").value = config.kd; fillProfileEditor();
}
$("#applyPid").onclick = async () => { const payload = {speed_mode:$("#speedMode").checked, target_speed:Number($("#targetSpeed").value), kp:Number($("#kp").value), ki:Number($("#ki").value), kd:Number($("#kd").value)};
  try { const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)}); if (!response.ok) throw Error("PID 参数提交失败"); note("PID 参数已应用并保存"); } catch (error) { note(error.message); }
};
$("#reconnect").onclick = async () => { const button = $("#reconnect"); button.disabled = true; note("正在重新连接 Arduino……"); try { const response = await fetch("/api/reconnect", {method:"POST"}), data = await response.json(); if (!response.ok || !data.ok) throw Error(data.robot?.error || "Arduino 未收到回包"); note("Arduino 已重新连接并收到回包"); } catch (error) { note(`重新连接失败：${error.message}`); } finally { button.disabled = false; } };

function dot(online, text, warning = false) { return `<i class="dot ${online ? "online" : warning ? "warn" : "offline"}"></i>${text}`; }
function fixed(value, digits = 1) { const number = Number(value); return Number.isFinite(number) ? number.toFixed(digits) : "—"; }
function bytes(value) { const number = Number(value); if (!Number.isFinite(number)) return "—"; return number >= 1e9 ? `${fixed(number / 1e9)} GB` : `${fixed(number / 1e6)} MB`; }
function duration(seconds) { const whole = Math.max(0, Math.floor(Number(seconds) || 0)); return `${Math.floor(whole / 60)} 分 ${whole % 60} 秒`; }
function distance(value) {
  if (value == null) return "等待传感器数据…";
  const front = Number(value);
  if (!Number.isFinite(front) || front < 0) return "无有效回波（-1）";
  return `${front.toFixed(1)} cm`;
}
function streamDiagnosis(camera) {
  if (!camera.online) return ["树莓派相机服务离线", "请先处理相机状态或服务错误；网络判断暂不可用。"];
  const target = Number(camera.target_fps) || Number(camera.sensor_target_fps) || 0;
  const capture = Number(camera.capture_fps) || 0;
  const encode = Number(camera.encode_ms) || 0;
  const frameBudget = target > 0 ? 1000 / target : Infinity;
  if (target > 0 && capture < target * 0.7) {
    return ["更像树莓派采集/编码瓶颈", `相机只编码 ${fixed(capture)} FPS，低于目标 ${fixed(target)} FPS；先检查相机模式、CPU 占用和编码耗时。`];
  }
  if (encode > frameBudget * 0.7) {
    return ["更像树莓派 JPEG 编码压力", `单帧编码 ${fixed(encode)} ms，接近 ${fixed(frameBudget)} ms 的帧预算；降低分辨率或 JPEG 质量再比较。`];
  }
  if (statusRttMs != null && statusRttMs > 120) {
    return ["更像网络往返延迟", `状态请求 RTT ${fixed(statusRttMs)} ms 偏高；检查 Wi-Fi 信号、同网段/中继和其他网络占用。`];
  }
  if (browserReceiveFps > 0 && capture > 0 && browserReceiveFps < capture * 0.7) {
    const reason = statusRttMs != null && statusRttMs > 50 ? "网络传输或浏览器解码" : "浏览器解码/渲染";
    return [`更像${reason}瓶颈`, `树莓派编码 ${fixed(capture)} FPS，但本浏览器只收到 ${fixed(browserReceiveFps)} FPS；尝试关闭其他视频客户端、降低分辨率后复测。`];
  }
  return ["未发现明显传输瓶颈", `树莓派编码 ${fixed(capture)} FPS、本浏览器收到 ${fixed(browserReceiveFps)} FPS、RTT ${fixed(statusRttMs)} ms；若操作仍卡，重点检查控制请求和浏览器负载。`];
}
async function refreshStatus() { try { const statusStartedAt = performance.now(); const response = await fetch("/api/status", {cache:"no-store"}), data = await response.json(), robot = data.robot, system = data.system || {}; statusRttMs = performance.now() - statusStartedAt;
    const camera = data.camera || {};
    $("#cameraState").innerHTML = dot(camera.online, camera.online ? "在线" : camera.status); $("#arduinoState").innerHTML = dot(robot.arduino_online, robot.arduino_online ? "在线" : robot.serial ? "无响应" : "离线", robot.serial);
    const resolution = camera.resolution || (camera.width && camera.height ? `${camera.width}×${camera.height}` : "—");
    if (!cameraModeDirty && !cameraModeBusy && camera.mode) $("#cameraMode").value = camera.mode;
    if (!streamProfileDirty && !streamProfileBusy && camera.stream_profile?.key) $("#streamProfile").value = camera.stream_profile.key;
    if (camera.width && camera.height) { $("#auxCropWidth").max = camera.width; $("#auxCropHeight").max = camera.height; }
    if (!exposureDirty && !exposureBusy && camera.exposure) {
      $("#exposureMode").value = camera.exposure.auto ? "auto" : "manual";
      $("#cameraEv").value = camera.exposure.ev;
      $("#shutterDenominator").value = camera.exposure.shutter_denominator;
      updateExposureUi();
    }
    const streamResolution = camera.stream_profile?.resolution || resolution;
    $("#cameraMeta").textContent = `采集 ${resolution} → 网页 ${streamResolution} · 传感器目标 ${fixed(camera.sensor_target_fps)} FPS · 实际编码 ${fixed(camera.capture_fps)} FPS`;
    $("#cameraResolution").textContent = `${resolution} → ${streamResolution}`;
    $("#streamProfileState").textContent = camera.stream_profile?.label || "—";
    $("#cameraFps").textContent = `${fixed(camera.capture_fps)} / ${fixed(camera.stream_fps)} FPS`;
    $("#cameraBandwidth").textContent = `${fixed(camera.stream_kBps)} kB/s · ${fixed(camera.stream_kbps)} kbps`;
    $("#cameraFrameSize").textContent = camera.jpeg_bytes ? `${fixed(camera.jpeg_bytes / 1000)} KB` : "—";
    $("#cameraEncode").textContent = `${fixed(camera.encode_ms)} ms（平均 ${fixed(camera.encode_ms_avg)} ms）`;
    $("#cameraAge").textContent = camera.frame_age_ms == null ? "—" : `${fixed(camera.frame_age_ms)} ms`;
    $("#cameraClients").textContent = `${camera.active_clients ?? 0}`;
    $("#browserReceiveFps").textContent = browserReceiveFps > 0 ? `${fixed(browserReceiveFps)} FPS` : "测量中…";
    $("#statusRtt").textContent = statusRttMs == null ? "测量中…" : `${fixed(statusRttMs)} ms`;
    const [diagnosis, diagnosisDetail] = streamDiagnosis(camera);
    $("#streamDiagnosis").textContent = diagnosis;
    $("#streamDiagnosisDetail").textContent = diagnosisDetail;
    $("#systemCpu").textContent = system.cpu_percent == null ? "测量中…" : `${fixed(system.cpu_percent)}%`;
    $("#systemLoad").textContent = fixed(system.load_1m, 2);
    $("#systemMemory").textContent = system.memory_total_bytes == null ? "—" : `${bytes(system.memory_used_bytes)} / ${bytes(system.memory_total_bytes)}`;
    $("#systemTemperature").textContent = system.cpu_temperature_c == null ? "—" : `${fixed(system.cpu_temperature_c)} °C`;
    $("#systemDisk").textContent = system.disk_total_bytes == null ? "—" : `${bytes(system.disk_used_bytes)} / ${bytes(system.disk_total_bytes)}`;
    $("#systemUptime").textContent = duration(system.uptime_seconds);
    $("#controlState").innerHTML = dot(robot.client_online, robot.client_online ? "在线" : "已超时停车"); $("#action").textContent = robot.action; $("#keys").textContent = robot.keys?.join("+") || "—"; $("#distance").textContent = distance(robot.ultrasonic); $("#lastReply").textContent = robot.reply || "等待 Arduino 回包";
    if (!servoBusy && robot.servo_angle != null) { $("#servoSlider").value = robot.servo_angle; $("#servoAngleDisplay").textContent = `${robot.servo_angle}°`; }
    if (robot.imu) { $("#roll").textContent = `${robot.imu[0].toFixed(2)}°`; $("#pitch").textContent = `${robot.imu[1].toFixed(2)}°`; $("#yaw").textContent = `${robot.imu[2].toFixed(2)}°`; }
    if (robot.speed) { $("#wheelSpeed").textContent = `${robot.speed[0].toFixed(1)} / ${robot.speed[1].toFixed(1)} pps`; $("#targetWheelSpeed").textContent = `${robot.speed[2].toFixed(1)} / ${robot.speed[3].toFixed(1)} pps`; }
    if (!configLoaded) { fillConfig(robot.config); configLoaded = true; }
    const age = robot.last_rx_age == null ? "—" : `${robot.last_rx_age.toFixed(2)} s`;
    $("#status").textContent = [`Arduino: ${robot.arduino_online ? "在线" : robot.serial ? "无响应" : "离线"}`, `串口: ${robot.serial ? "已打开" : "未打开"}`, `最近回包: ${age}`, `动作: ${robot.action}`, `按键: ${robot.keys?.join("+") || "—"}`, `回复: ${robot.reply || "—"}`, `错误: ${robot.error || "—"}`].join("\n");
  } catch (error) { $("#status").textContent = `网页后端连接失败：${error}`; $("#arduinoState").innerHTML = dot(false, "网页服务异常"); }
}
refreshStatus(); setInterval(refreshStatus, 500);
