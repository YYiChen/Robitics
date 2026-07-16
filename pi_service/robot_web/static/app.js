const $ = selector => document.querySelector(selector);
const video = $("#video"), webrtcVideo = $("#webrtcVideo"), highresVideo = $("#highresVideo"), save = $("#save");
const auxVideo = $("#auxVideo"), auxContext = auxVideo.getContext("2d", {alpha:false});
let folder, aborter, count = 0, profiles = {}, activeMode = "all", configLoaded = false, keysSending = false, keysQueued = false;
let cameraModeDirty = false, cameraModeBusy = false, streamProfileDirty = false, streamProfileBusy = false, highresProfileDirty = false, highresProfileBusy = false, highresFpsDirty = false, highresFpsBusy = false, exposureDirty = false, exposureBusy = false, videoRetryTimer;
let servoBusy = false, queuedServoAngle = null, steeringCenterAngle = 90, steeringReversed = true;
let receivedFrameCount = 0, receivedFrameWindowAt = performance.now(), browserReceiveFps = 0, statusRttMs = null;
let activeVideoTransport = "mjpeg", currentWebrtcUrl = "";
let highresPreviewEnabled = false, highresPreviewAvailable = false;
let webrtcPeer = null, webrtcSessionUrl = "", webrtcStatsTimer = null, webrtcStatsPrevious = null;
const webrtcMetrics = {state:"未连接", fps:null, kbps:null, jitterMs:null, jitterBufferMs:null, packetsLost:null, framesDropped:null};
const wheelNames = ["rf", "lf", "lr", "rr"];
const actionKeys = {F:"w", SF:"r", PL:"a", PR:"d", B:"s", BR:"c"};
const keyboardKeys = {w:"w", r:"r", a:"a", s:"s", d:"d", c:"c", ArrowUp:"w", ArrowDown:"s", ArrowLeft:"a", ArrowRight:"d"};
const heldKeys = new Set();
const heldSteeringKeys = new Set();

async function requestJson(url, options = {}, timeoutMs = 500) {
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), timeoutMs);
  try { return await fetch(url, {...options, signal:abort.signal}); } finally { clearTimeout(timer); }
}

const note = text => { save.textContent = text; };
const fileName = () => `robot_${new Date().toISOString().replace(/[:.]/g, "-")}_${++count}.jpg`;
const isMjpeg = () => activeVideoTransport === "mjpeg";
function webrtcUrl(camera) {
  const port = Number(camera.webrtc_port) || 8889;
  const path = String(camera.webrtc_path || "cam").replace(/^\/+|\/+$/g, "");
  return `http://${location.hostname}:${port}/${path}/`;
}
function whepUrl(camera) { return `${webrtcUrl(camera).replace(/\/$/, "")}/whep`; }
function setHighresPreview(enabled) {
  highresPreviewEnabled = !!enabled && highresPreviewAvailable;
  const image = $("#highresVideo"), idle = $("#highresPreviewIdle"), toggle = $("#highresPreviewToggle");
  if (highresPreviewEnabled) {
    image.src = `/highres_feed?ts=${Date.now()}`;
    image.classList.remove("hidden"); idle.classList.add("hidden");
    toggle.textContent = "关闭预览";
  } else {
    image.removeAttribute("src"); image.classList.add("hidden");
    idle.classList.toggle("hidden", !highresPreviewAvailable);
    toggle.textContent = "开启预览";
  }
}
$("#highresPreviewToggle").onclick = () => setHighresPreview(!highresPreviewEnabled);
function waitForIceGathering(peer) {
  if (peer.iceGatheringState === "complete") return Promise.resolve();
  return new Promise(resolve => {
    const done = () => { if (peer.iceGatheringState === "complete") { peer.removeEventListener("icegatheringstatechange", done); resolve(); } };
    peer.addEventListener("icegatheringstatechange", done);
  });
}
function stopWebrtc() {
  clearInterval(webrtcStatsTimer); webrtcStatsTimer = null; webrtcStatsPrevious = null;
  const session = webrtcSessionUrl; webrtcSessionUrl = "";
  if (session) fetch(session, {method:"DELETE", keepalive:true}).catch(() => {});
  if (webrtcPeer) { webrtcPeer.close(); webrtcPeer = null; }
  webrtcVideo.srcObject = null;
}
async function refreshWebrtcStats() {
  if (!webrtcPeer) return;
  try {
    const reports = await webrtcPeer.getStats();
    let inbound = null;
    reports.forEach(report => { if (report.type === "inbound-rtp" && report.kind === "video") inbound = report; });
    if (!inbound) return;
    const now = performance.now(), previous = webrtcStatsPrevious;
    if (previous) {
      const seconds = Math.max(.001, (now - previous.at) / 1000);
      webrtcMetrics.kbps = (inbound.bytesReceived - previous.bytes) * 8 / seconds / 1000;
      webrtcMetrics.fps = (inbound.framesDecoded - previous.frames) / seconds;
    }
    webrtcMetrics.jitterMs = Number.isFinite(inbound.jitter) ? inbound.jitter * 1000 : null;
    webrtcMetrics.jitterBufferMs = inbound.jitterBufferEmittedCount ? inbound.jitterBufferDelay / inbound.jitterBufferEmittedCount * 1000 : null;
    webrtcMetrics.packetsLost = Number.isFinite(inbound.packetsLost) ? inbound.packetsLost : null;
    const quality = webrtcVideo.getVideoPlaybackQuality?.();
    webrtcMetrics.framesDropped = quality ? quality.droppedVideoFrames : null;
    webrtcStatsPrevious = {at:now, bytes:inbound.bytesReceived || 0, frames:inbound.framesDecoded || 0};
  } catch (_) {}
}
async function startWebrtc(camera) {
  stopWebrtc();
  const endpoint = whepUrl(camera), peer = new RTCPeerConnection();
  currentWebrtcUrl = endpoint; webrtcPeer = peer; webrtcMetrics.state = "正在建立 WHEP/WebRTC…";
  peer.addTransceiver("video", {direction:"recvonly"});
  peer.ontrack = event => { webrtcVideo.srcObject = event.streams[0]; webrtcVideo.play().catch(() => {}); webrtcMetrics.state = "已连接"; };
  peer.onconnectionstatechange = () => { if (peer === webrtcPeer && peer.connectionState !== "connected") webrtcMetrics.state = `连接状态：${peer.connectionState}`; };
  try {
    await peer.setLocalDescription(await peer.createOffer());
    await waitForIceGathering(peer);
    const response = await fetch(endpoint, {method:"POST", headers:{"Content-Type":"application/sdp", Accept:"application/sdp"}, body:peer.localDescription.sdp});
    if (!response.ok) throw Error(`WHEP 返回 HTTP ${response.status}`);
    const location = response.headers.get("location");
    webrtcSessionUrl = location ? new URL(location, endpoint).toString() : endpoint;
    await peer.setRemoteDescription({type:"answer", sdp:await response.text()});
    webrtcStatsTimer = setInterval(refreshWebrtcStats, 1000); refreshWebrtcStats();
  } catch (error) {
    if (peer === webrtcPeer) { webrtcMetrics.state = `WHEP 连接失败：${error.message}`; stopWebrtc(); }
  }
}
function setVideoTransport(camera) {
  activeVideoTransport = camera.transport || "mjpeg";
  const mjpeg = isMjpeg();
  const highresAvailable = mjpeg || !!camera.highres_available;
  highresPreviewAvailable = highresAvailable;
  for (const id of ["mjpegModeControls", "mjpegProfileControls", "mjpegExposureControls", "mjpegAuxiliary", "mjpegCaptureControls"]) {
    $("#" + id).classList.toggle("hidden", !mjpeg);
  }
  $("#mjpegHighresControls").classList.toggle("hidden", !highresAvailable);
  video.classList.toggle("hidden", !mjpeg);
  webrtcVideo.classList.toggle("hidden", mjpeg);
  highresVideo.classList.toggle("hidden", !highresAvailable || !highresPreviewEnabled);
  $("#highresUnavailable").classList.toggle("hidden", highresAvailable);
  $("#highresPreviewToggle").disabled = !highresAvailable;
  if (!highresAvailable && highresPreviewEnabled) setHighresPreview(false);
  $("#highresPreviewIdle").classList.toggle("hidden", !highresAvailable || highresPreviewEnabled);
  if (!mjpeg) {
    const url = whepUrl(camera);
    if (url !== currentWebrtcUrl || !webrtcPeer) void startWebrtc(camera);
  } else if (webrtcPeer) {
    stopWebrtc(); currentWebrtcUrl = "";
  }
}
function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
function drawAuxiliaryFrame() {
  if (!isMjpeg()) { requestAnimationFrame(drawAuxiliaryFrame); return; }
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
  if (!isMjpeg()) throw Error("WebRTC 预览由独立媒体服务提供，当前浏览器不能直接导出这一帧；请使用 RTSP/DL 端保存。");
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
  if (!isMjpeg()) return;
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
highresVideo.addEventListener("error", () => {
  $("#highresFeedMeta").textContent = "高清流断开";
  $("#highresCard").classList.add("feed-error");
});
highresVideo.addEventListener("load", () => {
  $("#highresCard").classList.remove("feed-error");
});
for (const tab of document.querySelectorAll(".section-tab")) {
  tab.addEventListener("click", () => {
    const panelId = tab.dataset.panel;
    document.querySelectorAll(".section-tab").forEach(item => item.classList.toggle("active", item === tab));
    document.querySelectorAll(".tab-panel").forEach(panel => panel.classList.toggle("active", panel.id === panelId));
  });
}
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

$("#highresProfile").onchange = () => { highresProfileDirty = true; };
$("#applyHighresProfile").onclick = async () => {
  if (highresProfileBusy) return;
  highresProfileBusy = true;
  const button = $("#applyHighresProfile"), select = $("#highresProfile");
  button.disabled = true; select.disabled = true;
  try {
    const response = await fetch("/api/camera/highres-profile", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({profile:select.value})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "高清图片档位切换失败");
    highresProfileDirty = false;
    note(`已应用 ${data.camera.highres_profile.label}；高清 JPEG 将在下一张（最多 0.5 秒）生效。`);
  } catch (error) { note(error.message); }
  finally { highresProfileBusy = false; button.disabled = false; select.disabled = false; }
};
$("#highresFps").onchange = () => { highresFpsDirty = true; };
$("#applyHighresFps").onclick = async () => {
  if (highresFpsBusy) return;
  highresFpsBusy = true;
  const button = $("#applyHighresFps"), input = $("#highresFps");
  button.disabled = true; input.disabled = true;
  try {
    const response = await fetch("/api/camera/highres-fps", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({fps:Number(input.value)})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "高清图片帧率设置失败");
    highresFpsDirty = false;
    input.value = data.camera.highres.target_fps;
    note(`高清 JPEG 已设为 ${fixed(data.camera.highres.target_fps)} FPS；该设置会保存。`);
  } catch (error) { note(error.message); }
  finally { highresFpsBusy = false; button.disabled = false; input.disabled = false; }
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
function steeringDirection() {
  if (heldSteeringKeys.has("q") === heldSteeringKeys.has("e")) return 0;
  return heldSteeringKeys.has("q") ? -1 : 1;
}
async function sendKeys() {
  if (keysSending) { keysQueued = true; return; }
  keysSending = true;
  do { keysQueued = false; try { const response = await requestJson("/api/keys", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({keys:[...heldKeys], steering:steeringDirection()}), keepalive:true});
      const data = await response.json(); if (!response.ok) throw Error(data.error || "动作发送失败"); $("#action").textContent = data.action;
    } catch (error) { note(error.message); }
  } while (keysQueued);
  keysSending = false;
}
function setKey(key, pressed) { if (pressed) heldKeys.add(key); else heldKeys.delete(key); sendKeys(); }
function setSteeringKey(key, pressed) { if (pressed) heldSteeringKeys.add(key); else heldSteeringKeys.delete(key); sendKeys(); }
function releaseKeys() { heldKeys.clear(); heldSteeringKeys.clear(); sendKeys(); }
for (const button of document.querySelectorAll("[data-action]")) {
  const key = actionKeys[button.dataset.action];
  if (!key) { button.onclick = releaseKeys; continue; }
  button.addEventListener("pointerdown", event => { event.preventDefault(); button.setPointerCapture(event.pointerId); setKey(key, true); });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(name, event => { event.preventDefault(); setKey(key, false); });
}
for (const button of document.querySelectorAll("[data-steering]")) {
  const key = button.dataset.steering;
  button.addEventListener("pointerdown", event => { event.preventDefault(); button.setPointerCapture(event.pointerId); setSteeringKey(key, true); });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(name, event => { event.preventDefault(); setSteeringKey(key, false); });
}
for (const button of document.querySelectorAll("[data-servo-center]")) button.addEventListener("click", centerServo);
addEventListener("keydown", event => { if (editing(event) || event.repeat) return; if (event.code === "Space") { event.preventDefault(); releaseKeys(); return; }
  if (event.key?.toLowerCase() === "z") { event.preventDefault(); centerServo(); return; }
  const steering = event.key?.toLowerCase(); if (steering === "q" || steering === "e") { event.preventDefault(); setSteeringKey(steering, true); return; }
  const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, true); }
});
addEventListener("keyup", event => { if (editing(event)) return; const steering = event.key?.toLowerCase(); if (steering === "q" || steering === "e") { event.preventDefault(); setSteeringKey(steering, false); return; } const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, false); } });
addEventListener("blur", releaseKeys); addEventListener("beforeunload", () => navigator.sendBeacon("/api/stop")); setInterval(sendKeys, 180);
async function sendHeartbeat() { try { await requestJson("/api/heartbeat", {method:"POST", keepalive:true}, 500); } catch (_) {} }
setInterval(sendHeartbeat, 180);
$("#stopButton").onclick = releaseKeys;

// Send the first slider update immediately. While a request is in flight, keep
// only the newest angle so a rapid drag cannot build up stale serial commands.
async function flushServoQueue() {
  if (servoBusy) return;
  servoBusy = true;
  try {
    while (queuedServoAngle !== null) {
      const angle = queuedServoAngle;
      queuedServoAngle = null;
      const response = await fetch("/api/servo", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({angle}),
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw Error(data.error || "舵机指令失败");
    }
  } catch (error) {
    queuedServoAngle = null;
    note(error.message);
  } finally {
    servoBusy = false;
    if (queuedServoAngle !== null) void flushServoQueue();
  }
}

$("#servoSlider").addEventListener("input", () => {
  const angle = Number($("#servoSlider").value);
  $("#servoAngleDisplay").textContent = `${angle}°`;
  updateSteeringDial(angle, true);
  queuedServoAngle = angle;
  void flushServoQueue();
});

function updateSteeringDial(angle, known) {
  const numeric = Number(angle);
  const dial = $("#steeringDial"), text = $("#steeringDialText"), cameraGimbal = $("#vehicleCameraGimbal"), turnLabel = $("#vehicleTurnLabel");
  if (!Number.isFinite(numeric) || !known) {
    dial.classList.add("steering-unknown"); text.textContent = "云台状态未知"; turnLabel.textContent = "等待 Arduino 回包"; cameraGimbal.style.transform = "rotate(0deg)"; return;
  }
  // Render the complete physical travel relative to the configured centre.
  // This is intentionally not scaled for visual layout: 0° versus a 90°
  // centre is displayed as a full 90° camera rotation.
  const servoOffset = numeric - steeringCenterAngle;
  const cameraOffset = steeringReversed ? -servoOffset : servoOffset;
  const direction = Math.abs(cameraOffset) < .5 ? "回正" : cameraOffset < 0 ? "左转" : "右转";
  dial.classList.remove("steering-unknown");
  text.textContent = `云台指令 ${Math.round(numeric)}° · ${direction}`;
  cameraGimbal.style.transform = `rotate(${cameraOffset}deg)`;
  turnLabel.textContent = `摄像头${direction} · 相对中位 ${Math.abs(cameraOffset).toFixed(0)}°`;
}

function centerServo() {
  heldSteeringKeys.clear(); sendKeys();
  $("#servoSlider").value = steeringCenterAngle;
  $("#servoSlider").dispatchEvent(new Event("input"));
}
$("#servoCenter").onclick = centerServo;
$("#applyServoSettings").onclick = async () => {
  const payload = {servo_center_angle:Number($("#servoCenterAngle").value), servo_speed_dps:Number($("#servoSpeedDps").value), servo_qe_reversed:$("#servoQeReversed").checked};
  try {
    const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)});
    const data = await response.json(); if (!response.ok || !data.ok) throw Error(data.error || "转向设置保存失败");
    steeringCenterAngle = data.config.servo_center_angle; steeringReversed = !!data.config.servo_qe_reversed;
    $("#servoCenterAngle").value = steeringCenterAngle; $("#servoSpeedDps").value = data.config.servo_speed_dps; $("#servoQeReversed").checked = steeringReversed;
    updateSteeringDial($("#servoSlider").value, true); note("转向设置已保存");
  } catch (error) { note(error.message); }
};

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
  profiles = config.profiles || profiles; $("#speedMode").checked = !!config.speed_mode; $("#targetSpeed").value = config.target_speed; $("#kp").value = config.kp; $("#ki").value = config.ki; $("#kd").value = config.kd;
  steeringCenterAngle = Number(config.servo_center_angle ?? 90); steeringReversed = !!config.servo_qe_reversed;
  $("#servoCenterAngle").value = steeringCenterAngle; $("#servoSpeedDps").value = config.servo_speed_dps ?? 30; $("#servoQeReversed").checked = steeringReversed;
  fillProfileEditor(); updateSteeringDial($("#servoSlider").value, true);
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
  if (camera.transport === "webrtc") {
    if (webrtcMetrics.jitterBufferMs != null && webrtcMetrics.jitterBufferMs > 150) return ["浏览器 WebRTC 缓冲偏高", `当前抖动缓冲约 ${fixed(webrtcMetrics.jitterBufferMs)} ms；请先关闭高清预览，并检查热点信号或降低实时码率。`];
    if (webrtcMetrics.packetsLost != null && webrtcMetrics.packetsLost > 0) return ["WebRTC 存在丢包", `累计丢包 ${webrtcMetrics.packetsLost}；热点链路会为恢复丢包而增加缓冲。`];
    return ["H.264 / WebRTC 低延迟模式", `浏览器状态：${webrtcMetrics.state}；请观察接收 FPS、码率和抖动缓冲。`];
  }
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
async function refreshStatus() { try { const statusStartedAt = performance.now(); const response = await fetch("/api/status", {cache:"no-store"}), data = await response.json(), robot = data.robot, system = data.system || {}, oled = data.oled || {}, capabilities = data.capabilities || {}; statusRttMs = performance.now() - statusStartedAt;
    const camera = data.camera || {};
    setVideoTransport(camera);
    $("#cameraState").innerHTML = dot(camera.online, camera.online ? "在线" : camera.status); $("#arduinoState").innerHTML = dot(robot.arduino_online, robot.arduino_online ? "在线" : robot.serial ? "无响应" : "离线", robot.serial);
    const resolution = camera.resolution || (camera.width && camera.height ? `${camera.width}×${camera.height}` : "—");
    if (isMjpeg()) {
      if (!cameraModeDirty && !cameraModeBusy && camera.mode) $("#cameraMode").value = camera.mode;
      if (!streamProfileDirty && !streamProfileBusy && camera.stream_profile?.key) $("#streamProfile").value = camera.stream_profile.key;
      if (camera.width && camera.height) { $("#auxCropWidth").max = camera.width; $("#auxCropHeight").max = camera.height; }
      if (!exposureDirty && !exposureBusy && camera.exposure) {
        $("#exposureMode").value = camera.exposure.auto ? "auto" : "manual";
        $("#cameraEv").value = camera.exposure.ev;
        $("#shutterDenominator").value = camera.exposure.shutter_denominator;
        updateExposureUi();
      }
    }
    const streamResolution = camera.stream_profile?.resolution || resolution;
    $("#cameraMeta").textContent = isMjpeg()
      ? `采集 ${resolution} → 网页 ${streamResolution} · 传感器目标 ${fixed(camera.sensor_target_fps)} FPS · 实际编码 ${fixed(camera.capture_fps)} FPS`
      : `采集 ${resolution} · H.264 ${fixed(camera.sensor_target_fps)} FPS · WebRTC 低延迟传输`;
    $("#cameraResolution").textContent = isMjpeg() ? `${resolution} → ${streamResolution}` : resolution;
    $("#streamProfileState").textContent = camera.stream_profile?.label || "—";
    $("#cameraFps").textContent = isMjpeg() ? `${fixed(camera.capture_fps)} / ${fixed(camera.stream_fps)} FPS` : `${fixed(camera.sensor_target_fps)} FPS · H.264`;
    $("#cameraBandwidthLabel").textContent = isMjpeg() ? "MJPEG 发送带宽（所有网页合计）" : "H.264 目标码率";
    $("#cameraBandwidthHint").textContent = isMjpeg() ? "同时显示 kB/s 和 kbps，包含 MJPEG 分片头；单个网页时就是当前流量" : "实际 WebRTC 码率由 MediaMTX/浏览器统计；此处显示启动时的编码目标。";
    $("#cameraBandwidth").textContent = isMjpeg() ? `${fixed(camera.stream_kBps)} kB/s · ${fixed(camera.stream_kbps)} kbps` : camera.stream_profile?.label || "H.264";
    $("#cameraBandwidthDetail").textContent = $("#cameraBandwidth").textContent;
    $("#cameraFrameSize").textContent = isMjpeg() ? (camera.jpeg_bytes ? `${fixed(camera.jpeg_bytes / 1000)} KB` : "—") : "连续 H.264 帧";
    $("#cameraEncode").textContent = isMjpeg() ? `${fixed(camera.encode_ms)} ms（平均 ${fixed(camera.encode_ms_avg)} ms）` : (camera.stream_profile?.encoder || "H.264 encoder");
    $("#cameraAge").textContent = isMjpeg() ? (camera.frame_age_ms == null ? "—" : `${fixed(camera.frame_age_ms)} ms`) : "由 WebRTC 自适应";
    $("#dashboardFps").textContent = isMjpeg() ? `${fixed(camera.capture_fps)} / ${fixed(camera.stream_fps)} FPS` : `${fixed(webrtcMetrics.fps)} / ${fixed(camera.sensor_target_fps)} FPS`;
    $("#dashboardFrameAge").textContent = isMjpeg() ? (camera.frame_age_ms == null ? "—" : `${fixed(camera.frame_age_ms)} ms`) : (webrtcMetrics.jitterBufferMs == null ? "—" : `${fixed(webrtcMetrics.jitterBufferMs)} ms`);
    $("#dashboardEncode").textContent = isMjpeg() ? `${fixed(camera.encode_ms)} ms` : (camera.stream_profile?.encoder || "H.264");
    $("#dashboardBandwidth").textContent = isMjpeg() ? `${fixed(camera.stream_kbps)} kbps` : (webrtcMetrics.kbps == null ? "—" : `${fixed(webrtcMetrics.kbps)} kbps`);
    const highres = camera.highres || {}, highresProfile = camera.highres_profile || {};
    if (!highresProfileDirty && !highresProfileBusy && highresProfile.key) $("#highresProfile").value = highresProfile.key;
    if (!highresFpsDirty && !highresFpsBusy && highres.target_fps != null) $("#highresFps").value = highres.target_fps;
    const highresFpsSupported = capabilities.highres_fps_control === true;
    if (!highresFpsBusy) { $("#highresFps").disabled = !highresFpsSupported; $("#applyHighresFps").disabled = !highresFpsSupported; }
    const highresAvailable = isMjpeg() || !!camera.highres_available;
    $("#highresStats").textContent = highresAvailable
      ? `${highresProfile.resolution || "—"} · ${fixed(highres.capture_fps)} / ${fixed(highres.target_fps)} FPS · 发送 ${fixed(highres.stream_kBps)} kB/s`
      : "当前 WebRTC 模式不可用";
    $("#highresHint").textContent = highresAvailable
      ? `JPEG ${highresProfile.quality ?? 75} · 单张 ${highres.jpeg_bytes ? `${fixed(highres.jpeg_bytes / 1000)} KB` : "—"} · 编码 ${fixed(highres.encode_ms)} ms · 客户端 ${highres.active_clients ?? 0}`
      : "请使用 start_robot.sh 启动 MJPEG 服务。";
    $("#highresStatusDetail").textContent = highresPreviewEnabled ? $("#highresStats").textContent : "预览关闭 · 不占用网页高清传输带宽";
    $("#highresDetailHint").textContent = highresPreviewEnabled ? $("#highresHint").textContent : "点击高清画面右上角“开启预览”后再订阅。";
    if (camera.transport === "webrtc") {
      const fps = webrtcMetrics.fps == null ? "—" : `${fixed(webrtcMetrics.fps)} FPS`;
      const rate = webrtcMetrics.kbps == null ? "—" : `${fixed(webrtcMetrics.kbps)} kbps`;
      $("#webrtcReception").textContent = `${webrtcMetrics.state} · ${fps} · ${rate}`;
      const jitter = webrtcMetrics.jitterBufferMs == null ? "—" : `${fixed(webrtcMetrics.jitterBufferMs)} ms`;
      const lost = webrtcMetrics.packetsLost == null ? "—" : webrtcMetrics.packetsLost;
      const dropped = webrtcMetrics.framesDropped == null ? "—" : webrtcMetrics.framesDropped;
      $("#webrtcReceptionDetail").textContent = `抖动缓冲 ${jitter} · 累计丢包 ${lost} · 浏览器丢帧 ${dropped} · 相机缓冲 ${camera.stream_profile?.camera_buffer_count ?? "—"}`;
    } else {
      $("#webrtcReception").textContent = "当前为 MJPEG 模式";
      $("#webrtcReceptionDetail").textContent = "切换到 start_webrtc.sh 后显示浏览器 WebRTC 接收统计。";
    }
    $("#liveFeedMeta").textContent = isMjpeg()
      ? `${streamResolution} · ${fixed(camera.stream_fps)} FPS`
      : `${resolution} · H.264`;
    $("#highresFeedMeta").textContent = highresAvailable
      ? `${highresProfile.resolution || "—"} · ${fixed(highres.capture_fps)} FPS`
      : "MJPEG 专用";
    const [diagnosis, diagnosisDetail] = streamDiagnosis(camera);
    $("#streamDiagnosis").textContent = diagnosis;
    $("#streamDiagnosisDetail").textContent = diagnosisDetail;
    $("#dashboardFlowState").textContent = diagnosis;
    const systemMetricsSupported = capabilities.system_metrics === true && Object.keys(system).length > 0;
    $("#systemCpu").textContent = !systemMetricsSupported ? "后端未更新" : system.cpu_percent == null ? "测量中…" : `${fixed(system.cpu_percent)}%`;
    $("#systemLoad").textContent = !systemMetricsSupported ? "后端未更新" : fixed(system.load_1m, 2);
    $("#systemMemory").textContent = !systemMetricsSupported ? "后端未更新" : system.memory_total_bytes == null ? "不可用" : `${bytes(system.memory_used_bytes)} / ${bytes(system.memory_total_bytes)}`;
    $("#systemTemperature").textContent = !systemMetricsSupported ? "后端未更新" : system.cpu_temperature_c == null ? "不可用" : `${fixed(system.cpu_temperature_c)} °C`;
    $("#systemDisk").textContent = !systemMetricsSupported ? "后端未更新" : system.disk_total_bytes == null ? "不可用" : `${bytes(system.disk_used_bytes)} / ${bytes(system.disk_total_bytes)}`;
    $("#systemUptime").textContent = !systemMetricsSupported ? "后端未更新" : duration(system.uptime_seconds);
    $("#oledState").textContent = oled.online ? `在线 · ${oled.address || "I2C"}` : oled.disabled ? "已关闭" : "不可用";
    $("#oledHint").textContent = oled.online ? `I2C-${oled.i2c_port ?? 1} · 每秒刷新状态` : (oled.error || "检查 I2C、地址和 luma.oled 依赖");
    $("#controlState").innerHTML = dot(robot.client_online, robot.client_online ? "在线" : "已超时停车"); $("#action").textContent = robot.action; $("#keys").textContent = robot.keys?.join("+") || "—"; $("#distance").textContent = distance(robot.ultrasonic); $("#lastReply").textContent = robot.reply || "等待 Arduino 回包";
    const frontDistance = Number(robot.ultrasonic), validDistance = Number.isFinite(frontDistance) && frontDistance >= 0, blocked = validDistance && frontDistance <= 30;
    $("#dashboardDistance").textContent = distance(robot.ultrasonic); $("#dashboardDistanceState").textContent = !validDistance ? "无有效回波" : blocked ? "前进限位" : "通行";
    $("#dashboardDistanceHint").textContent = !validDistance ? "请检查前向超声波回波" : blocked ? `距障碍 ${fixed(frontDistance)} cm · 前进已限制` : `安全距离 ${fixed(frontDistance)} cm`;
    const marker = $("#distanceMarker"); marker.style.left = validDistance ? `${Math.min(frontDistance, 100)}%` : "100%"; marker.style.background = blocked ? "var(--red)" : "var(--green)"; marker.style.boxShadow = blocked ? "0 0 9px rgb(222 89 101)" : "0 0 9px rgb(71 201 140)";
    if (!servoBusy && robot.servo_angle != null) { $("#servoSlider").value = robot.servo_angle; $("#servoAngleDisplay").textContent = `${robot.servo_angle}°`; updateSteeringDial(robot.servo_angle, true); }
    else if (robot.servo_angle == null) updateSteeringDial(null, false);
    if (robot.imu) { $("#roll").textContent = `${robot.imu[0].toFixed(2)}°`; $("#pitch").textContent = `${robot.imu[1].toFixed(2)}°`; $("#yaw").textContent = `${robot.imu[2].toFixed(2)}°`; }
    if (robot.speed) { $("#wheelSpeed").textContent = `${robot.speed[0].toFixed(1)} / ${robot.speed[1].toFixed(1)} pps`; $("#targetWheelSpeed").textContent = `${robot.speed[2].toFixed(1)} / ${robot.speed[3].toFixed(1)} pps`; }
    if (!configLoaded) { fillConfig(robot.config); configLoaded = true; }
    const age = robot.last_rx_age == null ? "—" : `${robot.last_rx_age.toFixed(2)} s`;
    $("#status").textContent = [`后端: ${data.api_version || "旧版本，需同步 app.py"}`, `系统指标: ${systemMetricsSupported ? (system.error || "正常") : "当前后端未提供 system/capabilities"}`, `高清帧率: ${highresFpsSupported ? `${fixed(highres.target_fps)} FPS，可调整` : "当前后端不支持，需同步 camera.py"}`, `Arduino: ${robot.arduino_online ? "在线" : robot.serial ? "无响应" : "离线"}`, `串口: ${robot.serial ? "已打开" : "未打开"}`, `最近回包: ${age}`, `动作: ${robot.action}`, `按键: ${robot.keys?.join("+") || "—"}`, `回复: ${robot.reply || "—"}`, `错误: ${robot.error || "—"}`].join("\n");
  } catch (error) { $("#status").textContent = `网页后端连接失败：${error}`; $("#arduinoState").innerHTML = dot(false, "网页服务异常"); }
}
addEventListener("beforeunload", stopWebrtc);
refreshStatus(); setInterval(refreshStatus, 500);
