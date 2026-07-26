const $ = selector => document.querySelector(selector);
const video = $("#video"), webrtcVideo = $("#webrtcVideo"), highresVideo = $("#highresVideo"), save = $("#save");
const facePreview = $("#facePreview");
const FACE_PREVIEW_BASE = "http://127.0.0.1:5059";
const auxVideo = $("#auxVideo"), auxContext = auxVideo.getContext("2d", {alpha:false});
let folder, aborter, count = 0, profiles = {}, configLoaded = false, keysSending = false, keysQueued = false, stopQueued = false;
let faceTurnSending = false, faceTurnInFlightCommand = null, queuedFaceTurnCommand = null;
let cameraModeDirty = false, cameraModeBusy = false, streamProfileDirty = false, streamProfileBusy = false, highresProfileDirty = false, highresProfileBusy = false, highresFpsDirty = false, highresFpsBusy = false, exposureDirty = false, exposureBusy = false, colorCorrectionDirty = false, colorCorrectionBusy = false, videoRetryTimer;
let servoBusy = false, queuedServoAngle = null, steeringCenterAngle = 90, steeringReversed = true;
let feedBusy = false, dealBusy = false;
let pendingDealRequest = null, dealRequestSerial = 0, dealResetTimer = null;
let visualServoAngle = 90, visualServoVelocity = 0, visualSteeringDirection = 0, visualServoLastAt = performance.now();
let receivedFrameCount = 0, receivedFrameWindowAt = performance.now(), browserReceiveFps = 0, statusRttMs = null;
let activeVideoTransport = "mjpeg", currentWebrtcUrl = "";
let highresPreviewEnabled = false, highresPreviewAvailable = false;
let webrtcPeer = null, webrtcSessionUrl = "", webrtcStatsTimer = null, webrtcStatsPrevious = null;
const webrtcMetrics = {state:"未连接", fps:null, kbps:null, jitterMs:null, jitterBufferMs:null, packetsLost:null, framesDropped:null};
const actionKeys = {F:"w", SF:"slow", PL:"a", PR:"d", SPL:"x", SPR:"c", B:"s"};
const keyboardKeys = {w:"w", a:"a", s:"s", d:"d", x:"x", c:"c", ArrowUp:"w", ArrowDown:"s", ArrowLeft:"a", ArrowRight:"d"};
const heldKeys = new Set();
const heldSteeringKeys = new Set();
let autonomousToggleBusy = false;
let facePreviewRetryTimer = null;
let routeTuningBusy = false;
const routeTuningDirtyInputs = new Set();

async function toggleAutonomousDrive() {
  if (autonomousToggleBusy) return;
  autonomousToggleBusy = true;
  try {
    // Never leave a manual key held when handing control to the route tracker.
    releaseKeys();
    const response = await requestJson("/api/autonomous/toggle", {method:"POST"}, 1000);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "自动行驶切换失败");
    updateAutonomousUi(data.autonomous || {});
  } catch (error) {
    note(error.message);
  } finally {
    autonomousToggleBusy = false;
  }
}
function updateAutonomousUi(autonomous) {
  const available = autonomous.available === true;
  const enabled = autonomous.enabled === true;
  const scanlineI = autonomous.mode === "scanline_i" || autonomous.mode === "scanline_i_green_white" || autonomous.mode === "scanline_i_four_endpoint_green_white";
  const endLine = autonomous.mode === "end_line_turn_adaptor";
  const scanlineLabel = autonomous.mode === "scanline_i_four_endpoint_green_white" ? "工字形四端点验证" : (autonomous.mode === "scanline_i_green_white" ? "绿地白线 I 型" : "扫描线 I 型");
  const button = $("#autonomousToggle"), unavailable = $("#routePreviewUnavailable"), image = $("#routePreview");
  button.disabled = !available;
  button.textContent = available ? (enabled ? "M：暂停并停车" : "M：开启自动行驶") : "路线预判未开启";
  $("#autonomousState").textContent = available ? `${scanlineI ? scanlineLabel : "视觉"}：${autonomous.state || "等待"} · ${autonomous.detail || "—"}` : "视觉识别：本次服务未开启";
  $("#routePreviewMeta").textContent = available ? `${enabled ? "行驶中" : "已暂停"} · ${autonomous.confidence == null ? "—" : `置信度 ${fixed(autonomous.confidence)}`}` : "未开启";
  unavailable.classList.toggle("hidden", available);
  if (!available) image.removeAttribute("src");
  const tuning = autonomous.tuning || {};
  const activeTuningInputs = document.querySelectorAll(scanlineI ? "[data-scanline-tuning]" : (endLine ? "[data-end-line-tuning]" : "[data-route-tuning]"));
  for (const input of activeTuningInputs) {
    const key = scanlineI ? input.dataset.scanlineTuning : (endLine ? input.dataset.endLineTuning : input.dataset.routeTuning);
    const supported = Object.prototype.hasOwnProperty.call(tuning, key);
    if (supported && document.activeElement !== input && !routeTuningDirtyInputs.has(input)) input.value = tuning[key];
    input.closest("label")?.classList.toggle("hidden", endLine && !supported);
    input.disabled = !available || routeTuningBusy;
  }
  $("#scanlineRouteTuning").classList.toggle("hidden", !scanlineI);
  $("#endLineRouteTuning").classList.toggle("hidden", !endLine);
  $("#endLineTurnProfiles").classList.toggle("hidden", !endLine);
  $("#endLineGreenGate").classList.toggle("hidden", !endLine);
  $("#genericRouteTuning").classList.toggle("hidden", scanlineI || endLine);
  $("#genericRouteTuningNote").classList.toggle("hidden", scanlineI || endLine);
  const tuningState = $("#routeTuningState");
  if (tuningState) tuningState.textContent = available ? (routeTuningDirtyInputs.size ? "有未保存的参数修改" : (scanlineI ? "扫描线 I 型实时参数" : (endLine ? "单白线按键转向实时参数" : "实时参数"))) : "路线预判未开启";
  $("#applyRouteTuning").disabled = !available || routeTuningBusy;
  $("#applyRouteTuning").textContent = scanlineI ? "实时应用并保存 I 型参数" : (endLine ? "实时应用并保存单白线参数" : "实时应用并保存路线参数");
}
$("#autonomousToggle").onclick = toggleAutonomousDrive;
for (const input of document.querySelectorAll("[data-scanline-tuning],[data-end-line-tuning],[data-route-tuning]")) {
  input.addEventListener("input", () => { routeTuningDirtyInputs.add(input); });
  input.addEventListener("change", () => { routeTuningDirtyInputs.add(input); });
}
$("#applyRouteTuning").onclick = async () => {
  if (routeTuningBusy) return;
  const payload = {};
  const scanlineI = $("#scanlineRouteTuning").classList.contains("hidden") === false;
  const endLine = $("#endLineRouteTuning").classList.contains("hidden") === false;
  const inputs = [...document.querySelectorAll(scanlineI ? "[data-scanline-tuning]" : (endLine ? "[data-end-line-tuning]" : "[data-route-tuning]"))];
  for (const input of inputs) {
    const value = Number(input.value);
    if (!Number.isFinite(value) || !input.checkValidity()) {
      note(`${input.closest("label")?.childNodes[0]?.textContent?.trim() || "参数"}不是有效数值`);
      input.focus();
      return;
    }
    payload[scanlineI ? input.dataset.scanlineTuning : (endLine ? input.dataset.endLineTuning : input.dataset.routeTuning)] = value;
  }
  routeTuningBusy = true;
  const button = $("#applyRouteTuning");
  button.disabled = true;
  for (const input of inputs) input.disabled = true;
  try {
    const response = await requestJson("/api/autonomous/tuning", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)}, 5000);
    const data = await response.json().catch(() => ({}));
    if (!response.ok || !data.ok) throw Error(data.error || `循迹参数应用失败（HTTP ${response.status}）`);
    for (const input of inputs) routeTuningDirtyInputs.delete(input);
    updateAutonomousUi(data.autonomous || {});
    note(scanlineI ? "I 型直行、掉头与预判刹车参数已实时应用并保存。" : (endLine ? "单白线、Q/E/U/I 预设与 J/L/H/K 视觉转向参数已实时应用并保存。" : "循迹参数已实时应用，并保存到 tuning.py。"));
  } catch (error) {
    note(error.name === "AbortError" ? "循迹参数保存超时，输入值已保留，请检查网络后重试。" : error.message);
  } finally {
    routeTuningBusy = false;
    button.disabled = false;
    for (const input of inputs) input.disabled = false;
  }
};

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
function connectFacePreview() {
  clearTimeout(facePreviewRetryTimer);
  facePreview.src = `${FACE_PREVIEW_BASE}/preview_feed?ts=${Date.now()}`;
}
facePreview.addEventListener("load", () => {
  $("#facePreviewCard").classList.remove("feed-error");
  $("#facePreviewUnavailable").classList.add("hidden");
});
facePreview.addEventListener("error", () => {
  $("#facePreviewCard").classList.add("feed-error");
  $("#facePreviewUnavailable").classList.remove("hidden");
  facePreviewRetryTimer = setTimeout(connectFacePreview, 1500);
});
async function refreshFaceDetectionStatus() {
  try {
    const response = await requestJson(
      `${FACE_PREVIEW_BASE}/api/face/latest?ts=${Date.now()}`,
      {cache:"no-store", mode:"cors"},
      700,
    );
    if (!response.ok) throw Error(`HTTP ${response.status}`);
    const face = await response.json();
    if (face.error) throw Error(face.error);
    const capturedAt = Date.parse(face.time), ageMs = Number.isFinite(capturedAt) ? Date.now() - capturedAt : null;
    const fresh = ageMs != null && ageMs >= 0 && ageMs <= 450;
    const offset = Number(face.offset_x_normalized);
    const centred = face.detected === true && Number.isFinite(offset) && Math.abs(offset) <= .20;
    const rawCount = Number(face.detected_face_count) || 0;
    const usableCount = Number(face.usable_face_count) || 0;
    $("#facePreviewMeta").textContent = `${face.model ? "DeskMate YuNet" : "旧检测器"} · ${fresh ? "实时" : `${fixed(ageMs)} ms 前`}`;
    $("#faceDetectionState").textContent = face.detected
      ? `已识别 · 置信度 ${fixed(face.score, 2)} · 原始 ${rawCount} / 可用 ${usableCount}`
      : `未识别 · 原始 ${rawCount} / 可用 ${usableCount} · 阈值 ${fixed(face.detector_score_threshold, 2)}`;
    $("#faceGateState").textContent = face.detected && Number.isFinite(offset)
      ? `${centred ? "已进入" : "未进入"}中心门禁 · 偏移 ${offset >= 0 ? "+" : ""}${fixed(offset, 3)}`
      : "中心门禁 ±20%";
  } catch (error) {
    $("#facePreviewMeta").textContent = "电脑 5059 离线";
    $("#faceDetectionState").textContent = `人脸识别不可用：${error.message}`;
    $("#faceGateState").textContent = "不影响 CSI 与路线识别";
    $("#facePreviewUnavailable").classList.remove("hidden");
  }
}
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
// Status polling runs every 500 ms.  Mark the field dirty on each keystroke,
// not only after blur, so polling cannot overwrite a partially typed FPS.
$("#highresFps").addEventListener("input", () => { highresFpsDirty = true; });
$("#highresFps").addEventListener("change", () => { highresFpsDirty = true; });
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

for (const input of [$("#colorCorrectionEnabled"), $("#colorCorrectionStrength")]) input.addEventListener("input", () => { colorCorrectionDirty = true; });
$("#applyColorCorrection").onclick = async () => {
  if (colorCorrectionBusy) return;
  colorCorrectionBusy = true;
  const button = $("#applyColorCorrection"), enabled = $("#colorCorrectionEnabled"), strength = $("#colorCorrectionStrength");
  button.disabled = true; enabled.disabled = true; strength.disabled = true;
  try {
    const response = await fetch("/api/camera/color-correction", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({enabled:enabled.checked, strength:Number(strength.value)})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "颜色校正设置失败");
    colorCorrectionDirty = false;
    note(data.camera.color_correction.enabled ? `已开启边缘偏色校正，强度 ${fixed(data.camera.color_correction.strength, 2)}` : "已关闭边缘偏色校正");
  } catch (error) { note(error.message); }
  finally { colorCorrectionBusy = false; button.disabled = false; enabled.disabled = false; strength.disabled = false; }
};

function editing(event) { return ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName); }
function steeringDirection() {
  if (heldSteeringKeys.has("q") === heldSteeringKeys.has("e")) return 0;
  return heldSteeringKeys.has("q") ? -1 : 1;
}
function syncVisualSteeringDirection() {
  visualSteeringDirection = steeringDirection() * (steeringReversed ? -1 : 1);
}
function animateServoIndicator(now) {
  const elapsed = Math.max(0, Math.min(.1, (now - visualServoLastAt) / 1000));
  visualServoLastAt = now;
  const speed = Number($("#servoSpeedDps")?.value) || 45;
  const acceleration = Number($("#servoAccelerationDps2")?.value) || 120;
  const wantedVelocity = visualSteeringDirection * speed;
  const maximumDelta = acceleration * elapsed;
  visualServoVelocity += Math.max(-maximumDelta, Math.min(maximumDelta, wantedVelocity - visualServoVelocity));
  if (Number.isFinite(visualServoAngle) && (visualSteeringDirection !== 0 || Math.abs(visualServoVelocity) > .01)) {
    visualServoAngle = clamp(visualServoAngle + visualServoVelocity * elapsed, 0, 180);
    $("#servoAngleDisplay").textContent = `${Math.round(visualServoAngle)}°`;
  }
  requestAnimationFrame(animateServoIndicator);
}
async function sendKeys(stop = false) {
  if (stop) stopQueued = true;
  if (keysSending) { keysQueued = true; return; }
  keysSending = true;
  do {
    keysQueued = false;
    const stopRequested = stopQueued;
    stopQueued = false;
    // Capture which P event is actually carried by this particular HTTP
    // request. A previous WASD heartbeat may already be in flight when P is
    // pressed; its timeout must never clear the newly queued P event.
    const dealRequestSent = pendingDealRequest;
    try {
      const payload = {keys:[...heldKeys], steering:steeringDirection()};
      if (stopRequested) payload.stop = true;
      if (dealRequestSent) payload.deal_request = dealRequestSent;
      const response = await requestJson("/api/keys", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload), keepalive:true}, dealRequestSent ? 3500 : 1800);
      const data = await response.json();
      if (!response.ok || !data.ok) throw Error(data.error || "动作发送失败");
      $("#action").textContent = data.action;
      if (data.deal && dealRequestSent && data.deal.token === dealRequestSent.token) {
        if (data.deal.state === "pending") {
          note(`P 已送达树莓派：正在等待 M3、M4 的 Arduino 确认`);
          continue;
        }
        const request = dealRequestSent;
        if (pendingDealRequest?.token === dealRequestSent.token) pendingDealRequest = null;
        const feedResult = data.deal.feed || {}, dealResult = data.deal.deal || {};
        const feedText = feedResult.state === "error"
          ? `M3失败：${feedResult.error || feedResult.reply || "未确认"}`
          : feedResult.state === "busy"
            ? `M3已在运行：${feedResult.reply}`
            : `M3已运行 ${request.feed_direction_label}/PWM ${request.feed_power}/${request.feed_seconds}秒：${feedResult.reply}`;
        const dealText = dealResult.state === "error"
          ? `M4失败：${dealResult.error || dealResult.reply || "未确认"}`
          : dealResult.state === "busy"
            ? `M4已在运行：${dealResult.reply}`
            : dealResult.state === "legacy"
              ? `M4按旧固件固定参数运行：${dealResult.reply}`
              : `M4已运行 ${request.deal_direction_label}/PWM ${request.deal_power}/${request.deal_seconds}秒：${dealResult.reply}`;
        note(`${feedText}；${dealText}`);
        resetDealControls(data.deal.state === "error" ? 0 : Math.max(request.feed_duration_ms, request.deal_duration_ms));
      }
    } catch (error) {
      if (dealRequestSent) {
        note(`M4 命令失败：${error.message}`);
        if (pendingDealRequest?.token === dealRequestSent.token) pendingDealRequest = null;
        resetDealControls(0);
      } else {
        note(`行驶命令失败：${error.message}`);
      }
    }
  } while (keysQueued || stopQueued);
  keysSending = false;
}
function setKey(key, pressed) { if (pressed) heldKeys.add(key); else heldKeys.delete(key); sendKeys(); }
function setSteeringKey(key, pressed) { if (pressed) heldSteeringKeys.add(key); else heldSteeringKeys.delete(key); syncVisualSteeringDirection(); sendKeys(); }
function stopFaceVisionTurn() {
  requestJson("/api/autonomous/face-turn", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command:"STOP"})}, 800).catch(() => {});
}
function stopLineVisionTurn() {
  requestJson("/api/autonomous/line-turn", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command:"STOP"})}, 800).catch(() => {});
}
function stopVisionTurns() {
  stopFaceVisionTurn();
  stopLineVisionTurn();
}
function releaseKeys(stopVision = true) {
  const hadManualInput = heldKeys.size > 0 || heldSteeringKeys.size > 0;
  heldKeys.clear();
  heldSteeringKeys.clear();
  syncVisualSteeringDirection();
  if (stopVision || hadManualInput) sendKeys(stopVision);
  if (stopVision) stopVisionTurns();
}
async function manualVisionTurn(command) {
  releaseKeys();
  try {
    const response = await requestJson("/api/autonomous/manual-turn", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command})}, 1200);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "转向请求失败");
    updateAutonomousUi(data.autonomous || {});
    note(`${command} 已触发：仅按配置的分段时间转动，全部段完成即停车；红线仅作画面诊断。空格或 M 可停止。`);
  } catch (error) { note(error.message); }
}
async function flushFaceVisionTurnQueue() {
  if (faceTurnSending) return;
  faceTurnSending = true;
  while (queuedFaceTurnCommand) {
    const command = queuedFaceTurnCommand;
    queuedFaceTurnCommand = null;
    faceTurnInFlightCommand = command;
    try {
      const response = await requestJson("/api/autonomous/face-turn", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command})}, 2500);
      const data = await response.json();
      if (!response.ok || !data.ok) throw Error(data.error || "人脸转向请求失败");
      updateAutonomousUi(data.autonomous || {});
      note(`${command === "START_LEFT" ? "J 人脸持续左转" : "L 人脸持续右转"}已启动：电脑端 face_turn_web_bridge.py 将持续续租，只在人脸居中时停车；桥接停止后 Pi 会在 3 秒内安全停车。`);
    } catch (error) {
      note(`人脸转向请求失败：${error.message}`);
    } finally {
      faceTurnInFlightCommand = null;
    }
  }
  faceTurnSending = false;
}
function faceVisionTurn(command) {
  // Do not issue the generic async STOP before START: a delayed STOP could
  // otherwise arrive at the Pi after START and cancel the new face turn.
  releaseKeys(false);
  if (command === faceTurnInFlightCommand || command === queuedFaceTurnCommand) return;
  queuedFaceTurnCommand = command;
  flushFaceVisionTurnQueue();
}
async function lineVisionTurn(command) {
  // As with J/L, avoid sending an asynchronous stale STOP immediately before
  // the new H/K request.
  releaseKeys(false);
  try {
    const response = await requestJson("/api/autonomous/line-turn", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command})}, 1200);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "白线转向请求失败");
    updateAutonomousUi(data.autonomous || {});
    note(`${command === "START_LEFT" ? "H 白线持续左转" : "K 白线持续右转"}已启动：忽略起始居中的白线，离开后再次连续 3 帧居中才停车；15 秒未找到会安全停车。`);
  } catch (error) { note(error.message); }
}
async function roundtripRequest(path, body, successText) {
  releaseKeys(false);
  try {
    const options = {method:"POST"};
    if (body) {
      options.headers = {"Content-Type":"application/json"};
      options.body = JSON.stringify(body);
    }
    const response = await requestJson(`/api/autonomous/roundtrip/${path}`, options, 1200);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "双人脸序列请求失败");
    updateAutonomousUi(data.autonomous || {});
    note(successText);
  } catch (error) { note(error.message); }
}
function startRoundtrip(sweepSide) {
  return roundtripRequest("start", {sweep_side:sweepSide}, `${sweepSide === "LEFT" ? "左扫" : "右扫"}序列已启动：先沿白线到端点，再按 人脸→白线→人脸→反向白线 执行；人脸阶段需要电脑桥接持续运行。`);
}
function startRoundtripReturn() {
  return roundtripRequest("return", null, "返程已启动：沿当前居中的白线行驶，到另一端后自动停车。");
}
function stopRoundtrip() {
  return roundtripRequest("stop", null, "双人脸往返序列已停止。");
}
async function followToEnd() {
  releaseKeys();
  try {
    const response = await requestJson("/api/autonomous/follow-to-end", {method:"POST"}, 1200);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "N 自动巡线失败");
    updateAutonomousUi(data.autonomous || {});
    note("N 已触发：沿白线行驶至尽头，到达后自动回到手动模式等待 Q/E 转向。");
  } catch (error) { note(error.message); }
}
function timedMotorSettings(prefix) {
  const power = Number($(`#${prefix}Pwm`).value);
  const direction = Number($(`#${prefix}Direction`).value);
  const seconds = Number($(`#${prefix}Seconds`).value);
  if (!Number.isInteger(power) || power < 1 || power > 255) throw Error("PWM 必须是 1 到 255 的整数；0 不会驱动电机");
  if (direction !== 1 && direction !== -1) throw Error("电机方向必须选择正转或反转");
  if (!Number.isFinite(seconds) || seconds < .1 || seconds > 60) throw Error("运行时间必须在 0.1 到 60 秒之间");
  return {pwm: power * direction, power, direction, directionLabel: direction > 0 ? "正转" : "反转", duration_ms: Math.round(seconds * 1000), seconds};
}
async function feedCards() {
  if (feedBusy) return;
  const button = document.querySelector("[data-feed]");
  let settings;
  try { settings = timedMotorSettings("feed"); } catch (error) { note(error.message); return; }
  feedBusy = true;
  if (button) button.disabled = true;
  try {
    const response = await requestJson("/api/feed", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(settings)}, 3500);
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "M3 送牌命令发送失败");
    note(`已触发 M3：${settings.directionLabel}，PWM ${settings.power}，运行 ${settings.seconds} 秒`);
  } catch (error) {
    note(error.message);
  } finally {
    setTimeout(() => {
      feedBusy = false;
      if (button) button.disabled = false;
    }, settings.duration_ms);
  }
}
async function dealCard() {
  if (dealBusy) return;
  let feedSettings, dealSettings;
  try {
    feedSettings = timedMotorSettings("feed");
    dealSettings = timedMotorSettings("deal");
  } catch (error) { note(error.message); return; }
  dealBusy = true;
  const button = document.querySelector("[data-deal]");
  if (button) button.disabled = true;
  pendingDealRequest = {
    token:`${Date.now()}-${++dealRequestSerial}`,
    feed_pwm:feedSettings.pwm,
    feed_power:feedSettings.power,
    feed_direction_label:feedSettings.directionLabel,
    feed_duration_ms:feedSettings.duration_ms,
    feed_seconds:feedSettings.seconds,
    deal_pwm:dealSettings.pwm,
    deal_power:dealSettings.power,
    deal_direction_label:dealSettings.directionLabel,
    deal_duration_ms:dealSettings.duration_ms,
    deal_seconds:dealSettings.seconds,
  };
  note(`P 组合命令发送中：M3 ${feedSettings.directionLabel}/PWM ${feedSettings.power}/${feedSettings.seconds}秒，M4 ${dealSettings.directionLabel}/PWM ${dealSettings.power}/${dealSettings.seconds}秒`);
  await sendKeys();
}
function resetDealControls(delayMs) {
  clearTimeout(dealResetTimer);
  dealResetTimer = setTimeout(() => {
    dealBusy = false;
    const button = document.querySelector("[data-deal]");
    if (button) button.disabled = false;
  }, Math.max(0, delayMs));
}
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
for (const button of document.querySelectorAll("[data-face-turn]")) {
  button.addEventListener("click", () => faceVisionTurn(button.dataset.faceTurn));
}
for (const button of document.querySelectorAll("[data-line-turn]")) {
  button.addEventListener("click", () => lineVisionTurn(button.dataset.lineTurn));
}
for (const button of document.querySelectorAll("[data-roundtrip-start]")) {
  button.addEventListener("click", () => startRoundtrip(button.dataset.roundtripStart));
}
for (const button of document.querySelectorAll("[data-roundtrip-return]")) button.addEventListener("click", startRoundtripReturn);
for (const button of document.querySelectorAll("[data-roundtrip-stop]")) button.addEventListener("click", stopRoundtrip);
for (const button of document.querySelectorAll("[data-feed]")) button.addEventListener("click", feedCards);
for (const button of document.querySelectorAll("[data-deal]")) button.addEventListener("click", dealCard);
for (const button of document.querySelectorAll("[data-servo-center]")) button.addEventListener("click", centerServo);
addEventListener("keydown", event => { if (event.repeat) return;
  // P is reserved for the combined M3/M4 card action. Use KeyboardEvent.code as well
  // so the physical key still works with a Chinese input method enabled.
  if (event.code === "KeyP" || event.key?.toLowerCase() === "p") { event.preventDefault(); dealCard(); return; }
  // M controls only the autonomous motor gate. Vision stays alive and the
  // current route-prediction frame remains visible while no form field is active.
  if (!editing(event) && (event.code === "KeyM" || event.key?.toLowerCase() === "m")) { event.preventDefault(); toggleAutonomousDrive(); return; }
  if (editing(event)) return;
  if (event.code === "Space") { event.preventDefault(); releaseKeys(); return; }
  if (event.key?.toLowerCase() === "z") { event.preventDefault(); centerServo(); return; }
  if (!editing(event) && (event.code === "KeyN" || event.key?.toLowerCase() === "n")) { event.preventDefault(); followToEnd(); return; }
  if (!editing(event) && (event.code === "KeyJ" || event.key?.toLowerCase() === "j")) { event.preventDefault(); faceVisionTurn("START_LEFT"); return; }
  if (!editing(event) && (event.code === "KeyL" || event.key?.toLowerCase() === "l")) { event.preventDefault(); faceVisionTurn("START_RIGHT"); return; }
  if (!editing(event) && (event.code === "KeyH" || event.key?.toLowerCase() === "h")) { event.preventDefault(); lineVisionTurn("START_LEFT"); return; }
  if (!editing(event) && (event.code === "KeyK" || event.key?.toLowerCase() === "k")) { event.preventDefault(); lineVisionTurn("START_RIGHT"); return; }
  const turnKey = event.key?.toLowerCase();
  // KeyboardEvent.code keeps Q/E/U/I stable when a Chinese IME is active.
  const manualTurn = {q:"LEFT_90", e:"RIGHT_90", u:"LEFT_180", i:"RIGHT_180"}[turnKey] || {KeyQ:"LEFT_90", KeyE:"RIGHT_90", KeyU:"LEFT_180", KeyI:"RIGHT_180"}[event.code];
  if (manualTurn) { event.preventDefault(); manualVisionTurn(manualTurn); return; }
  const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, true); }
});
addEventListener("keyup", event => { if (editing(event)) return; if (event.code === "KeyP" || event.key?.toLowerCase() === "p") { event.preventDefault(); return; } const key = keyboardKeys[event.key] || keyboardKeys[event.key?.toLowerCase()]; if (key) { event.preventDefault(); setKey(key, false); } });
// Losing browser focus must release held WASD/QE keys, but must not cancel a
// landmark sequence that is intentionally continuing under Pi/PC vision.
addEventListener("blur", () => releaseKeys(false)); addEventListener("beforeunload", () => navigator.sendBeacon("/api/stop"));
setInterval(() => {
  if (heldKeys.size > 0 || heldSteeringKeys.size > 0 || pendingDealRequest) sendKeys();
}, 180);
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
  queuedServoAngle = angle;
  void flushServoQueue();
});

async function centerServo() {
  heldSteeringKeys.clear(); sendKeys(); queuedServoAngle = null;
  visualSteeringDirection = 0; visualServoVelocity = 0; visualServoAngle = steeringCenterAngle;
  $("#servoSlider").value = steeringCenterAngle;
  $("#servoAngleDisplay").textContent = `${steeringCenterAngle}°`;
  try {
    const response = await fetch("/api/servo", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({angle:steeringCenterAngle, fast:true})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "摄像头快速回正失败");
    note("摄像头正在快速回正");
  } catch (error) { note(error.message); }
}
$("#servoCenter").onclick = centerServo;
$("#applyServoSettings").onclick = async () => {
  const payload = {servo_center_angle:Number($("#servoCenterAngle").value), servo_speed_dps:Number($("#servoSpeedDps").value), servo_acceleration_dps2:Number($("#servoAccelerationDps2").value), servo_qe_reversed:$("#servoQeReversed").checked};
  try {
    const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)});
    const data = await response.json(); if (!response.ok || !data.ok) throw Error(data.error || "转向设置保存失败");
    steeringCenterAngle = data.config.servo_center_angle; steeringReversed = !!data.config.servo_qe_reversed;
    $("#servoCenterAngle").value = steeringCenterAngle; $("#servoSpeedDps").value = data.config.servo_speed_dps; $("#servoAccelerationDps2").value = data.config.servo_acceleration_dps2; $("#servoQeReversed").checked = steeringReversed;
    note("转向设置已保存");
  } catch (error) { note(error.message); }
};

function profileFor(action) {
  return profiles[action] || {rf:0, lf:0, lr:0, rr:0};
}
function boundedPwm(value) {
  const number = Number(value);
  return Number.isFinite(number) ? Math.max(-255, Math.min(255, Math.round(number))) : 0;
}
function fillDriveProfileEditor() {
  const profile = profileFor($("#profileAction").value);
  $("#rfValue").value = boundedPwm(profile.rf);
  $("#lfValue").value = boundedPwm(profile.lf);
  updateDriveProfilePreview();
}
function driveProfileFromEditor() {
  return {
    ...profileFor($("#profileAction").value),
    rf: boundedPwm($("#rfValue").value),
    lf: boundedPwm($("#lfValue").value),
  };
}
function updateDriveProfilePreview() {
  const profile = driveProfileFromEditor();
  $("#profilePreview").textContent = `M1 ${profile.rf} · M2 ${profile.lf}`;
}
for (const input of document.querySelectorAll("#rfValue,#lfValue")) input.addEventListener("input", updateDriveProfilePreview);
$("#profileAction").onchange = fillDriveProfileEditor;
$("#applyProfile").onclick = async () => {
  profiles[$("#profileAction").value] = driveProfileFromEditor();
  try {
    const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({profiles})});
    const data = await response.json();
    if (!response.ok || !data.ok) throw Error(data.error || "行驶电机配置保存失败");
    profiles = data.config.profiles;
    fillDriveProfileEditor();
    note("M1/M2 当前动作 PWM 已应用并保存");
  } catch (error) { note(error.message); }
};

function fillConfig(config) {
  profiles = config.profiles || profiles;
  $("#speedMode").checked = !!config.speed_mode; $("#targetSpeed").value = config.target_speed; $("#kp").value = config.kp; $("#ki").value = config.ki; $("#kd").value = config.kd;
  steeringCenterAngle = Number(config.servo_center_angle ?? 90); steeringReversed = !!config.servo_qe_reversed;
  $("#servoCenterAngle").value = steeringCenterAngle; $("#servoSpeedDps").value = config.servo_speed_dps ?? 45; $("#servoAccelerationDps2").value = config.servo_acceleration_dps2 ?? 120; $("#servoQeReversed").checked = steeringReversed;
  fillDriveProfileEditor();
}
$("#applyPid").onclick = async () => { const payload = {speed_mode:$("#speedMode").checked, target_speed:Number($("#targetSpeed").value), kp:Number($("#kp").value), ki:Number($("#ki").value), kd:Number($("#kd").value)};
  try { const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)}); if (!response.ok) throw Error("PID 参数提交失败"); note("PID 参数已应用并保存"); } catch (error) { note(error.message); }
};
$("#reconnect").onclick = async () => { const button = $("#reconnect"); button.disabled = true; note("正在重新连接 Arduino……"); try { const response = await fetch("/api/reconnect", {method:"POST"}), data = await response.json(); if (!response.ok || !data.ok) throw Error(data.robot?.error || "Arduino 未收到回包"); note("Arduino 已重新连接并收到回包"); } catch (error) { note(`重新连接失败：${error.message}`); } finally { button.disabled = false; } };

function dot(online, text, warning = false) { return `<i class="dot ${online ? "online" : warning ? "warn" : "offline"}"></i>${text}`; }
function fixed(value, digits = 1) { const number = Number(value); return Number.isFinite(number) ? number.toFixed(digits) : "—"; }
function bytes(value) { const number = Number(value); if (!Number.isFinite(number)) return "—"; return number >= 1e9 ? `${fixed(number / 1e9)} GB` : `${fixed(number / 1e6)} MB`; }
function duration(seconds) { const whole = Math.max(0, Math.floor(Number(seconds) || 0)); return `${Math.floor(whole / 60)} 分 ${whole % 60} 秒`; }
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
function motorMotionComponents(rawOutput) {
  if (!Array.isArray(rawOutput) || rawOutput.length < 2 || !rawOutput.slice(0, 2).every(Number.isFinite)) return null;
  const [right, left] = rawOutput;
  // M1 is the right wheel and M2 is the left wheel.  Each component is only
  // non-zero when both wheel commands agree with that physical motion.
  return {
    right,
    left,
    forward: Math.max(0, Math.min(right, left)),
    reverse: Math.max(0, Math.min(-right, -left)),
    leftTurn: Math.max(0, Math.min(right, -left)),
    rightTurn: Math.max(0, Math.min(-right, left)),
  };
}
function updateMotorOutputUi(rawOutput, arduinoOnline) {
  const motion = motorMotionComponents(rawOutput);
  const fields = ["motorForwardOutput", "motorReverseOutput", "motorLeftOutput", "motorRightOutput"];
  if (!motion) {
    fields.forEach((id) => { $("#" + id).textContent = "—"; });
    $("#motorRawOutput").textContent = "—";
    $("#motorOutputState").textContent = arduinoOnline ? "等待 OUT 回包" : "Arduino 未在线";
    return;
  }
  $("#motorForwardOutput").textContent = `${motion.forward} PWM`;
  $("#motorReverseOutput").textContent = `${motion.reverse} PWM`;
  $("#motorLeftOutput").textContent = `${motion.leftTurn} PWM`;
  $("#motorRightOutput").textContent = `${motion.rightTurn} PWM`;
  $("#motorRawOutput").textContent = `M1 ${motion.right >= 0 ? "+" : ""}${motion.right} · M2 ${motion.left >= 0 ? "+" : ""}${motion.left}`;
  $("#motorOutputState").textContent = arduinoOnline ? "Arduino OUT 回包" : "Arduino 已离线（保留最近值）";
}
async function refreshStatus() { try { const statusStartedAt = performance.now(); const response = await fetch("/api/status", {cache:"no-store"}), data = await response.json(), robot = data.robot, system = data.system || {}, oled = data.oled || {}, capabilities = data.capabilities || {}; statusRttMs = performance.now() - statusStartedAt;
    updateAutonomousUi(data.autonomous || {});
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
      if (!colorCorrectionDirty && !colorCorrectionBusy && camera.color_correction) {
        $("#colorCorrectionEnabled").checked = !!camera.color_correction.enabled;
        $("#colorCorrectionStrength").value = camera.color_correction.strength;
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
    $("#controlState").innerHTML = dot(robot.client_online, robot.client_online ? "在线" : "已超时停车"); $("#action").textContent = robot.action; $("#keys").textContent = robot.keys?.join("+") || "—"; $("#lastReply").textContent = robot.reply || "等待 Arduino 回包"; updateMotorOutputUi(robot.motor_output, robot.arduino_online);
    if (!servoBusy && robot.servo_angle != null) { visualServoAngle = Number(robot.servo_angle); $("#servoSlider").value = robot.servo_angle; $("#servoAngleDisplay").textContent = `${robot.servo_angle}°`; }
    if (robot.imu) { $("#roll").textContent = `${robot.imu[0].toFixed(2)}°`; $("#pitch").textContent = `${robot.imu[1].toFixed(2)}°`; $("#yaw").textContent = `${robot.imu[2].toFixed(2)}°`; }
    if (robot.speed) { $("#wheelSpeed").textContent = `${robot.speed[0].toFixed(1)} / ${robot.speed[1].toFixed(1)} pps`; $("#targetWheelSpeed").textContent = `${robot.speed[2].toFixed(1)} / ${robot.speed[3].toFixed(1)} pps`; }
    if (!configLoaded) { fillConfig(robot.config); configLoaded = true; }
    const age = robot.last_rx_age == null ? "—" : `${robot.last_rx_age.toFixed(2)} s`;
    $("#status").textContent = [`后端: ${data.api_version || "旧版本，需同步 app.py"}`, `系统指标: ${systemMetricsSupported ? (system.error || "正常") : "当前后端未提供 system/capabilities"}`, `高清帧率: ${highresFpsSupported ? `${fixed(highres.target_fps)} FPS，可调整` : "当前后端不支持，需同步 camera.py"}`, `卡牌电机协议: ${robot.card_motor_protocol || "旧后端未报告"}`, `卡牌命令回包: ${robot.card_command_reply || "尚未触发"}`, `驱动配置: ${robot.config_source || "旧后端未报告"}`, `配置路径: ${robot.config_path || "旧后端未报告"}`, `配置读取错误: ${robot.config_error || "无"}`, `Arduino: ${robot.arduino_online ? "在线" : robot.serial ? "无响应" : "离线"}`, `串口: ${robot.serial ? "已打开" : "未打开"}`, `最近回包: ${age}`, `动作: ${robot.action}`, `按键: ${robot.keys?.join("+") || "—"}`, `回复: ${robot.reply || "—"}`, `错误: ${robot.error || "—"}`].join("\n");
  } catch (error) { $("#status").textContent = `网页后端连接失败：${error}`; $("#arduinoState").innerHTML = dot(false, "网页服务异常"); }
}
addEventListener("beforeunload", stopWebrtc);
requestAnimationFrame(animateServoIndicator);
connectFacePreview();
refreshFaceDetectionStatus(); setInterval(refreshFaceDetectionStatus, 500);
refreshStatus(); setInterval(refreshStatus, 200);
