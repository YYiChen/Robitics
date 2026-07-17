(() => {
  "use strict";

  const storageKey = "robitics-ui-language";
  let language = localStorage.getItem(storageKey) === "en" ? "en" : "zh";
  const originalText = new WeakMap();
  const originalAttributes = new WeakMap();

  // Longer phrases are intentionally matched first, so dynamic status text remains natural.
  const dictionary = {
    "Robitics 控制台": "Robitics Control Console",
    "树莓派网页服务 · Arduino Mega 实时控制": "Raspberry Pi Web Service · Arduino Mega Real-Time Control",
    "切换为英文": "Switch to English",
    "切换为中文": "Switch to Chinese",
    "实时预览用于遥控；高清图片按设定帧率刷新（最高 30 FPS），供对照与 DL 使用。两条流均只保留最新帧，不写入树莓派磁盘。": "The live preview is for driving. High-resolution images refresh at the selected rate (up to 30 FPS) for reference and DL. Both streams retain only the latest frame and do not write to the Raspberry Pi disk.",
    "预览默认关闭。开启后才会通过热点接收 1640 px 高清 JPEG。": "Preview is off by default. Enable it to receive 1640 px high-resolution JPEGs over the hotspot.",
    "高清 JPEG 通道仅在 MJPEG 服务模式下可用。": "The high-resolution JPEG channel is available only in MJPEG service mode.",
    "显示当前低延迟预览链路的关键参数。": "Shows key metrics for the current low-latency preview path.",
    "30 cm 内会阻止前进；原地转向与后退不受限制。": "Forward motion is blocked within 30 cm; pivoting and reversing are not restricted.",
    "电脑端辅助裁切（不增加树莓派带宽）": "Computer-side helper crop (no extra Raspberry Pi bandwidth)",
    "图片只写入浏览器授权的电脑文件夹；需要 HTTPS 或 localhost。": "Images are written only to the browser-authorized computer folder; HTTPS or localhost is required.",
    "中心不变；边缘逐渐降低偏红/偏蓝并补偿绿色。建议从 1.00 开始。": "The center is unchanged; edges gradually reduce red/blue tint and compensate green. Start at 1.00.",
    "范围 1–30 FPS（传感器为 30 FPS）；提高帧率会同步提高高清 JPEG 编码和热点带宽占用。": "Range: 1–30 FPS (sensor is 30 FPS). Higher rates also increase high-resolution JPEG encoding and hotspot bandwidth use.",
    "Q/E 按住转动摄像头、松开平滑减速；滑块设置目标角度后由 Arduino 平滑追踪。": "Hold Q/E to pan the camera and release to decelerate smoothly; Arduino smoothly follows the target angle set by the slider.",
    "仅 H.264/WebRTC 模式显示浏览器接收统计。": "Browser reception statistics are shown only in H.264/WebRTC mode.",
    "请观察至少 5 秒。": "Observe for at least 5 seconds.",
    "等待 WebRTC 连接": "Waiting for WebRTC connection",
    "正在收集样本…": "Collecting samples…",
    "等待云台状态": "Waiting for gimbal status",
    "等待传感器数据…": "Waiting for sensor data…",
    "等待 Arduino 回包": "Waiting for Arduino reply",
    "正在读取……": "Loading…",
    "正在建立 WHEP/WebRTC…": "Establishing WHEP/WebRTC…",
    "当前为 H.264/WebRTC 模式，MJPEG 相机设置由推流服务管理。": "H.264/WebRTC mode is active; MJPEG camera settings are managed by the streaming service.",
    "当前为 H.264/WebRTC 模式，高清 JPEG 通道已独立运行。": "H.264/WebRTC mode is active; the high-resolution JPEG channel runs independently.",
    "当前为 H.264/WebRTC 模式，浏览器端保存使用当前实时画面。": "H.264/WebRTC mode is active; browser saving uses the current live frame.",
    "摄像头已切换到": "Camera switched to ",
    "传输档位已切换到": "Stream profile switched to ",
    "高清 JPEG 档位已切换到": "High-resolution JPEG profile switched to ",
    "高清图片帧率已设置为": "High-resolution image rate set to ",
    "曝光设置已应用": "Exposure settings applied",
    "边缘偏色校正已应用": "Edge color correction applied",
    "云台设置已保存": "Gimbal settings saved",
    "已重新连接 Arduino": "Arduino reconnected",
    "正在重新连接 Arduino…": "Reconnecting Arduino…",
    "重新连接失败：": "Reconnect failed: ",
    "保存当前画面": "Save Current Frame",
    "连续保存 5 FPS": "Save Continuously (5 FPS)",
    "停止": "Stop",
    "选择 Pic 文件夹": "Choose Pic Folder",
    "开启预览": "Enable Preview",
    "关闭预览": "Disable Preview",
    "预览关闭": "Preview Off",
    "高清图片预览": "High-Resolution Image Preview",
    "高清 JPEG 通道": "High-Resolution JPEG Channel",
    "实时预览": "Live Preview",
    "低延迟控制画面": "Low-Latency Control View",
    "实时画面": "Live View",
    "驾驶态势": "Driving Overview",
    "摄像头朝向 · 轮端输出": "Camera Direction · Wheel Output",
    "实时流状态": "Live Stream Status",
    "前向超声波": "Front Ultrasonic",
    "摄像头": "Camera",
    "网页控制": "Web Control",
    "读取视频参数…": "Loading video parameters…",
    "连接中…": "Connecting…",
    "读取中…": "Loading…",
    "读取中": "Loading",
    "读取数据…": "Loading data…",
    "读取": "Loading",
    "相机读取档位": "Camera Capture Mode",
    "实时预览传输": "Live Preview Transport",
    "电脑端保存": "Save to Computer",
    "显示方式": "Display Mode",
    "完整画面": "Full Frame",
    "中心裁切": "Center Crop",
    "裁切宽度": "Crop Width",
    "裁切高度": "Crop Height",
    "轮速配置": "Wheel Speed Profiles",
    "直接 PWM": "Direct PWM",
    "遥控动作": "Remote Action",
    "四轮同步": "All Four Wheels",
    "左右两侧": "Left / Right Sides",
    "四轮独立": "Four Wheels Independently",
    "统一 PWM": "Uniform PWM",
    "左侧": "Left Side",
    "右侧": "Right Side",
    "发送": "Output",
    "应用并保存当前动作": "Apply and Save Current Action",
    "后轮速度 PID": "Rear-Wheel Speed PID",
    "速度模式": "Speed Mode",
    "目标 pps": "Target PPS",
    "应用 PID 参数": "Apply PID Settings",
    "摄像头云台（SG90）": "Camera Gimbal (SG90)",
    "摄像头回正": "Center Camera",
    "中位角度": "Center Angle",
    "Q/E 速度 °/s": "Q/E Speed °/s",
    "平滑加速度 °/s²": "Smooth Acceleration °/s²",
    "反转 Q/E 方向": "Reverse Q/E Direction",
    "保存云台设置": "Save Gimbal Settings",
    "附加：即时遥控与紧急停止": "Optional: Direct Control and Emergency Stop",
    "终端同款控制": "Terminal-Style Control",
    "紧急停止": "Emergency Stop",
    "摄像头左调": "Camera Left",
    "摄像头右调": "Camera Right",
    "慢速原地左转": "Slow Pivot Left",
    "慢速原地右转": "Slow Pivot Right",
    "慢速前进": "Slow Forward",
    "原地左转": "Pivot Left",
    "原地右转": "Pivot Right",
    "前左弯": "Forward Left",
    "前右弯": "Forward Right",
    "后左": "Reverse Left",
    "后右（仅接口）": "Reverse Right (API only)",
    "前进": "Forward",
    "后退": "Reverse",
    "回正": "Centered",
    "动作": "Action",
    "按键": "Keys",
    "控制台设置": "Console Settings",
    "相机": "Camera",
    "驱动": "Drive",
    "状态": "Status",
    "系统状态": "System Status",
    "重新连接 Arduino": "Reconnect Arduino",
    "Arduino 遥测": "Arduino Telemetry",
    "前向超声波": "Front Ultrasonic",
    "等待数据…": "Waiting for data…",
    "仅限制前进": "forward only",
    "轮速 L / R": "Wheel Speed L / R",
    "目标 L / R": "Target L / R",
    "采集 → 网页": "Capture → Browser",
    "传输档位": "Transport Profile",
    "采集 / 发送 FPS": "Capture / Send FPS",
    "单帧 JPEG": "JPEG per Frame",
    "实时编码": "Live Encoding",
    "实时帧延迟": "Live Frame Age",
    "MJPEG 发送带宽": "MJPEG Send Bandwidth",
    "包含所有网页客户端。": "Includes all browser clients.",
    "卡顿诊断": "Stutter Diagnostics",
    "测量中…": "Measuring…",
    "服务运行时长": "Service Uptime",
    "I2C 状态": "I2C Status",
    "WebRTC 接收": "WebRTC Reception",
    "最近回包：": "Latest Reply: ",
    "自动曝光": "Auto Exposure",
    "固定快门": "Fixed Shutter",
    "自动模式下 EV 生效。": "EV applies in auto mode.",
    "边缘偏色校正": "Edge Color Correction",
    "应用帧率": "Apply FPS",
    "应用": "Apply",
    "已连接": "Connected",
    "连接状态：": "Connection: ",
    "在线": "Online",
    "离线": "Offline",
    "无响应": "No Response",
    "运行中": "Running",
    "不可用": "Unavailable",
    "已关闭": "Disabled",
    "等待": "Waiting",
    "正常": "Normal",
    "车头": "Front",
    "左": "Left",
    "右": "Right"
  };
  const phrases = Object.keys(dictionary).sort((a, b) => b.length - a.length);

  function render(source) {
    if (language !== "en" || !source) return source;
    let result = source;
    for (const phrase of phrases) result = result.split(phrase).join(dictionary[phrase]);
    return result;
  }

  function syncText(node) {
    const previous = originalText.get(node);
    const source = previous && node.nodeValue === previous.rendered ? previous.source : node.nodeValue;
    const translated = render(source);
    originalText.set(node, { source, rendered: translated });
    if (node.nodeValue !== translated) node.nodeValue = translated;
  }

  function syncAttribute(element, attribute) {
    let attributes = originalAttributes.get(element);
    if (!attributes) {
      attributes = new Map();
      originalAttributes.set(element, attributes);
    }
    const current = element.getAttribute(attribute);
    if (current === null) return;
    const previous = attributes.get(attribute);
    const source = previous && current === previous.rendered ? previous.source : current;
    const translated = render(source);
    attributes.set(attribute, { source, rendered: translated });
    if (current !== translated) element.setAttribute(attribute, translated);
  }

  function localize(root) {
    if (!root || root.nodeType === Node.TEXT_NODE) {
      if (root) syncText(root);
      return;
    }
    if (root.nodeType !== Node.ELEMENT_NODE && root.nodeType !== Node.DOCUMENT_NODE) return;
    const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
    const nodes = [];
    while (walker.nextNode()) nodes.push(walker.currentNode);
    nodes.forEach(syncText);
    const elements = root.querySelectorAll ? [root, ...root.querySelectorAll("*")] : [];
    elements.forEach(element => {
      if (element.nodeType !== Node.ELEMENT_NODE || ["SCRIPT", "STYLE"].includes(element.tagName)) return;
      ["aria-label", "alt", "title", "placeholder"].forEach(attribute => {
        if (element.hasAttribute(attribute)) syncAttribute(element, attribute);
      });
    });
  }

  function applyLanguage() {
    document.documentElement.lang = language === "en" ? "en" : "zh-CN";
    document.title = language === "en" ? "Robitics Control Console" : "Robitics 控制台";
    localize(document.body);
    const button = document.getElementById("languageToggle");
    if (button) {
      button.textContent = language === "en" ? "中文" : "EN";
      button.setAttribute("aria-label", language === "en" ? "Switch to Chinese" : "切换为英文");
    }
  }

  function initialize() {
    const button = document.getElementById("languageToggle");
    button?.addEventListener("click", () => {
      language = language === "en" ? "zh" : "en";
      localStorage.setItem(storageKey, language);
      applyLanguage();
    });
    const observer = new MutationObserver(records => {
      if (language !== "en") return;
      records.forEach(record => {
        if (record.type === "characterData") syncText(record.target);
        if (record.type === "childList") record.addedNodes.forEach(localize);
      });
    });
    observer.observe(document.body, { childList: true, subtree: true, characterData: true });
    applyLanguage();
  }

  window.RobiticsI18n = { get language() { return language; }, applyLanguage };
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", initialize, { once: true });
  else initialize();
})();
