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
    "预览关闭 · 不占用网页高清传输带宽": "Preview Off · No high-resolution browser bandwidth in use",
    "点击高清画面右上角“开启预览”后再订阅。": "Click Enable Preview at the top right of the high-resolution view to subscribe.",
    "当前为 MJPEG 模式": "MJPEG mode is active",
    "未发现明显传输瓶颈": "No obvious transmission bottleneck found",
    "更像树莓派采集/编码瓶颈": "Likely Raspberry Pi capture/encoding bottleneck",
    "树莓派编码": "Raspberry Pi encoding",
    "本浏览器只收到": "this browser receives only",
    "若操作仍卡，重点检查控制请求和浏览器负载。": "If controls still lag, check control requests and browser load.",
    "浏览器状态：": "Browser status: ",
    "请观察接收 FPS、码率和抖动缓冲。": "Check receive FPS, bitrate, and jitter buffer.",
    "当前 WebRTC 模式不可用": "WebRTC mode is currently unavailable",
    "无有效回波（-1）": "No valid echo (-1)",
    "无有效回波": "No valid echo",
    "前进限位": "Forward blocked",
    "通行": "Clear",
    "每秒刷新状态": "status refreshes every second",
    "检查 I2C、地址和 luma.oled 依赖": "Check I2C, address, and luma.oled dependency",
    "后端未提供 system/capabilities": "backend does not provide system/capabilities",
    "当前后端不支持，需同步 camera.py": "current backend is unsupported; synchronize camera.py",
    "旧版本，需同步 app.py": "older version; synchronize app.py",
    "旧后端未报告": "not reported by older backend",
    "轮速配置已应用并保存": "Wheel speed profile applied and saved",
    "高清图片帧率设置失败": "Failed to set high-resolution image FPS",
    "相机模式切换失败": "Failed to switch camera mode",
    "自动模式下 EV 调整亮度；快门由相机自动决定。": "In auto mode, EV adjusts brightness; shutter is selected by the camera.",
    "固定快门使用 1/xx 秒；此时自动 EV 不参与曝光。": "Fixed shutter uses 1/xx seconds; auto EV does not affect exposure.",
    "已应用自动曝光：": "Auto exposure applied: ",
    "已固定快门：": "Fixed shutter: ",
    "高清流断开": "High-resolution stream disconnected",
    "请用 Chrome/Edge 通过 HTTPS 或 localhost 打开。": "Open with Chrome/Edge over HTTPS or localhost.",
    "未获得文件夹权限": "Folder permission was not granted",
    "保存目标：": "Save destination: ",
    "视频尚未就绪": "Video is not ready",
    "已保存": "Saved ",
    "张到电脑": " images to computer",
    "视频传输档位切换失败": "Failed to switch video transport profile",
    "高清图片档位切换失败": "Failed to switch high-resolution image profile",
    "高清 JPEG 已设为": "High-resolution JPEG set to ",
    "该设置会保存。": "This setting will be saved.",
    "曝光设置失败": "Failed to apply exposure settings",
    "颜色校正设置失败": "Failed to apply color correction",
    "已开启边缘偏色校正，强度": "Edge color correction enabled, strength ",
    "已关闭边缘偏色校正": "Edge color correction disabled",
    "动作发送失败": "Failed to send action",
    "舵机指令失败": "Servo command failed",
    "云台状态未知": "Gimbal status unknown",
    "云台指令": "Gimbal command",
    "摄像头快速回正失败": "Failed to center camera quickly",
    "摄像头正在快速回正": "Camera is centering quickly",
    "转向设置保存失败": "Failed to save steering settings",
    "转向设置已保存": "Steering settings saved",
    "PID 参数提交失败": "Failed to apply PID settings",
    "Arduino 未收到回包": "Arduino did not return a reply",
    "Arduino 已重新连接并收到回包": "Arduino reconnected and replied",
    "浏览器 WebRTC 缓冲偏高": "Browser WebRTC buffer is high",
    "WebRTC 存在丢包": "WebRTC packet loss detected",
    "树莓派相机服务离线": "Raspberry Pi camera service offline",
    "更像树莓派 JPEG 编码压力": "Likely Raspberry Pi JPEG encoding pressure",
    "更像网络往返延迟": "Likely network round-trip latency",
    "网络传输或浏览器解码": "network transport or browser decoding",
    "浏览器解码/渲染": "browser decoding/rendering",
    "采集": "Capture",
    "网页": "Browser",
    "传感器目标": "Sensor Target",
    "实际编码": "Actual Encoding",
    "目标码率": "Target Bitrate",
    "所有网页合计": "all browser clients",
    "同时显示 kB/s 和 kbps，包含 MJPEG 分片头；单个网页时就是当前流量": "Shows kB/s and kbps, including MJPEG multipart headers; with one browser this is the current traffic.",
    "连续 H.264 帧": "Continuous H.264 Frames",
    "由 WebRTC 自适应": "WebRTC Adaptive",
    "单张": "Per Image",
    "客户端": "Clients",
    "请使用 start_robot.sh 启动 MJPEG 服务。": "Use start_robot.sh to start the MJPEG service.",
    "抖动缓冲": "Jitter Buffer",
    "累计丢包": "Packets Lost",
    "浏览器丢帧": "Browser Frames Dropped",
    "相机缓冲": "Camera Buffer",
    "切换到 start_webrtc.sh 后显示浏览器 WebRTC 接收统计。": "After switching to start_webrtc.sh, browser WebRTC reception statistics will be shown.",
    "MJPEG 专用": "MJPEG Only",
    "后端未更新": "Backend not updated",
    "已超时停车": "Stopped on timeout",
    "请检查前向超声波回波": "Check the front ultrasonic echo",
    "距障碍": "Distance to obstacle",
    "前进已限制": "Forward restricted",
    "安全距离": "Safe distance",
    "网页后端连接失败：": "Web backend connection failed: ",
    "网页服务异常": "Web service error",
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
    "高清图片帧率": "High-Resolution Image FPS",
    "高清平衡": "High-Resolution Balanced",
    "高清轻量": "High-Resolution Compact",
    "低延迟": "Low Latency",
    "平衡": "Balanced",
    "原始尺寸": "Source Size",
    "最大": "Maximum",
    "最宽": "Max Width",
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
    "左右同步": "Both Drive Sides",
    "左右独立": "Left / Right Independently",
    "端口明细": "Port Details",
    "统一 PWM": "Uniform PWM",
    "左侧": "Left Side",
    "右侧": "Right Side",
    "发送": "Output",
    "应用并保存当前动作": "Apply and Save Current Action",
    "后轮速度 PID": "Rear-Wheel Speed PID",
    "左右行驶速度 PID": "Left / Right Drive PID",
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
    "负载": "Load",
    "内存": "Memory",
    "温度": "Temperature",
    "磁盘": "Disk",
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
    "后端:": "Backend: ",
    "系统指标:": "System Metrics: ",
    "高清帧率:": "High-Resolution FPS: ",
    "驱动配置:": "Drive Config: ",
    "配置路径:": "Config Path: ",
    "配置读取错误:": "Config Read Error: ",
    "串口:": "Serial: ",
    "最近回包:": "Last Reply: ",
    "动作:": "Action: ",
    "按键:": "Keys: ",
    "回复:": "Reply: ",
    "错误:": "Error: ",
    "已打开": "Open",
    "未打开": "Closed",
    "可调整": "Adjustable",
    "无": "None",
    "自动曝光": "Auto Exposure",
    "固定快门": "Fixed Shutter",
    "曝光": "Exposure",
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
    "秒": "s",
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
