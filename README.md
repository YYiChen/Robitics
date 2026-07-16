# Robitics

正式控制链路：浏览器 → 树莓派网页服务 → USB 串口 → Arduino Mega。

- Arduino 在 `firmware/motor_bridge/` 中执行电机输出、后轮编码器 PID、1 秒心跳停车、MPU-6500 遥测和一个前向超声波测距。
- 树莓派服务在 `pi_service/` 中执行串口协议转换、CSI 视频和网页 API。
- 浏览器仅显示/控制；图片保存是可选的本机功能，绝不写入树莓派。

## 部署

1. 用 Arduino IDE 打开并烧录 `firmware/motor_bridge/motor_bridge_1_/motor_bridge_1_.ino`。该目录中的两个 `.h` 文件必须与草图保留在同一目录。
2. 将 `pi_service/` 复制到树莓派，执行 `./install_dependencies.sh` 一次。
3. 执行 `chmod +x start_robot.sh && ./start_robot.sh`，随后打开输出的网址。脚本使用 Bash，不能用 `sh start_robot.sh` 启动。

## 低延迟 H.264 / WebRTC 视频

默认 `start_robot.sh` 仍是 MJPEG 兼容模式。需要低延迟连续视频时，使用 H.264/WebRTC：CSI 由 `rpicam-vid` 独占并以 H.264 编码，MediaMTX 分发 WebRTC 给网页、RTSP 给电脑 DL；Flask 只保留控制、串口和状态 API。

```bash
cd ~/Desktop/Robitics/pi_service
chmod +x install_webrtc.sh start_webrtc.sh
./install_webrtc.sh        # 一次性下载匹配系统架构的 MediaMTX
./start_webrtc.sh
```

首次参数为 `1280×720 / 30 FPS / 2.5 Mbps / 15 帧关键帧`。网页仍访问 `http://树莓派IP:5000`，它会自动嵌入 `http://树莓派IP:8889/cam/` 的 WebRTC 预览；电脑端 DL 可读取 `rtsp://树莓派IP:8554/cam`。按 `Ctrl+C` 会同时停止 Flask、`rpicam-vid` 与 MediaMTX。日志保存在 `pi_service/logs/`。

可在启动前调节参数，例如：

```bash
ROBOT_WEBRTC_WIDTH=1640 ROBOT_WEBRTC_HEIGHT=1232 ROBOT_WEBRTC_BITRATE=4000000 ./start_webrtc.sh
```

WebRTC 模式不能同时运行 `start_robot.sh`，也不会支持现有 MJPEG 的浏览器 Canvas 裁切、浏览器 JPG 保存或网页 EV/快门动态调节；这些功能仍可通过 `start_robot.sh` 使用。WebRTC 页面和 RTSP 使用同一条 H.264 流，DL 不应再单独拉取 `/video_feed`。

## 重要约定

- 串口波特率为 9600；协议见 `docs/serial-protocol.md`。
- 后轮编码器使用 Mega 的 18（左）和 19（右）；IMU 使用 MPU-6500 I2C。
- 超声波仅保留中间前向传感器（TRIG 26 / ECHO 27），返回 `US,front`；前方距离小于等于 30 cm 时 Arduino 只拒绝前进，原地转向和后退不受限制。
- SG90 舵机信号线使用 Mega D22，网页滑块控制 `0–180°`；舵机必须独立稳定供电并与 Mega 共地。舵机命令不会延长电机心跳。
- 网页“轮速配置”、PWM 与 PID 参数保存在树莓派独立的 `drive_config.json`；该文件不进入 Git，也不会被后续代码更新覆盖。首次升级会从旧 `robot_config.json` 自动复制现有调参值。`drive_config.example.json` 仅是可提交的默认模板。
- 网页相机支持自动曝光 EV 和固定快门；快门以 `1/xx` 秒输入。自动曝光模式下 EV 生效，固定快门模式下关闭自动曝光。
- 网页采用“按住才动、松开即停”的按键状态协议：浏览器每 180 ms 经 HTTP/TCP 发送按键集合，树莓派每 200 ms 向 Arduino 发送电机命令。网页失焦、网络断开和 Arduino 1 秒收不到命令都会停车。
- 网页 CSI 视频卡片会显示实时分辨率、采集/发送 FPS、JPEG 单帧大小、编码耗时、最新帧延迟以及 MJPEG 实际发送带宽；带宽同时显示十进制 `kB/s` 和 `kbps`。
- 网页“传输档位”与“相机读取档位”分开：相机仍以选择的 CSI 分辨率和 30 FPS 采集，树莓派只将缩放后的 JPEG 最新帧发给网页。默认“低延迟”为最多 `820×616`、JPEG 质量 70，显著降低热点带宽且不新增队列/缓存；需要全分辨率截图或 DL 原图时再切换“原始尺寸”。此项会保存到 `camera_config.json`。
- 当前网页视频仍是低延迟 MJPEG，不是 H.264/WebRTC。H.264/WebRTC 能进一步节省带宽，但需要独立媒体服务与浏览器播放链路，不能仅替换 Flask 的一个响应头；后续应在保持此控制服务不变的前提下单独部署。
- 视频卡片中的“电脑端辅助画面”使用浏览器 Canvas 复用当前视频帧，可完整缩放或中心裁切到 640×480；它不创建第二条视频连接，不增加树莓派网络带宽和编码负载。
- 网页可在 `1640×1232` 与 `3280×2464` 两个 CSI 读取档位之间切换；两个档位都请求 30 FPS 传感器帧时长，切换会短暂重启视频流，实际采集/编码帧率以网页指标为准。
- 本地图片保存需要 Chrome/Edge 的 HTTPS 或 localhost 安全上下文，并由用户选择文件夹授权。

## Git 工作流

`main` 是唯一可部署分支；`archive/*` 仅保留旧版本。每次完成真实硬件验证后创建本地标签，例如 `robot-v0.1.0-verified`。

## Windows 本地 JPG 保存（5 FPS）

在 Windows PowerShell 中运行，替换为树莓派实际 IP：

```powershell
.\start_windows_recorder.ps1 -StreamUrl 'http://树莓派IP:5000/video_feed'
```

图片只会保存到 `Pic\YYYY-MM-DD\`，不会上传或写回树莓派。按 `Ctrl+C` 停止；断线后每 3 秒自动重连，并在可用空间少于 5 GB 时安全停止。

浏览器内的“连续保存”也是 5 FPS，但需要 Chrome/Edge 的 HTTPS 或 localhost 文件夹授权；给队友使用时优先采用上述 Windows 记录器。
