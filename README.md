# Robitics

正式控制链路：浏览器 → 树莓派网页服务 → USB 串口 → Arduino Mega。

- Arduino 在 `firmware/motor_bridge/` 中执行电机输出、后轮编码器 PID、1 秒心跳停车、MPU-6500 遥测和一个前向超声波测距。
- 树莓派服务在 `pi_service/` 中执行串口协议转换、CSI 视频和网页 API。
- 浏览器仅显示/控制；图片保存是可选的本机功能，绝不写入树莓派。

## 项目结构

日常部署只需要关注下列正式主线；完整边界见 [docs/architecture.md](docs/architecture.md)。

给同事交接时，请先阅读 [docs/TEAM_HANDOVER.md](docs/TEAM_HANDOVER.md)：它列出正式部署入口、全部小车相关页面、验证页面边界、配置文件和资料归档位置。

```text
firmware/motor_bridge/     正式 Arduino Mega 固件（唯一日常烧录目标）
pi_service/robot_web/      正式树莓派网页、相机和串口服务
pi_service/tests/          正式服务回归测试
docs/                      协议与架构文档
tools/windows_recorder/    Windows 端 MJPEG 图片记录工具
```

`firmware/experiments/` 与 `pi_service/experiments/` 仅用于硬件或算法验证，不能替代正式部署。`archive/` 保存历史资料、快照和采集证据；`hardware/cad/` 保存机械设计；`deskmate/` 和 `third_party/` 是独立 Git 项目。

## 部署

1. 用 Arduino IDE 打开并烧录 `firmware/motor_bridge/motor_bridge_1_/motor_bridge_1_.ino`。该目录中的两个 `.h` 文件必须与草图保留在同一目录。
2. 将 `pi_service/` 复制到树莓派，执行 `./install_dependencies.sh` 一次。
3. 执行 `chmod +x start_robot.sh && ./start_robot.sh`，随后打开输出的网址。脚本使用 Bash，不能用 `sh start_robot.sh` 启动。

`start_robot.sh` 是日常正式入口：`camera.py` 同时配置 CSI 主画面与 640×480 lores 输出，低延迟 MJPEG 直接读取 lores。高清 JPEG 默认 `2 FPS / 质量 75`，可在网页调为 `1–15 FPS`；关闭高清预览时不持续编码该通道。

当前默认路线模式为 `pc_vision_adaptor`：树莓派只做底部近场白胶带跟随、网页预览、M 键门控和唯一的 M1/M2 PWM 输出；电脑端通过 `/api/vision-adaptor/frame` 拉取最新 JPEG，在本机运行绿地、红标、骨架和路口等重型识别，并仅向 `/api/vision-adaptor/event` 回传带帧号、时间和 token 的高层视觉事件。电脑端不能调用 PWM，也不能绕过 M 键。协议、电脑端启动命令、失联安全策略和测试见 [pc_vision_adaptor_validation](pi_service/experiments/pc_vision_adaptor_validation/README.md)。

## 低延迟 H.264 / WebRTC 视频

默认 `start_robot.sh` 仍是 MJPEG 兼容模式。需要低延迟连续视频时，使用 H.264/WebRTC：CSI 由 `rpicam-vid` 独占并以 H.264 编码，MediaMTX 分发 WebRTC 给网页、RTSP 给电脑 DL；Flask 只保留控制、串口和状态 API。

```bash
cd ~/Desktop/Robitics/pi_service
chmod +x install_webrtc.sh start_webrtc.sh
./install_webrtc.sh        # 一次性下载匹配系统架构的 MediaMTX
./start_webrtc.sh
```

默认采用单 CSI 相机双输出：`1640×1232` 主画面每秒编码 2 张 JPEG 75，`640×480 / 30 FPS / 1.5 Mbps` 低分辨率画面编码为 H.264/WebRTC。实时 H.264 使用 Baseline、每 8 帧一个关键帧（30 FPS 时约 267 ms）和立即刷包的 MPEG-TS 输出，以缩短固定缓冲。网页仍访问 `http://树莓派IP:5000`，左侧为 `http://树莓派IP:8889/cam/` 的 WebRTC 预览，右侧为高清 JPEG；电脑端 DL 可读取 `rtsp://树莓派IP:8554/cam`。按 `Ctrl+C` 会停止 Flask、Picamera2 双输出和 MediaMTX。日志保存在 `pi_service/logs/`。

服务启动约 3 秒后，执行自动验收：

```bash
chmod +x verify_webrtc.sh
./verify_webrtc.sh
```

它必须输出 `Flask backend: webrtc`、高清 JPEG 目标与 `PASS`。若 `Camera online: False`，先查看 `logs/mediamtx.log` 和运行 `start_webrtc.sh` 的终端输出；若命令通过但电脑无法播放，检查热点网络是否允许树莓派的 TCP 8889 和 WebRTC UDP 通信。

可在启动前调节参数，例如：

```bash
ROBOT_WEBRTC_WIDTH=640 ROBOT_WEBRTC_HEIGHT=480 ROBOT_WEBRTC_BITRATE=1500000 ROBOT_WEBRTC_GOP_FRAMES=8 \
ROBOT_HIGHRES_WIDTH=1640 ROBOT_HIGHRES_HEIGHT=1232 ./start_webrtc.sh
```

WebRTC 模式不能同时运行 `start_robot.sh`，也不会支持现有 MJPEG 的浏览器 Canvas 裁切、浏览器 JPG 保存或网页 EV/快门动态调节；这些功能仍可通过 `start_robot.sh` 使用。WebRTC 页面与 RTSP 使用同一条低分辨率 H.264 流；高清 JPEG 通过 `/highres_feed` 和 `/api/camera/highres/latest` 独立提供，浏览器与 DL 可各自使用该接口，但每增加一个客户端都会增加该 JPEG 通道的网络流量。

## 重要约定

- 串口波特率为 9600；协议见 `docs/serial-protocol.md`。
- 电机分配为 M1 右侧行驶、M2 左侧行驶、M3 送牌、M4 出牌。M3/M4 上电后默认关闭；网页可分别实时设置 PWM（1–255）、正转/反转和运行时间（0.1–60 秒），无需为这些参数重复烧录。默认值为 M3 反转 255/5 秒、M4 正转 255/1 秒。网页保持 `W/A/S/D` 行驶控制；单击键盘 `P` 会通过同一按键控制通道同时按各自预设触发 M3 和 M4，并分别等待 Arduino 确认。
- 后轮编码器使用 Mega 的 18（左）和 19（右）；IMU 使用 MPU-6500 I2C。
- 超声波仅保留中间前向传感器（TRIG 26 / ECHO 27），返回 `US,front`；当前阈值为 1 cm，达到阈值时 Arduino 只拒绝前进，原地转向和后退不受限制。
- SG90 舵机信号线使用 Mega D23，网页滑块控制 `0–180°`；Mega D22 保留给 HW-487 卡片光电传感器。舵机必须独立稳定供电并与 Mega 共地。舵机命令不会延长电机心跳。
- 网页“轮速配置”、PWM 与 PID 参数保存在树莓派独立的 `drive_config.json`；该文件不进入 Git，也不会被后续代码更新覆盖。首次升级会从旧 `robot_config.json` 自动复制现有调参值。`drive_config.example.json` 仅是可提交的默认模板。
- 网页相机支持自动曝光 EV 和固定快门；快门以 `1/xx` 秒输入。自动曝光模式下 EV 生效，固定快门模式下关闭自动曝光。
- 日常网页预览的“低延迟”档位为 `640×480`、JPEG 质量 60，直接使用相机 lores 输出。高清 JPEG 通道默认 `2 FPS`、质量 75，可在网页调为 `1–15 FPS`，不在树莓派保存文件；DL 或电脑可用 `/highres_feed` 获取连续图片，或用 `/api/camera/highres/latest` 拉取最新单张。高清通道可在网页选择原始尺寸、最大 1640 px 或最大 1280 px，切换不重启相机。
- 网页采用“按住才动、松开即停”的按键状态协议：浏览器每 180 ms 经 HTTP/TCP 发送按键集合，树莓派每 200 ms 向 Arduino 发送电机命令。网页失焦、网络断开和 Arduino 1 秒收不到命令都会停车。
- 摄像头云台舵机由 Arduino 每 20 ms 平滑更新；网页 Q/E 只保持左/右方向心跳，断开后 Arduino 会平滑减速停止。网页的“Q/E 速度”是最大角速度，默认加速度为 `120 °/s²`。
- `Z` 或“摄像头回正”使用独立快速回正命令，直接以舵机自身的机械最大速度回到网页配置的中位角度；它不会受 Q/E 速度限制。
- Arduino 每 200 ms 回传一次实际云台角度；网页会用该读数校正云台图示，并在两次读数之间平滑动画。
- 网页 CSI 视频卡片会显示实时分辨率、采集/发送 FPS、JPEG 单帧大小、编码耗时、最新帧延迟以及 MJPEG 实际发送带宽；带宽同时显示十进制 `kB/s` 和 `kbps`。
- 网页“传输档位”与“相机读取档位”分开：相机仍以选择的 CSI 分辨率和 30 FPS 采集，树莓派只将缩放后的 JPEG 最新帧发给网页。默认“低延迟”为 `640×480`、JPEG 质量 60，显著降低热点带宽且不新增队列/缓存；需要高清图片时使用独立的 2 FPS 高清 JPEG 通道。上述档位会保存到 `camera_config.json`。
- 默认网页视频仍是低延迟 MJPEG；通过 `start_webrtc.sh` 可切换至 H.264/WebRTC 独立媒体服务，不能与 MJPEG 模式同时启动。
- 视频卡片中的“电脑端辅助画面”使用浏览器 Canvas 复用当前视频帧，可完整缩放或中心裁切到 640×480；它不创建第二条视频连接，不增加树莓派网络带宽和编码负载。
- 网页可在 `1640×1232` 与 `3280×2464` 两个 CSI 读取档位之间切换；两个档位都请求 30 FPS 传感器帧时长，切换会短暂重启视频流，实际采集/编码帧率以网页指标为准。
- 本地图片保存需要 Chrome/Edge 的 HTTPS 或 localhost 安全上下文，并由用户选择文件夹授权。
- OLED 为可选的 SSD1306 `64×48` I2C 模块：GND→pin 6、VCC→pin 1（3.3V，推荐）或 pin 2（5V）、SDA→GPIO2/pin 3、SCL→GPIO3/pin 5；先用 `sudo raspi-config` 开启 I2C。默认地址为 `0x3C`、总线 1，服务启动后每秒显示 Arduino、相机和前向距离。OLED 缺失、I2C 未开或依赖未装不会阻止控制服务启动，网页“状态”标签会显示错误原因。可用 `--disable-oled` 关闭，或以 `--oled-address 0x3D --oled-i2c-port 1` 修改参数。

## Git 工作流

`main` 是唯一可部署分支；`archive/*` 仅保留旧版本。每次完成真实硬件验证后创建本地标签，例如 `robot-v0.1.0-verified`。

## Windows 本地 JPG 保存（5 FPS）

在 Windows PowerShell 中运行，替换为树莓派实际 IP：

```powershell
.\tools\windows_recorder\start_windows_recorder.ps1 -StreamUrl 'http://树莓派IP:5000/video_feed'
```

图片只会保存到 `data\captures\YYYY-MM-DD\`，不会上传或写回树莓派。按 `Ctrl+C` 停止；断线后每 3 秒自动重连，并在可用空间少于 5 GB 时安全停止。

浏览器内的“连续保存”也是 5 FPS，但需要 Chrome/Edge 的 HTTPS 或 localhost 文件夹授权；给队友使用时优先采用上述 Windows 记录器。
