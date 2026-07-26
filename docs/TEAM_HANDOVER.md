# Robitics 同事交接索引

本仓库的正式控制链路是：**浏览器 → 树莓派网页服务 → USB 串口 → Arduino Mega**。日常使用只从本页列出的“正式入口”开始；`experiments/` 中的内容用于单项验证，不能当作完整小车程序部署。

## 1. 交接时需要的目录

| 目录 | 用途 | 是否需要部署到树莓派 |
| --- | --- | --- |
| `firmware/motor_bridge/motor_bridge_1_/` | Arduino Mega 正式固件：M1/M2 行驶、M3/M4 发牌、云台、编码器 PID、IMU、超声波安全与串口协议 | 需要烧录到 Mega |
| `pi_service/robot_web/` | 正式 Flask 控制台、相机、串口控制、自动路线适配器和网页静态资源 | 需要 |
| `pi_service/start_robot.sh` | MJPEG 日常启动入口（默认端口 5000） | 需要 |
| `pi_service/install_dependencies.sh` | Pi 上的一次性依赖安装 | 需要 |
| `pi_service/start_webrtc.sh`、`install_webrtc.sh` | 可选 H.264/WebRTC 启动方式；不能和 `start_robot.sh` 同时运行 | 按需 |
| `docs/` | 协议、架构和本交接索引 | 建议一并保留 |
| `tools/windows_recorder/` | Windows 电脑端录像/逐帧 JPG 保存工具 | 仅 Windows 电脑 |

部署时将整个 `pi_service/` 复制到树莓派；不要只复制 `robot_web/`，因为启动脚本、依赖脚本、测试和实验说明都在其上层目录。

## 2. 所有小车相关网页/页面

### 正式日常控制台

- 页面源码：`pi_service/robot_web/templates/index.html`
- 前端逻辑：`pi_service/robot_web/static/app.js`
- 样式：`pi_service/robot_web/static/style.css`
- 后端入口：`pi_service/robot_web/app.py`
- 启动：在 Pi 的 `pi_service/` 目录执行 `chmod +x start_robot.sh && ./start_robot.sh`，浏览器打开 `http://<Pi-IP>:5000`。

控制台包含三组页面标签：

1. **相机**：低延迟 MJPEG、路线预判、按需高清 JPEG、曝光/传输档位与电脑端本地保存。
2. **行驶控制**：M1/M2 PWM、速度 PID、摄像头云台、M3 送牌/M4 出牌、W/A/S/D/Q/E/Z/P 等实时控制。
3. **系统状态**：Arduino、IMU、轮速、实际 M1/M2 PWM、相机指标、CPU/温度/OLED 状态。

安全规则：网页按键松开、失焦或断网会停车；树莓派心跳停止和 Arduino 一秒未收到指令也会停车。自动路线必须先按 **M** 解锁，且只允许主服务控制 M1/M2。

### 验证页面（保留，不作为正式交付入口）

| 页面/实验 | 路径 | 用途与边界 |
| --- | --- | --- |
| 小车整车测试 | `pi_service/experiments/car_drive_test/templates/index.html` | M1/M2 方向和 PWM 的单项检查；不能与正式控制台同时控制电机。 |
| 路线验证页面 | `pi_service/experiments/*/debug_web.py` | 直线、矩形、I 型、扫描线、终点转向等视觉/路线验证。先阅读各目录 `README.md`。 |
| 人脸跟踪服务 | `pi_service/experiments/face_tracking_validation/` | 独立相机/人脸识别验证，非小车主控功能。 |

## 3. 固件与电机对应关系

- `M1`：右侧行驶电机；`M2`：左侧行驶电机。
- `M3`：送牌；`M4`：出牌。
- 正式草图是 `motor_bridge_1_.ino`，同目录 `motor_control.h` 和 `ultrasonic_avoidance.h` 必须一起保留。
- 协议与命令说明见 [serial-protocol.md](serial-protocol.md)。Pi 端一键烧录工具见 `firmware/motor_bridge/motor_bridge_1_/pi_flash/`。

## 4. 配置与不可提交文件

Pi 上调出的真实参数会保存在以下本机文件，Git 特意忽略它们；交接时如需复用已调好的数值，请通过安全渠道单独复制，不能把其中的设备信息或密码推送到 GitHub：

- `pi_service/robot_web/drive_config.json`
- `pi_service/robot_web/camera_config.json`
- `pi_service/robot_web/robot_config.json`

可提交的默认参考是 `drive_config.example.json`。同事第一次部署前应先确认串口设备、CSI 相机和 Arduino 实际接线。

## 5. 资料归档与仓库边界

- `hardware/cad/`：机械 CAD、STL、装配体；只做建模/打印时使用。
- `archive/`：课程材料、旧控制程序、旧发布包和采集证据；保留可追溯性，不参与日常部署。
- `artifacts/`：离线验证产物。
- `deskmate/` 和 `third_party/`：**独立 Git 仓库**，不属于 Robitics 主仓库推送范围，也不应复制进本项目历史。

## 6. 推送前检查

```bash
git status -sb
git diff --check
python3 -m unittest discover -s pi_service/tests -v
```

推送前确认没有 `drive_config.json`、`camera_config.json`、日志、录像、账号令牌或私钥被暂存。当前 GitHub 主仓库：<https://github.com/YYiChen/Robitics>。
