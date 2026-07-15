# Robitics

正式控制链路：浏览器 → 树莓派网页服务 → USB 串口 → Arduino Mega。

- Arduino 在 `firmware/motor_bridge/` 中执行电机输出、后轮编码器 PID、1 秒心跳停车、MPU-6500 遥测和三路超声波测距。
- 树莓派服务在 `pi_service/` 中执行串口协议转换、CSI 视频和网页 API。
- 浏览器仅显示/控制；图片保存是可选的本机功能，绝不写入树莓派。

## 部署

1. 用 Arduino IDE 打开并烧录 `firmware/motor_bridge/motor_bridge_1_/motor_bridge_1_.ino`。该目录中的两个 `.h` 文件必须与草图保留在同一目录。
2. 将 `pi_service/` 复制到树莓派，执行 `./install_dependencies.sh` 一次。
3. 执行 `./start_robot.sh`，随后打开输出的网址。

## 重要约定

- 串口波特率为 9600；协议见 `docs/serial-protocol.md`。
- 后轮编码器使用 Mega 的 18（左）和 19（右）；IMU 使用 MPU-6500 I2C。
- 首版 `US,right,front,left` 只用于显示；不会自动拦截任何动作。
- 本地图片保存需要 Chrome/Edge 的 HTTPS 或 localhost 安全上下文，并由用户选择文件夹授权。

## Git 工作流

`main` 是唯一可部署分支；`archive/*` 仅保留旧版本。每次完成真实硬件验证后创建本地标签，例如 `robot-v0.1.0-verified`。

## Windows 本地 JPG 保存（5 FPS）

在 Windows PowerShell 中运行，替换为树莓派实际 IP：

```powershell
.\start_windows_recorder.ps1 -StreamUrl 'http://树莓派IP:5000/video_feed'
```

图片只会保存到 `Pic\YYYY-MM-DD\`，不会上传或写回树莓派。按 `Ctrl+C` 停止；断线后每 3 秒自动重连，并在可用空间少于 5 GB 时安全停止。
