# Robitics

正式控制链路：浏览器 → 树莓派网页服务 → USB 串口 → Arduino Mega。

- Arduino 在 `firmware/motor_bridge/` 中执行电机输出、后轮编码器 PID、1 秒心跳停车、MPU-6500 遥测和一个前向超声波测距。
- 树莓派服务在 `pi_service/` 中执行串口协议转换、CSI 视频和网页 API。
- 浏览器仅显示/控制；图片保存是可选的本机功能，绝不写入树莓派。

## 部署

1. 用 Arduino IDE 打开并烧录 `firmware/motor_bridge/motor_bridge_1_/motor_bridge_1_.ino`。该目录中的两个 `.h` 文件必须与草图保留在同一目录。
2. 将 `pi_service/` 复制到树莓派，执行 `./install_dependencies.sh` 一次。
3. 执行 `chmod +x start_robot.sh && ./start_robot.sh`，随后打开输出的网址。脚本使用 Bash，不能用 `sh start_robot.sh` 启动。

## 重要约定

- 串口波特率为 9600；协议见 `docs/serial-protocol.md`。
- 后轮编码器使用 Mega 的 18（左）和 19（右）；IMU 使用 MPU-6500 I2C。
- 超声波返回 `US,-1,front,-1`；前方距离小于等于 15 cm 时 Arduino 只拒绝前进，原地转向和后退不受限制。
- 网页“轮速配置”支持每个动作的四轮同步、左右两侧同步和四轮独立微调；配置保存在树莓派 `robot_config.json`，点击应用后立即保存，服务通过 Ctrl+C 或正常停止时也会再次落盘，重启后自动恢复。
- 网页采用“按住才动、松开即停”的按键状态协议：浏览器每 180 ms 经 HTTP/TCP 发送按键集合，树莓派每 200 ms 向 Arduino 发送电机命令。网页失焦、网络断开和 Arduino 1 秒收不到命令都会停车。
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
