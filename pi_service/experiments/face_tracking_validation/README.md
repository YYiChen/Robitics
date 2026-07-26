# MediaPipe 人脸转向（隔离验证）

此实验只读取已经运行的 5000 相机流，在 5058 画出人脸框并记录 JSONL。核心检测器使用
MediaPipe Tasks 的 BlazeFace short-range 模型；它不导入 `robot_web.controller`，不发送 HTTP
电机指令，也不响应 M 键。

## 树莓派运行

先确保主服务的相机预览可访问，并在当前 Python 环境安装 `mediapipe`，再运行：

```bash
cd /home/g11/Desktop/pi_service/experiments/face_tracking_validation
python3 -m unittest test_face_detector -v
./run_pi_face_probe.sh
```

浏览器打开：`http://127.0.0.1:5058`。

## 唯一的人脸转向链路

只保留这一套链路：电脑上的 `face_position_server.py` 使用 MediaPipe BlazeFace 做唯一一次推理；
`face_turn_web_bridge.py` 只读取其 JSON，并在网页 J/L 已启动后向 Pi 发送心跳或居中停车。
它们不是两套检测器，且没有使用 DeskMate 的 BD05/M9 模型。

当前车控人脸摄像头固定为 `http://10.157.23.223:4747/video`。先在一个终端启动检测：

```powershell
cd C:\Users\32126\Desktop\Robitics\pi_service\experiments\face_tracking_validation
py -3 face_position_server.py --port 5059
```

再在第二个终端启动桥接：

```powershell
py -3 face_turn_web_bridge.py --face-url http://127.0.0.1:5059/api/face/latest --pi-url http://100.80.46.54:5000
```

此后在 5000 网页先按 M，再按 J 或 L。只有 Pi 已进入 `FACE_CENTER_TURN` 时桥接才会工作：
人脸未居中时持续 `HEARTBEAT`，人脸居中后发送 `STOP`。检测流或网络异常时不续租，Pi 会按安全超时停车。

数据地址为 `http://127.0.0.1:5059/api/face/latest`，字段包括 `detected`、`offset_x`、
归一化偏移、框尺寸和置信度。正 `offset_x` 表示人脸在画面右侧，负值表示左侧。

## 电脑端双相机综合分析

电脑会并行分析 Pi 相机和手机 IP Webcam；两路偏移均保留，综合结果选择当前人脸框更大的
主观测，**不会错误地平均两个相机的像素偏移**。手机与电脑需在同一局域网：

```powershell
py -3 multi_camera_face_position_server.py `
  --pi-source http://100.80.46.54:5000/video_feed `
  --phone-source http://10.50.77.86:8080/video `
  --port 5060
```

浏览器打开 `http://127.0.0.1:5060/` 可并列观看两个带检测框的画面；
`http://10.50.77.205:5060/api/faces/latest` 是供程序读取的 JSON。`sources.pi` 与 `sources.phone`
是各自原始检测；`fused` 是选择出的主观测和选择原因。若未来要算真实空间坐标，需要额外完成
双相机标定。

## 怎么测“最远可检出距离”

1. 面向镜头站立，从近到远每次移动 0.5 米；不要让人脸偏向或被遮挡。
2. 每个距离保持约 5 秒，记录是否连续绿框、页面的“近 5 秒检出率”、框宽（px）和耗时。
3. `runtime_logs/face_probe.jsonl` 保存每帧 `detected`、框宽/面积、中心坐标和处理时间。
4. 将“连续检出率明显下降前的最远距离”作为探索结果；不要把框宽直接当作精确距离。

旧的 `face_track_turn.py` 直接检测并控车，与本链路重复，已移除，避免两个进程同时对 Pi 发人脸转向命令。
