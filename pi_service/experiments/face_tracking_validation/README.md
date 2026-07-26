# Haar 人脸灵敏度探索（隔离，不控制小车）

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

## 电脑端位置数据服务（给后续 Pi 功能复用）

MediaPipe 必须运行在安装了它的电脑上，而非当前 Python 3.13 的树莓派。电脑读取 Pi 的
相机流后，可通过 HTTP 发布人脸相对相机的位置；它只发布 JSON，不控制小车：

```powershell
cd C:\Users\32126\Desktop\Robitics\pi_service\experiments\face_tracking_validation
py -3 face_position_server.py --source http://100.80.46.54:5000/video_feed --port 5059
```

数据地址为 `http://<电脑局域网IP>:5059/api/face/latest`，典型返回字段包括
`detected`、`offset_x`、`offset_y`、归一化偏移、框尺寸和置信度。正 `offset_x` 表示人脸在
相机画面右侧，负值表示左侧。未来 Pi 若接入此地址，也必须把它当作只读传感器；电机门控仍由
Pi 的 M 键和现有安全逻辑负责。

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

这不是路线、停车或转向功能。结束时按 `Ctrl+C` 即可；5000 主服务与电机状态不受影响。
