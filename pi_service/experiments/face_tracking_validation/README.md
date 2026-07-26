# 人脸转向桥接与历史隔离验证

本目录保留早期5058 MediaPipe纯观测实验，但正式车控不再使用该检测器。当前唯一车控
检测器是相邻目录 `deskmate_face_position_bridge/deskmate_face_position_server.py`
调用的DeskMate YuNet/SFace。正式运行时，该文件同时内嵌本目录可复用的
`FaceTurnBridge`，直接读取内存快照并转换成Pi正式动作API的心跳和停车。

## 树莓派运行

先确保主服务的相机预览可访问，并在当前 Python 环境安装 `mediapipe`，再运行：

```bash
cd /home/g11/Desktop/pi_service/experiments/face_tracking_validation
python3 -m unittest test_face_detector -v
./run_pi_face_probe.sh
```

浏览器打开：`http://127.0.0.1:5058`。

## 唯一的人脸转向链路

只保留这一套车控链路：电脑上的 `deskmate_face_position_server.py` 使用
DeskMate YuNet/SFace做唯一一次推理；
`deskmate_face_position_server.py` 内嵌的桥接线程在网页或状态机已经启动J/L转向后，
通过正式的 `/api/robotics/v1/status` 和 `/api/robotics/v1/actions` 向Pi发送
`face_turn_heartbeat` 或 `face_turn_stop`。每次心跳使用独立 `request_id`，
确保Pi的幂等缓存不会吞掉后续续租。
历史5058探测程序不参与电机控制，也不会和YuNet/SFace同时向Pi发动作。

当前车控人脸摄像头为 `http://100.93.97.117:4747/video`。先在一个终端启动检测：

```powershell
cd C:\Users\32126\Desktop\Robitics
py -3 .\pi_service\experiments\deskmate_face_position_bridge\deskmate_face_position_server.py --source http://100.93.97.117:4747/video --port 5059 --pi-url http://100.80.46.54:5000
```

不再需要第二个终端或独立桥接进程。此后在5000网页先按M，再按J或L；状态机也可用
`face_turn_start` 动作进入相同状态。只有 Pi 已进入 `FACE_CENTER_TURN` 时桥接才会工作：
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
