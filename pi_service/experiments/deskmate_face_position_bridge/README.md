# DeskMate-Advance YuNet 人脸位置桥接

这是一个电脑端隔离实验。它调用根目录 Git submodule
`subrepos/DeskMate-Advance` 的正式摄像头与人脸模型接口：

- `poker_dealer.io.camera.OpenCVCamera` 读取树莓派 MJPEG，并只保留最新帧；
- `OpenCvFaceIdentityAdapter` 使用 YuNet 检测和 SFace 特征模型；
- 本实验只发布人脸框、中心偏移和置信度，不保存画面或人脸特征；
- 本实验不导入机器人控制器，也不直接发送电机指令。

## 数据链路

```text
Pi 5000/video_feed
  -> PC OpenCVCamera
  -> DeskMate YuNet/SFace
  -> PC 5059/api/face/latest
  -> face_turn_web_bridge.py
  -> Pi /api/autonomous/face-turn (HEARTBEAT 或 STOP)
```

Pi 的 `start_robot.sh` 只负责 5000、相机和正式控制服务，不会启动电脑端
DeskMate 模型。本实验和桥接器必须分别在电脑上运行。

## 首次准备

从 Robitics 根目录初始化第三方子模块：

```powershell
git submodule update --init --recursive
py -3 -m pip install -e .\subrepos\DeskMate-Advance
```

## 无电机探测

先启动 Pi 的 5000 服务，然后在电脑执行有限帧探测：

```powershell
cd C:\Users\32126\Desktop\Robitics
py -3 .\pi_service\experiments\deskmate_face_position_bridge\deskmate_face_position_server.py --probe-frames 10
```

它只打印最后一帧 JSON 后退出，不向 Pi 发送控制请求。

## 持续发布并接入现有 J/L

电脑终端一：

```powershell
cd C:\Users\32126\Desktop\Robitics
py -3 .\pi_service\experiments\deskmate_face_position_bridge\deskmate_face_position_server.py --port 5059
```

浏览器检查：

```text
http://127.0.0.1:5059/api/face/latest
```

电脑终端二：

```powershell
cd C:\Users\32126\Desktop\Robitics\pi_service\experiments\face_tracking_validation
py -3 face_turn_web_bridge.py --face-url http://127.0.0.1:5059/api/face/latest --pi-url http://100.80.46.54:5000
```

随后才可由操作者在 5000 网页按 M，再按 J/L。桥接器不会开始转向；它只对
已经由 J/L 启动的 `FACE_CENTER_TURN` 续租，并在人脸居中后发送 STOP。

## 验证

```powershell
py -3 -m unittest discover -s .\pi_service\experiments\deskmate_face_position_bridge -p "test_*.py" -v
git diff --check -- .gitmodules .\pi_service\experiments\deskmate_face_position_bridge
```

单元测试和有限帧探测只证明代码、模型资源与视频读取链路可运行，不证明真车已经
按人脸完成原地转向。
