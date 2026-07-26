# DeskMate-Advance YuNet 人脸位置桥接

这是一个电脑端隔离实验。它调用根目录 Git submodule
`subrepos/DeskMate-Advance` 的正式摄像头与人脸模型接口：

- `poker_dealer.io.camera.OpenCVCamera` 独占读取 DroidCam MJPEG；
- `OpenCvFaceIdentityAdapter` 使用 YuNet 检测和 SFace 特征模型；
- 本实验只发布人脸框、中心偏移和置信度，不保存画面或人脸特征；
- 5059 同时提供带框 MJPEG 预览，JPEG 编码限频并在独立线程中运行；
- 同一进程内的桥接线程直接读取内存快照，不再通过5059回读；
- 桥接线程不启动转向，只用正式Robotics API续租或停止已有转向。

## 数据链路

```text
手机 DroidCam http://100.93.97.117:4747/video
  -> PC OpenCVCamera（直接 MJPEG）
  -> DeskMate YuNet/SFace
     ├─> PC 5059/api/face/latest + preview_feed（仅对外观察）
     └─> 内存 FaceTurnBridge
          -> Pi /api/robotics/v1/status
          -> Pi /api/robotics/v1/actions
             (face_turn_heartbeat 或 face_turn_stop，每次独立 request_id)
```

Pi 的 `start_robot.sh` 只负责 5000 和正式控制服务，不会启动电脑端
DeskMate模型。电脑端现在只需运行本文件一个进程，不再单独启动
`face_turn_web_bridge.py`。

5059 服务默认采用单实例启动。再次执行相同命令时，新进程只会终止命令行指向
本目录同一个 `deskmate_face_position_server.py`、且端口相同的旧进程；不会终止
其他 Python、5000 主服务或不同端口的实验。调试时可用
`--no-replace-existing` 禁止自动替换。

默认直接读取 `http://100.93.97.117:4747/video`，但只有本进程可以打开该原始地址。
不要在浏览器或DroidCam Windows客户端中同时打开原始流；看画面统一使用
`http://127.0.0.1:5059/preview_feed`。若上游被占用，5059会明确返回
`camera_source_busy:droidcam_single_client_in_use` 并每秒重试，释放上游后无需重启。
确需使用本地虚拟摄像头时可显式传 `--source 1 --backend msmf`，但必须先确认设备
索引和真实画面，不能把“设备成功打开”当作画面可用。

本实验不改 DeskMate submodule 的身份验证配置，而在入口层使用更适合远距离车控的
YuNet 参数：检测阈值 `0.75`、最小人脸边长 `40 px`。可分别通过
`--detector-score-threshold` 和 `--minimum-face-size-px` 覆盖。中心停车门禁仍由
桥接器负责，默认 `offset_x_normalized ±0.30`。

## 首次准备

从 Robitics 根目录初始化第三方子模块：

```powershell
git submodule update --init --recursive
py -3 -m pip install -e .\subrepos\DeskMate-Advance
```

两个 ONNX 模型由 Git LFS 管理。新机器如果没有完成 LFS smudge，模型文件会只是
约 130 字节的文本指针，仓库虽然能克隆但模型一定无法加载。安装 Git LFS 后执行：

```powershell
git -C .\subrepos\DeskMate-Advance lfs pull
```

然后用 `FaceIdentityConfig.verify_assets()` 或本目录单元测试核对模型 SHA-256。
当前固定模型哈希为：

- YuNet：`8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4`
- SFace：`0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79`

## 无电机探测

先启动 Pi 的 5000 服务，然后在电脑执行有限帧探测：

```powershell
cd C:\Users\32126\Desktop\Robitics
py -3 .\pi_service\experiments\deskmate_face_position_bridge\deskmate_face_position_server.py --probe-frames 10
```

它只打印最后一帧 JSON 后退出，不向 Pi 发送控制请求。

## 单进程持续运行并接入J/L/状态机

电脑终端一：

```powershell
cd C:\Users\32126\Desktop\Robitics
py -3 .\pi_service\experiments\deskmate_face_position_bridge\deskmate_face_position_server.py --source http://100.93.97.117:4747/video --port 5059 --pi-url http://100.80.46.54:5000
```

浏览器检查：

```text
http://127.0.0.1:5059/api/face/latest
http://127.0.0.1:5059/preview_feed
```

端口 5000 的正式控制台会从当前操作电脑的 `127.0.0.1:5059` 加载独立的
“电脑端人脸检测”卡片。绿色框是当前用于控制判断的主脸，黄色竖线是
`face_turn_web_bridge.py` 的中心门禁（默认 `offset_x_normalized ±0.30`），
蓝色竖线是画面中心。状态栏同时显示 YuNet 原始检测数、可用框数和主脸偏移。
如果用另一台电脑或手机打开 5000 页面，它的 `127.0.0.1` 不会指向运行模型的
电脑，因此该卡片会显示离线，但不会影响树莓派相机、路线识别或电机安全逻辑。

无需第二个Windows终端。启动时会精确结束同工作区残留的独立
`face_turn_web_bridge.py`，避免双心跳。随后可由操作者在5000网页按M，再按J/L；
状态机也可调用正式的 `face_turn_start`。内嵌桥接器不会开始转向；它只对
已经由 J/L 启动的 `FACE_CENTER_TURN` 续租，并在人脸居中后发送 STOP。

调试时可用 `--disable-pi-bridge` 只运行推理和5059观察，不向Pi发送任何动作。

## 验证

```powershell
py -3 -m unittest discover -s .\pi_service\experiments\deskmate_face_position_bridge -p "test_*.py" -v
git diff --check -- .gitmodules .\pi_service\experiments\deskmate_face_position_bridge
```

单元测试和有限帧探测只证明代码、模型资源与视频读取链路可运行，不证明真车已经
按人脸完成原地转向。
