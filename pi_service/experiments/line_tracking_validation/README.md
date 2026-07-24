# 直角循迹验证（独立，不接硬件）

普通循迹只根据黑线相对画面中心的偏差输出 `F`、`FL`、`FR`。在 90° 拐角处，黑线会暂时离开画面的下方取样区，所以必须增加状态机：

```text
FOLLOW --连续两帧找不到线--> SEARCH_LEFT / SEARCH_RIGHT
SEARCH --重新找到线--> RECOVER（短暂停车确认） --> FOLLOW
SEARCH --持续超过上限--> LOST（STOP）
```

这里的 `PL` / `PR` 是现有 `robot_web/controller.py` 已支持的原地左/右转动作，故软件协议本身可以支持直角转弯；能否稳定通过仍取决于相机视野、车速、转向速度和赛道对比度。

## 运行离线验证

在 Windows 项目根目录运行：

```powershell
py -3 -m unittest discover -s .\pi_service\experiments\line_tracking_validation -p 'test_*.py' -v
```

测试覆盖左直角、右直角以及丢线后安全停止。它不读取摄像头、不控制电机，也不修改现有网页控制程序。

## 实时画面状态监视（不控制电机）

双击 `start_rectangle_line_monitor.cmd`，会打开提供的 MJPEG 画面，并在画面
上叠加 `STRAIGHT`、`TURN_RIGHT` 或 `STOP`。按 `Q` 或 `Esc` 退出。

启动文件默认读取：

```text
http://100.80.46.54:5000/video_feed
```

它默认使用开发阶段的白底黑线配置，并以 **10 FPS** 抽帧分析；未处理的帧会
直接丢弃，既不保存视频，也不会发送串口或电机命令。将来换成绿地白线时，将
启动参数中的配置文件替换为：

```text
..\\..\\..\\third_party\\DeskMate-Advance\\src\\track_line\\config.white_on_green.json
```

为避免桌腿、椅轮等黑色物体被误认为胶带，默认只在车前方的梯形赛道区域内
检测；预览中的浅蓝色梯形就是该范围。可通过 `--track-roi-top-width` 和
`--track-roi-bottom-width` 调整宽度。车不在路线附近时，显示 `STOP` 是预期的
安全行为。

监视器还会检查近、中、远三个绿点是否属于同一块连通的黑胶带。若最远点属于
另一块噪点，会从标线中删掉并显示 `far_candidate_disconnected`；此时程序不再
用该错误点计算转向趋势。

当近线所属的连通胶带在拐点高度向右伸出足够长的水平臂时，预览会显示
`RIGHT BRANCH: DETECTED`。该信号连续两帧后进入 `RIGHT_CORNER_ARMED`；待旧竖线
消失，才显示 `TURN_RIGHT`。已预判后允许最多三帧暂时看不到横线，避免相机轻微
抖动或胶带反光立刻取消转弯。

进入 `TURN_RIGHT` 后，程序持续发送原地右转；即使转动中又短暂看到旧角附近的
胶带，也不会提前结束。只有重新看见至少两段稳定、无右侧分支且接近直行方向的
新边线，连续三帧后才回到直行。为避免机械故障导致无限转向，默认 10 秒仍未找到
新线会安全停止。

## 自动电机执行（首次必须有人看守）

`start_rectangle_auto_drive.cmd` 才会向 Pi 控制器发送行驶命令；原来的
`start_rectangle_line_monitor.cmd` 始终只显示画面。自动版本启动时先检查 Arduino
在线，并保存以下 PWM 配置到 Pi 的 `drive_config.json`：

- 直行：`F` 为 120；循迹微调时外侧 120、内侧 80；
- 右直角：`PR` 原地右转，PWM 为 150；
- 任意 `STOP`、视觉异常、HTTP 通信异常或关闭窗口：立即发送 `STOP`。

启动自动版本前，关闭网页遥控的按键操作；网页的 `/api/keys` 心跳会覆盖自动程序
的动作。首次实车测试必须把车架空或低速有人扶持，按 `Q` / `Esc` 即停止。

命令行调试示例（需要提高处理频率时改为 `--process-fps 15`）：

```powershell
py -3 .\pi_service\experiments\line_tracking_validation\live_rectangle_route_monitor.py `
  --source http://100.80.46.54:5000/video_feed `
  --config .\third_party\DeskMate-Advance\src\track_line\config.dark_line.json `
  --process-fps 10
```

## 上车前的建议参数

- 先将直行速度调低到正常速度的 30%–40%。
- 相机朝下，确保画面下 1/3 能在转弯前看到横向/拐角区域。
- `missing_before_search=2`：避免单帧模糊就开始原地转。
- `max_search_frames=30`：约等于 1 秒（30 FPS）；超时必须 `STOP`。
- 对左转或右转分别摆放赛道试验，再记录实际需要的 `PL/PR` 时间；不要一开始就允许无限原地转。

下一阶段才将图像二值化得到的 `line_centre_x` 接给 `RightAngleTracker.step()`，并由一个明确的“开启循迹”开关接管 `RobotController` 动作。
