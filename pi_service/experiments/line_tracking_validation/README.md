# 直角循迹验证

## 树莓派自主运行（推荐的实车方式）

将整个项目复制到树莓派，至少要包含 `pi_service/` 以及
`third_party/DeskMate-Advance/src/track_line/`。在树莓派项目根目录执行：

```bash
chmod +x pi_service/run_autonomous_rectangle_line_tracking.sh
./pi_service/run_autonomous_rectangle_line_tracking.sh
```

该命令会在**同一台树莓派**完成：相机采集、OpenCV 识别、转弯状态机、P 微调、
向 Arduino 发送电机命令。视觉程序读取的是 `127.0.0.1` 本机视频流，而不是电脑的
网络画面；电脑可断开，仅不能再看到调试预览。终端中按 `Ctrl+C` 会先发送 `STOP`，
随后关闭本次启动的 `robot_web` 服务，释放 5000 端口。

脚本会先等待网页服务和 Arduino 串口都显示在线，再开始循迹，避免网页端口虽已
打开、Arduino 尚在 USB 重启握手阶段时过早退出。

默认串口为 `/dev/ttyACM0`。若树莓派上的 Arduino 是其他串口，例如
`/dev/ttyUSB0`，这样启动：

```bash
ROBOT_SERIAL_PORT=/dev/ttyUSB0 ./pi_service/run_autonomous_rectangle_line_tracking.sh
```

首次测试请架空车轮或低速扶持。脚本启动前，必须先停止旧的 `start_robot.sh` 服务；
否则两个程序会争用相机、串口或 5000 端口。

## Windows 画面监视与离线验证

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

- 启动脉冲：从静止起步时 `SF` 为 155、持续约 0.12 秒；
- 低速巡航：随后 `F` 为 65；
- 循迹微调：外侧轮最大为 180、内侧轮最小为 60；
- 右直角：`PR` 原地右转，PWM 为 155；
- 任意 `STOP`、视觉异常、HTTP 通信异常或关闭窗口：立即发送 `STOP`。

### P 微调

普通循迹不再只选择固定的 `FL/FR` 档位。它使用地面坐标中的标准化偏差 `offset` 计算 `修正量 = 150 × |offset|`：线偏右时左轮加速、右轮减速；线偏左时相反。两轮 PWM 被限制在内侧 60、外侧 180 之间，线接近画面中线（`|offset| <= 0.05`）时两轮同为巡航 PWM 65；每次 PWM 最多变化 12。直角转向仍使用固定的 `PR=155`，不会受 P 微调干扰。

启动功率和巡航功率分开：每次从停止重新进入前进时，先以 155 PWM 运行约 0.12 秒以克服静摩擦，再自动降为 65 PWM。这让车可以起步，同时给视觉控制留下更慢、更可预测的巡航速度。

### 粗略地面坐标

蓝色赛道梯形会被映射为俯视矩形，得到“左 -1、车身中线 0、右 +1”的粗略地面坐标；它不要求棋盘格标定，也不是厘米级测量。控制并不跟随最近的胶带，而是拟合地面线并取 `lookahead_y=0.55` 的前视点作为目标；这个参数越小，看得越远、越早预判转弯，越大则更贴近车头。地面偏差会先经过 5 帧中位数滤波、低通滤波和单帧变化限幅，再交给 P 控制和转弯状态机。预览窗口的 `GROUND OFFSET` 是实际用于控制的前视偏差，便于观察是否仍有跳变。

启动自动版本前，关闭网页遥控的按键操作；网页的 `/api/keys` 心跳会覆盖自动程序
的动作。首次实车测试必须把车架空或低速有人扶持，按 `Q` / `Esc` 即停止。

命令行调试示例（默认和一键启动均以 30 FPS 分析）：

```powershell
py -3 .\pi_service\experiments\line_tracking_validation\live_rectangle_route_monitor.py `
  --source http://100.80.46.54:5000/video_feed `
  --config .\third_party\DeskMate-Advance\src\track_line\config.dark_line.json `
  --process-fps 30
```

## 上车前的建议参数

- 先将直行速度调低到正常速度的 30%–40%。
- 相机朝下，确保画面下 1/3 能在转弯前看到横向/拐角区域。
- `missing_before_search=2`：避免单帧模糊就开始原地转。
- `max_search_frames=30`：约等于 1 秒（30 FPS）；超时必须 `STOP`。
- 对左转或右转分别摆放赛道试验，再记录实际需要的 `PL/PR` 时间；不要一开始就允许无限原地转。

下一阶段才将图像二值化得到的 `line_centre_x` 接给 `RightAngleTracker.step()`，并由一个明确的“开启循迹”开关接管 `RobotController` 动作。
## 直角右转的失线策略

这是一个**无岔路、顺时针矩形**的固定路线。到达角落时，摄像头会比车身更早看不见旧边的黑线；因此先以 `F` 继续前进约 0.05 秒。处理器以 30 FPS 工作时会自动换算为 2 个处理帧；若期间重新看到线，就取消转弯；若仍失线，即使右侧横线因视野、反光或阈值原因没有被 `RIGHT BRANCH` 检测到，规划器才会进入 `TURN_RIGHT`，电机执行 `PR`（原地右转，PWM 155）。它会持续转动，直到连续三帧确认看见新的边线后才恢复前进。

转向期间若检测到一条可信的新线（即使它还是斜的），程序会先停止原地转向，进入约 0.3 秒的 `RECOVERING_RIGHT`：让车向前并用前视 P 控制贴近新线；这段距离内线仍稳定可见，才恢复普通循迹。这样可避免画面中已经出现新线时仍持续原地转、导致车身落在新线右侧。

仍保留约 10 秒的安全上限（30 FPS 时自动换算为 `max_turn_frames=300`）；超过上限才会停止。若以后改成有岔路的路线，可将 `fixed_right_turn_on_line_end=False`，恢复“未确认角落就停止”的保守策略。
