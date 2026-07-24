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

## 上车前的建议参数

- 先将直行速度调低到正常速度的 30%–40%。
- 相机朝下，确保画面下 1/3 能在转弯前看到横向/拐角区域。
- `missing_before_search=2`：避免单帧模糊就开始原地转。
- `max_search_frames=30`：约等于 1 秒（30 FPS）；超时必须 `STOP`。
- 对左转或右转分别摆放赛道试验，再记录实际需要的 `PL/PR` 时间；不要一开始就允许无限原地转。

下一阶段才将图像二值化得到的 `line_centre_x` 接给 `RightAngleTracker.step()`，并由一个明确的“开启循迹”开关接管 `RobotController` 动作。
