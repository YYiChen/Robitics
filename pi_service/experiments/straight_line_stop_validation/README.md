# 直行循线到终点停车（独立验证）

这个目录不使用直角转弯、路线规划、丢线搜线或原地旋转代码。唯一规则是：

```text
检测到黑线 -> 直行；线偏左/右 -> 左右轮微调；连续 5 帧无有效线 -> STOP
```

默认在 30 FPS 下，连续 5 帧约为 0.17 秒。停车状态会锁定，重新出现线也不会自行
起步，必须重新启动实验。

微调量不是将电机总 PWM 降到 20，而是左右轮相对基础直行 PWM 的最小差速：

```text
直行 65/65
轻微右侧偏差 45/85
轻微左侧偏差 85/45
```

其中每次非零纠正至少 20 PWM，最大 60 PWM。

## 树莓派运行

先停止其他占用相机、串口或 5000 端口的服务，再运行：

```bash
chmod +x /home/g11/Desktop/pi_service/experiments/straight_line_stop_validation/run_pi_straight_line_stop.sh
/home/g11/Desktop/pi_service/experiments/straight_line_stop_validation/run_pi_straight_line_stop.sh
```

`Ctrl+C` 会发送 `STOP` 并关闭本次启动的网页服务。这个实验仍需要
`third_party/DeskMate-Advance/src/track_line/`，供 OpenCV 检测黑线。
