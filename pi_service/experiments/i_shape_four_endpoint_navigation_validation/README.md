# 工字形四端点：原地转向调度验证

这是一个纯 Python、不开相机和电机的状态机实验。它实现的赛道规则是：

- 小车只沿中间竖线行驶；
- 到上/下交叉点后，原地转 90° 面朝端点；
- **不沿横杆前进或倒车**；
- 面朝端点后由未来的执行器调用 M3/M4 发牌；
- 去同一交叉点另一端时，经由朝向竖线的两个连续 90° 转向，不使用 180° 定时转向；
- 去另一交叉点时，先转回竖线，再沿竖线行驶。

视觉适配层需要提供：

- `junction_detected`：骨架 junction / 横杆确认；
- `forward_line_detected`：转动后，预期方向的白线在画面底部中央连续出现；
- `deal_complete`：M3/M4 动作完成回报。

运行离线测试：

```bash
cd /home/g11/Desktop/pi_service/experiments/i_shape_four_endpoint_navigation_validation
python3 -m unittest -v test_four_endpoint_planner
```

## 5000 接入验证

新的可选路线模式只验证第一个目标 `[1]`：沿竖线到上交叉点，原地左转 90°，重捕横杆方向白线后停车。它只在按 `M` 后控制 M1/M2，**不会调用 M3/M4 发牌**，也不会沿横杆行驶。

```bash
cd /home/g11/Desktop/pi_service
bash ./run_four_endpoint_pivot_validation_console.sh
```

打开 `http://127.0.0.1:5000`，画面状态应依次显示：

`FOLLOW_STEM → STOP_AT_JUNCTION → PIVOT_TO_HEADING → DEAL_CARD`

现有 I 型 180° 掉头入口不变；该模式是单独的 `route-mode`，同一时刻只能运行其中一个。
