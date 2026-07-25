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

该目录尚未接入 5000、HTTP 或 Arduino；因此不会抢占 M1/M2 控制权，也不会影响现有 I 型 180° 掉头实验。
