# 固定顺时针矩形（独立验证）

只适用于无岔路、每个角都是固定右转约 90° 的矩形路线。它不使用旧的通用角落规划。

```text
FOLLOW_LINE -> 连续 5 帧丢线 -> FORWARD_APPROACH (0.20 s)
-> PIVOT_RIGHT (0.30 s) -> FORWARD_REACQUIRE
-> 连续 3 帧看到新线 -> FOLLOW_LINE
```

完成第 4 次右转后锁定 `STOP`。转完后 0.80 秒仍未重新看到线，也会 `STOP`。

需实车标定的仅有两项：

- `--corner-forward-seconds 0.20`：相机先看不到线后，车身到真实角点的补偿；
- `--right-turn-seconds 0.30`：155 PWM 原地右转约 90° 的时间。

运行前停止其他视觉控车程序：

```bash
chmod +x /home/g11/Desktop/pi_service/experiments/fixed_rectangle_validation/run_pi_fixed_rectangle.sh
/home/g11/Desktop/pi_service/experiments/fixed_rectangle_validation/run_pi_fixed_rectangle.sh
```

电脑浏览器打开 `http://<树莓派 IP>:5053` 可实时查看固定状态机与电机命令。按
`Ctrl+C` 会发送 `STOP`。
