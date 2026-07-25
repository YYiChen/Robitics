# 恒定 PWM 90 行驶测试（独立）

这是用于排查底盘驱动的独立实验，不读取摄像头、不运行循迹算法，也不发送任何发牌器命令。它只持续向行驶电机接口发送右 `90`、左 `90`；按 `Ctrl+C` 会向 `/api/stop` 发送停车命令。

在树莓派运行：

```bash
chmod +x /home/g11/Desktop/pi_service/experiments/constant_drive_90_validation/run_pi_constant_drive_90.sh
/home/g11/Desktop/pi_service/experiments/constant_drive_90_validation/run_pi_constant_drive_90.sh
```

如果小车在这个测试中仍不走或两侧转速明显不同，问题不在循迹或视觉算法，而在行驶电机、驱动、电池供电、接线或底盘摩擦。
