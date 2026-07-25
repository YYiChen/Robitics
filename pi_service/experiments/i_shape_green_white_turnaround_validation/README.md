# 绿地白线 I 型掉头验证

本目录只替换 I 型扫描线算法的图像分割：用 HSV 检出绿地中的白胶带。
主线追踪、横线预判、掉头状态机与 PWM 控制复用
`i_shape_scanline_turnaround_validation`，不修改原白底黑线实验。

5000 启动入口：

```bash
cd /home/g11/Desktop/pi_service
./run_green_white_scanline_i_console.sh
```

视觉默认暂停；网页按 `M` 才允许 M1/M2 行驶。
