# X 路标计数预览（无电机）

这是独立的视觉测试：摄像头画面中会显示完整路线、紫色骨架、X/T 路标和计数；不会创建电机控制器，也不会发送发牌器命令。

运行：

```bash
chmod +x /home/g11/Desktop/pi_service/experiments/marker_count_preview_validation/run_pi_marker_count_preview.sh
/home/g11/Desktop/pi_service/experiments/marker_count_preview_validation/run_pi_marker_count_preview.sh
```

在浏览器打开 `http://100.80.46.54:5055`。仅完整的四臂 X 横杠会进入候选；同一个横杠需连续识别两帧才记一次，离开画面约 1.2 秒后才能再次计数。四个横杠为一圈：显示由 `1/4`、`2/4`、`3/4` 变为 `0/4 LAP=1`。

原始逐帧日志在：

```text
/home/g11/Desktop/pi_service/logs/marker_count_preview/latest.log
```

结束时会自动输出路标事件摘要；也可手动运行：

```bash
python3 /home/g11/Desktop/pi_service/experiments/marker_count_preview_validation/summarize_marker_log.py \
  /home/g11/Desktop/pi_service/logs/marker_count_preview/latest.log
```
