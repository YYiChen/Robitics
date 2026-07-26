# Haar 人脸灵敏度探索（隔离，不控制小车）

此实验只读取已经运行的 5000 相机流，在 5058 画出人脸框并记录 JSONL。OpenCV 官方
`haarcascade_frontalface_default.xml` 已随实验目录保存，因此不依赖树莓派的 `cv2.data`。
它不导入
`robot_web.controller`，不发送 HTTP 电机指令，也不响应 M 键。

## 树莓派运行

先确保主服务的相机预览可访问，再运行：

```bash
cd /home/g11/Desktop/pi_service/experiments/face_tracking_validation
python3 -m unittest test_face_detector -v
./run_pi_face_probe.sh
```

浏览器打开：`http://127.0.0.1:5058`。

## 怎么测“最远可检出距离”

1. 面向镜头站立，从近到远每次移动 0.5 米；不要让人脸偏向或被遮挡。
2. 每个距离保持约 5 秒，记录是否连续绿框、页面的“近 5 秒检出率”、框宽（px）和耗时。
3. `runtime_logs/face_probe.jsonl` 保存每帧 `detected`、框宽/面积、中心坐标和处理时间。
4. 将“连续检出率明显下降前的最远距离”作为探索结果；不要把框宽直接当作精确距离。

这不是路线、停车或转向功能。结束时按 `Ctrl+C` 即可；5000 主服务与电机状态不受影响。
