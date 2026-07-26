# DroidCam 独立人脸居中转向

这个工具不参与白线循迹，也不占用网页现有快捷键。它只在独立的 DroidCam
窗口获得焦点时处理：

- `J` 或左侧画面按钮：向左短脉冲搜脸，检测到人脸后修正到画面中心并停车；
- `L` 或右侧画面按钮：向右短脉冲搜脸，检测到人脸后修正到画面中心并停车；
- `Space`：取消当前人脸转向并停车；
- `Esc`：停车并退出。

本机的设备列表中 `DroidCam Video` 与 `USB2.0 HD UVC WebCam` 同时存在。OpenCV
默认使用 `--face-source 1` 连接 DroidCam，避免再次接到电脑自带 USB 摄像头。

先让树莓派更新并启动 `end_line_turn_adaptor`，然后在 Windows 仓库根目录运行：

```powershell
python pi_service/experiments/droidcam_face_turn_validation/droidcam_face_turn.py
```

只看识别、不控制电机：

```powershell
python pi_service/experiments/droidcam_face_turn_validation/droidcam_face_turn.py --preview-only
```

如果 DroidCam 的编号发生变化，可改用 `--face-source 0`、`2`，或手机 HTTP 地址
`--face-source http://手机IP:4747/video`。
