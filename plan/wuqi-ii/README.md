# wuqi-ii：J/L 人脸居中转向交接说明

## 目标

这份目录只保存 `wuqi-ii` 负责的左转、右转人脸居中功能相关文件快照和交接说明，不包含完整项目，也不作为树莓派的直接运行目录。

- `J`：向左开始搜索人脸，发现人脸后逐步修正到画面中心并停车。
- `L`：向右开始搜索人脸，发现人脸后逐步修正到画面中心并停车。
- DroidCam 和人脸检测运行在 Windows 电脑。
- 转向状态机和电机 PWM 控制运行在树莓派。
- 源代码快照来自提交 `faaa928f81bb277867c901d43ba0d18448d95f6d`。

## 文件清单

所有快照都位于 `left_right_face_turn/`，并保留正式项目中的相对路径。

| 文件 | 作用 |
| --- | --- |
| `pi_service/robot_web/templates/index.html` | 显示 J/L 左右转按钮和快捷键说明，并通过查询参数刷新前端脚本缓存。 |
| `pi_service/robot_web/static/app.js` | 监听 Edge 中的 `J`、`L` 键以及页面按钮，向树莓派发送开始转向请求。 |
| `pi_service/robot_web/app.py` | 提供 `/api/autonomous/face-turn` 接口，接收 `START`、`OBSERVE`、`CANCEL`。 |
| `pi_service/robot_web/end_line_turn_adaptor.py` | 在树莓派上执行“短脉冲转动 → 停车等待画面稳定 → 检查人脸偏差”的闭环状态机。 |
| `pi_service/experiments/droidcam_face_turn_validation/droidcam_face_turn.py` | 在 Windows 读取 DroidCam、检测人脸并持续上报人脸中心位置。 |
| `pi_service/experiments/droidcam_face_turn_validation/README.md` | DroidCam 工具的运行说明。 |
| `pi_service/experiments/droidcam_face_turn_validation/test_droidcam_face_turn.py` | DroidCam 人脸转向工具的单元测试。 |
| `pi_service/experiments/droidcam_face_turn_validation/__init__.py` | 实验工具的 Python 包标记。 |

## 数据流

1. Windows 先运行 `droidcam_face_turn.py`，默认读取 `DroidCam Video` 对应的摄像头编号 `1`。
2. 用户在 Edge 网页中按 `J` 或 `L`。
3. `app.js` 向树莓派发送：

   ```json
   {"action": "START", "direction": "LEFT"}
   ```

   或：

   ```json
   {"action": "START", "direction": "RIGHT"}
   ```

4. Windows 的 DroidCam 进程以默认 8 FPS 检测人脸，并向同一接口发送 `OBSERVE`，包含：

   - `found`：是否检测到人脸；
   - `frame_width`：DroidCam 画面宽度；
   - `center_x`：人脸框中心的横坐标。

5. 树莓派将人脸中心换算为归一化偏差：

   ```text
   offset = (center_x - frame_width / 2) / (frame_width / 2)
   ```

6. 人脸不在中心时，树莓派按偏差方向执行一次短脉冲转动；脉冲结束后立即停车，等待画面稳定，再处理新的人脸帧。
7. 人脸连续满足居中条件后，状态机关闭转向权限并停车。

## 当前调参

参数定义在 `end_line_turn_adaptor.py`：

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `FACE_TURN_PWM` | `90` | 原地转向短脉冲 PWM。 |
| `FACE_PULSE_SECONDS` | `0.10 s` | 单次转动持续时间。 |
| `FACE_SETTLE_SECONDS` | `0.32 s` | 每次转动后的画面稳定等待时间。 |
| `FACE_CENTER_TOLERANCE` | `0.10` | 允许的人脸水平归一化偏差。 |
| `FACE_CENTER_CONFIRM_FRAMES` | `4` | 连续居中确认帧数。 |
| `FACE_MAX_PULSES` | `40` | 最大转动脉冲次数。 |
| `FACE_TURN_TIMEOUT_SECONDS` | `25 s` | 单次任务总超时时间。 |
| `FACE_OBSERVATION_TIMEOUT_SECONDS` | `1 s` | DroidCam 观测中断判定时间。 |

## 安全行为

- 收不到第一帧 DroidCam 观测时保持停车。
- DroidCam 观测超过 1 秒未更新时，以 `FACE_STREAM_LOST` 结束并停车。
- 超过 25 秒仍未居中时，以 `FACE_TURN_TIMEOUT` 结束并停车。
- 达到 40 次脉冲仍未居中时，以 `FACE_PULSE_LIMIT` 结束并停车。
- 达到连续 4 帧居中后，以 `FACE_CENTERED` 结束并停车。
- `CANCEL`、空格键或 DroidCam 工具退出时会请求取消并停车。

## 运行顺序

树莓派使用正式项目路径启动网页控制服务：

```bash
cd /home/g11/Desktop/pi_service
bash run_end_line_turn_console.sh
```

Windows 在正式仓库根目录启动 DroidCam 观测进程：

```powershell
python pi_service/experiments/droidcam_face_turn_validation/droidcam_face_turn.py
```

随后在 Edge 打开：

```text
http://100.80.46.54:5000
```

首次更新后使用 `Ctrl+F5` 强制刷新，再用 `J`、`L` 测试。

只验证人脸检测而不发送电机控制请求时：

```powershell
python pi_service/experiments/droidcam_face_turn_validation/droidcam_face_turn.py --preview-only
```

## 维护说明

`plan/wuqi-ii/left_right_face_turn/` 是交接快照。后续修复应先修改正式的 `pi_service/` 文件并完成测试，再更新本目录中的对应快照，避免正式代码和交接文件出现差异。
