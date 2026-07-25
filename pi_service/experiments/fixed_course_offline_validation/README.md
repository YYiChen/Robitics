# 固定绿布路线：离线节点回放

该实验只读取视频或 JPG 帧目录，输出路线、横线、节点编号和模拟停车状态；不会连接树莓派、Arduino 或电机。

```powershell
python .\pi_service\experiments\fixed_course_offline_validation\offline_node_replay.py `
  --source .\data\captures\your-session `
  --output-video .\artifacts\offline-node-replay.mp4 `
  --output-jsonl .\artifacts\offline-node-replay.jsonl
```

默认每个确认到的节点在横线到达画面 72% 高度时进入模拟停车，保持 2 秒后继续。输出只代表离线回放，不代表实车已验证。

也可以直接运行：

```powershell
.\pi_service\experiments\fixed_course_offline_validation\run_offline_node_replay.ps1 `
  -Source .\data\captures\your-session
```
