# Arduino 串口协议

所有命令均以换行结束，波特率为 9600。

| 命令 | 说明 |
| --- | --- |
| `M,m1,m2,m3,m4` | 四个电机的有符号原始 PWM（-255..255） |
| `V,leftPPS,rightPPS` | 后轮 PID 的目标脉冲每秒 |
| `KP,value` / `KI,value` / `KD,value` | 后轮 PID 参数 |
| `STOP` | 立即停车并退出速度模式 |
| `IMU` | 返回 `IMU,roll,pitch,yaw`；仅遥测 |
| `SPD` | 返回 `SPD,curL,curR,tgtL,tgtR,pidL,pidR` |
| `US` | 返回 `US,-1,front,-1`；当前只接中间前向超声波（26/27） |

Arduino 保留最终安全权：1 秒未收到有效命令必须停车。前方距离小于等于 15 cm 时只拒绝前进；原地左转、原地右转和后退不受超声波限制。`-1` 代表无有效超声波回波，不代表障碍物。
