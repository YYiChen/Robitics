# Pi Service 通用电机调用

`pi_service/robot_web` 是唯一能直接打开 Arduino 串口的服务。其他程序禁止各自
创建 `RobotController` 或直接操作串口；统一调用 `pi_service.robot_client`。

```python
from pi_service.robot_client import RobotClientConfig, RobotWebClient

robot = RobotWebClient(RobotClientConfig("http://127.0.0.1:5000"))
robot.require_arduino_online()
robot.send_action("F")   # F / FL / FR / PL / PR / STOP 等
# 程序结束或异常时必须执行：
robot.stop()
```

远程电脑控制 Pi 时，将地址替换为 Pi 的地址，例如
`http://100.80.46.54:5000`。

## 配置行驶 PWM

通过同一个客户端调用 `configure_drive()`，配置会交给 robot-web 校验并保存到
Pi 的 `drive_config.json`。不要绕开 API 直接改运行中的控制状态。

```python
robot.configure_drive({
    "speed_mode": False,
    "straight_pwm": 120,
    "pivot_pwm": 150,
})
```

浏览器遥控与自动程序会共享同一个动作状态；自动运行时不要按网页 WASD/方向键，
否则网页的心跳会覆盖自动程序的动作。控制循环异常、视觉丢失或任务结束时应发送
`STOP`。
