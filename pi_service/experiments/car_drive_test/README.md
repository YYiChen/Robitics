# 小车整车联调测试

这是与正式 `robot_web` 分开的最小测试包。Arduino 只用 M1（右）和 M2（左）；网页提供 CSI 摄像头画面、前进/后退/原地转向和 PWM 调节。

## 1. Arduino

在 Arduino IDE 打开并烧录：

`firmware/experiments/car_drive_test_m3_m4/car_drive_test_m3_m4.ino`

若按“前进”有一侧反向，在草图顶部将对应的 `LEFT_FORWARD_SIGN` 或 `RIGHT_FORWARD_SIGN` 改为 `-1`。
默认 PWM 为 100；请先将车轮悬空确认方向，再逐步在网页中提高速度。

## 2. 树莓派

将整个 `pi_service/experiments/car_drive_test/` 目录复制到树莓派的 `~/Desktop/Robitics/pi_service/experiments/`。首次运行需先按正式服务安装依赖：

```bash
cd ~/Desktop/Robitics/pi_service
./install_dependencies.sh
chmod +x experiments/car_drive_test/start_car_test.sh
./experiments/car_drive_test/start_car_test.sh
```

浏览器打开 `http://树莓派IP:5050`。按住 W/A/S/D 或页面方向按钮才会运动，松开/失焦/网页断连，以及 Arduino 700 ms 未收到心跳都会停车。

测试服务默认会依次查找 `/dev/ttyACM*` 与 `/dev/ttyUSB*`。若要固定使用某个串口：

```bash
ROBOT_SERIAL_PORT=/dev/ttyUSB0 ./experiments/car_drive_test/start_car_test.sh
```
