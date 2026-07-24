Loss Hunters 树莓派小车网页控制包
=================================

目标
----
只启动一个 Python 程序，然后在同一个网页完成：
- CSI 摄像头视频流
- W/A/S/D、方向键和 Q/E/Z/C 控制
- 松开按键自动停车
- 空格/网页按钮紧急停止
- PWM、速度模式、目标速度和 PID 参数调整
- Arduino/摄像头在线状态
- IMU 与轮速数据显示
- 拍摄当前画面

默认网页
--------
http://100.80.46.54:5000/

首次部署
--------
1. 把整个文件夹复制到树莓派，例如：
   /home/g11/robot_web/

2. 进入文件夹并赋予执行权限：
   cd /home/g11/robot_web
   chmod +x *.sh

3. 缺少依赖时只执行一次：
   ./install_dependencies.sh

一键启动
--------
./start_robot.sh

之后直接打开：
http://100.80.46.54:5000/

停止
----
./stop_robot.sh

开机自动启动（可选，只安装一次）
--------------------------------
./install_autostart.sh

之后树莓派开机即可直接访问网页，不需要打开 VS Code。

常见调整
--------
Arduino 不是 /dev/ttyACM0 时：
ROBOT_SERIAL_PORT=/dev/ttyACM1 ./start_robot.sh

也可以让程序自动搜索：
ROBOT_SERIAL_PORT=auto ./start_robot.sh

更换网页端口：
ROBOT_WEB_PORT=5001 ./start_robot.sh

日志位置：
logs/robot.log

重要安全机制
------------
网页每 180 ms 发送一次心跳；超过 0.8 秒没有心跳，树莓派自动清空运动指令。
网页失焦、切换标签页、松开按钮或网络断开时会停车。
程序正常退出时会重复发送停车命令。
