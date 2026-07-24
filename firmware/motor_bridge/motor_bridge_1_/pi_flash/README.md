# Raspberry Pi Arduino Mega upload package

Upload this entire `motor_bridge_1_` folder to the Raspberry Pi. Do not upload
only the `.ino`: the sketch also needs `motor_control.h` and
`ultrasonic_avoidance.h` in the same folder.

This firmware is for an **Arduino Mega 2560** using the Adafruit Motor Shield
V1-compatible `AFMotor` library. It supports the web controls for M3/M4 power,
direction, and runtime. A signed motor PWM is used internally: positive is
forward, negative is reverse.

## First-time Pi setup

From inside the uploaded `motor_bridge_1_` folder:

```bash
chmod +x pi_flash/*.sh
./pi_flash/setup_arduino_cli.sh
```

If the setup script tells you to add `dialout` permission, run its printed
`sudo usermod` command, then log out/in or reboot the Pi before uploading.

## Upload the firmware

1. Connect the Arduino to the Raspberry Pi USB port.
2. Stop the web service that is using the Arduino serial port (`Ctrl+C` in the
   terminal running `start_robot.sh`).
3. Confirm the port, normally `/dev/ttyACM0`:

   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
   ```

4. Upload:

   ```bash
   ./pi_flash/flash_mega.sh /dev/ttyACM0
   ```

5. Start the web service again:

   ```bash
   cd ~/Desktop/pi_service
   ./start_robot.sh
   ```

If the Arduino appears as a different port, replace `/dev/ttyACM0` in the
upload command. The upload script compiles before it writes anything to the
board, and stops if the serial port is occupied.
