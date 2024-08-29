Refer to https://goldenmotor.bike/wp-content/uploads/2024/04/EZkontrol-CANBUS-MCU-to-METER-V1.1-20230727-2.pdf and https://goldenmotor.bike/wp-content/uploads/2024/04/EZKontrol-Controllers-CAN-Protocols-Instruction-2.pdf

Use a USB-CAN analyzer that looks like the one in the following repo: https://github.com/kobolt/usb-can

https://www.seeedstudio.com/USB-CAN-Analyzer-p-2888.html?srsltid=AfmBOorjk0Vy--AGvpwsWtm63IqhQWV5z6zEskQg90uR6uetJyodnASR

You also need to change the name of your serial ports. You can refer to https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html#usb-serial-port-software-setup-on-linux

`92-canusb.rules`
SUBSYSTEM=="tty", ATTRS{idProduct}=="7523", ATTRS{idVendor}=="1a86", SYMLINK+="ttyCANUSB", GROUP="dialout",MODE="0666"

`99-pixhawk.rules`
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyPixhawk4"



You also need to source the environment (for ROS2 and the ROS2 package here. If you are lazy, you can add them to ~/.bashrc)

Usage
ros2 run dashboard dashboard
ros2 run dashboard lap_counter
(will add launchfile soon)