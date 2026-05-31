# RUDRA ROS 2 micro-ROS Firmware

This sketch is the ROS 2 replacement for the older ROS 1 `rosserial` base
controller. It is intended for Teensy-class boards using `micro_ros_arduino`.

## ROS 2 Topics

Subscribes:

- `/cmd_vel` (`geometry_msgs/msg/Twist`): autonomous/navigation velocity command.
- `/manual_mode` (`std_msgs/msg/Bool`): enables direct manual control when true.
- `/manual_cmd_vel` (`geometry_msgs/msg/Twist`): joystick/manual placeholder.

Publishes:

- `/odom` (`nav_msgs/msg/Odometry`)
- `/imu/data_raw` (`sensor_msgs/msg/Imu`)

## Manual Mode Placeholder

The joystick does not need to live on the Teensy. If it stays on an Arduino Uno,
bridge the Uno into ROS 2 on the NUC and publish:

- `/manual_mode`: `true` to let manual commands drive the base.
- `/manual_cmd_vel.linear.x`: throttle normalized from `-1.0` to `1.0`.
- `/manual_cmd_vel.angular.z`: steering normalized from `-1.0` to `1.0`.

That keeps the Teensy firmware independent of the joystick hardware.

## Agent

Run the micro-ROS Agent on the NUC:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Update the device path if the Teensy appears somewhere else.

## Arduino Setup

Install the Arduino libraries:

```bash
arduino-cli lib install micro_ros_arduino@3.0.0-iron
arduino-cli lib install Encoder
cp -a IMU-Teensy/I2Cdev IMU-Teensy/MPU6050 Sabertooth ~/Arduino/libraries/
```

Patch Teensy's `platform.txt` so it links the precompiled micro-ROS library:

```bash
TEENSY_PLATFORM=~/.arduino15/packages/teensy/hardware/avr/1.61.0
cp "$TEENSY_PLATFORM/platform.txt" "$TEENSY_PLATFORM/platform.txt.before_micro_ros"
cp ~/Arduino/libraries/micro_ros_arduino/extras/patching_boards/platform_teensy.txt \
  "$TEENSY_PLATFORM/platform.txt"
```

Compile for Teensy 3.6:

```bash
arduino-cli compile \
  --fqbn teensy:avr:teensy36 \
  v2_base_controller_ros2_micro_ros
```

Note: the current shell is using ROS 2 `lyrical`, while Arduino Library Manager
currently provides `micro_ros_arduino` through `3.0.0-iron`. This is enough to
compile the firmware. If the micro-ROS Agent has runtime compatibility issues,
build a distro-matched micro-ROS Arduino static library.

## TF

The firmware publishes `/odom`, but does not publish `/tf`. The ROS 2 package
`rudra_core` includes `odom_tf_broadcaster`, which republishes the odometry pose
as the `odom -> base_link` transform.
