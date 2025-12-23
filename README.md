# Robotic Arm
This document presents a personal engineering project involving the design and construction of a six-degree-of-freedom (6-DOF) robotic manipulator developed entirely from scratch. The robotic system is actuated by six MG996R pulse-width modulation (PWM) servomotors, which are driven by a PCA9685 16-channel, 12-bit PWM servo controller interfaced via the I2C protocol. Low-level actuator control and real-time signal generation are handled by an Arduino Nano microcontroller.

High-level system control is implemented on a Raspberry Pi running the Robot Operating System (ROS), which is responsible for computationally intensive tasks such as inverse kinematics, trajectory planning, and command generation. Control commands are transmitted from the Raspberry Pi to the Arduino Nano over a serial communication interface, enabling coordinated motion control across all joints.

## Physical Build


https://github.com/user-attachments/assets/ec7e50fa-2d13-46d9-afe8-c7680861c63d

## CAD Model with Solidworks

![Demo](docs/img/side_profile.png)

## Visualization and motion planning with rviz2

https://github.com/user-attachments/assets/5d312841-a5a9-4a7e-a08d-b234987b8f20


https://github.com/user-attachments/assets/f96ab1f5-55c6-4020-97d4-8c81f729513d


### Steps to run

1. Create a ROS package
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 robotic_arm
```

2. Add the contents of the robotic_arm folder into the package directory `~/ros2_ws/src/robotic_arm`. Replace `./robotic_arm` to the location of this cloned repo.
```
cp -rf ./robotic_arm ~/ros2_ws/src/robotic_arm
```

3. Build the package (Full Rebuild)
```
cd ~/ros2_ws
rm -rf build install log
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Alternatively, a quick rebuild alternative for minor changes to code
```
cd ~/ros2_ws
colcon build --packages-select robotic_arm
source install/setup.bash
```

4. Run the visualization with
```
ros2 launch robotic_arm robotic_arm.launch.py
```
