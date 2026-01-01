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

1. Install the ros2 package from this github repo: https://github.com/andrearante12/ros2_ws
```
cd ~/
git clone https://github.com/andrearante12/ros2_ws
```


2. Build the package (Full Rebuild)
```
cd ~/ros2_ws
rm -rf build install log
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Alternatively, a quick rebuild for select packages
```
cd ~/ros2_ws
colcon build --packages-select package_name
source install/setup.bash
```

3. Confirm that the distrubiton is working properly by launching the joint state manager with manual controls
```
ros2 launch robotic_arm robotic_arm.launch.py
```

## Starting the Inverse Kinematics Pipeline

1. Run the motion planner with MoveIt and rviz2
```
ros2 launch robotic_arm_v3_config demo_with_controllers.launch.py
```

2. Run pose_printer node to connect the device drivers to the simulation output
```
ros2 run pose_printer pose_printer
```

3. Send a plan (x, y, z) target coordinates with the move_program node
```
ros2 run move_program move_program 0.7 -1.2 1.145
```

5. Manual spawn robot model into gazebo for physics simulation (Optional, and buggy)
```
gz service -s /world/empty/create   --reqtype gz.msgs.EntityFactory   --reptype gz.msgs.Boolean   --req "sdf_filename: 'model://robotic_arm_model_v3', name: 'robotic_arm'"
```


