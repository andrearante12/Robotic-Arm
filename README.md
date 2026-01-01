# Robotic Arm

## System Overview
This document presents a personal engineering project involving the design and construction of a six-degree-of-freedom (6-DOF) robotic arm. The robotic system is controlled by six MG996R pulse-width modulation (PWM) servomotors, which are driven by a PCA9685 16-channel, 12-bit PWM servo controller interfaced via the I2C protocol. Low-level actuator control is handled by an Arduino Nano microcontroller.

High-level system control is implemented on a Raspberry Pi running the Robot Operating System (ROS), which is responsible for computationally intensive tasks such as inverse kinematics, trajectory planning, computer vision, and command generation. The ROS2 workspace can be download here: https://github.com/andrearante12/ros2_ws. Control commands are transmitted from the Raspberry Pi to the Arduino Nano over a serial communication interface, enabling coordinated motion control across all joints.

## System Requirements
- Developed on Ubuntu 24.04 (required for ROS2 Jazzy)
- ROS2 - Jazzy 
- Moveit2 - Harmonic

## Physical Build

Demo Video


https://github.com/user-attachments/assets/ec7e50fa-2d13-46d9-afe8-c7680861c63d

## Components
PCA9685 Servo Driver: https://www.amazon.com/dp/B0CNVBWX2M?ref=ppx_yo2ov_dt_b_fed_asin_title

Aluminum Body (w/out servos): https://www.amazon.com/dp/B01LW0LUPT?ref=ppx_yo2ov_dt_b_fed_asin_title

MG996R Digital Servo Motor (x6): https://www.amazon.com/dp/B09LS7RB5J?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1

External Power Source (5V 6A): https://www.amazon.com/dp/B01D8FM4N4?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1 

<img width="836" height="292" alt="Screenshot 2026-01-01 005100" src="https://github.com/user-attachments/assets/524ef937-452a-42c3-98e2-843da54fabd2" />
<img width="822" height="163" alt="Screenshot 2026-01-01 005316" src="https://github.com/user-attachments/assets/1aced96d-2121-4cd4-b6a2-b36aa6647610" />
<img width="812" height="187" alt="Screenshot 2026-01-01 005501" src="https://github.com/user-attachments/assets/a5a996ff-5c12-4416-a5e6-418a89ccaddd" />

## CAD Model with Solidworks

Note: the above mentioned aluminum body doesn't come with an official CAD model, so the below and attached Solidworks file are my own personal recreation.

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


