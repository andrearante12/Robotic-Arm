# ImitationArm
This document presents Imitation Arm, a 6DOF robotic arm that learns skills by watching human demonstrations and mimicing movements. The end goal is to create a generalized system where a human can record a video of them picking up and placing an object and upload that to the arm. The arm will then decode the video and be able to copy the exact motion the human made completely autonomously.


## System Overview
A human can train the robot via a wearable device consisting of an ESP32 microcontroller and a MPU-6050 Acceleromter and Gyroscope sensor. Data is streamed over WiFi to the central RaspberryPi using MQTT messaging protocol. The RaspberryPI is responsible for callibration and smoothing the sensor data using various signal processing techniques.

The robotic system is controlled by six MG996R pulse-width modulation (PWM) servomotors, which are driven by a PCA9685 16-channel, 12-bit PWM servo controller interfaced via the I2C protocol. Low-level actuator control is handled by an Arduino Nano microcontroller.

High-level system control is implemented on a Raspberry Pi running the Robot Operating System (ROS), which is responsible for computationally intensive control tasks. The ROS2 workspace can be download here: https://github.com/andrearante12/ros2_ws. A custom inverse kinematic algorithm implemented via gradient descent and used to convert target coordinates (x, y, z) into the corresponding servo angles. MoveIt2 was then used to create a motion plan and verify that the path is valid via trajectory planning and collision detection. Control commands are transmitted from the Raspberry Pi to the Arduino Nano over a serial communication interface.

## System Requirements
- Developed on Ubuntu 24.04 (required for ROS2 Jazzy)
- ROS2 - Jazzy 
- Moveit2 - Harmonic

## Physical Build and Demonstration

Demo Video using breadboard based protoype of wearable controller.

https://github.com/user-attachments/assets/144eaac5-5938-4217-9122-2242566a99c5


Rough prototype of wearable controller


https://github.com/user-attachments/assets/a6dace8c-65ac-41af-8a9c-40d3fd4ca57d

![IMG_1939](https://github.com/user-attachments/assets/fcc12da2-aa8b-4a2a-a6ee-2820b6211cab)

![IMG_1940](https://github.com/user-attachments/assets/e8a81c16-5bbc-4d5e-98a2-1cae8d4265bf)

## Visualization and motion planning with rviz2

https://github.com/user-attachments/assets/5d312841-a5a9-4a7e-a08d-b234987b8f20


Succesful motion planning demonstration

https://github.com/user-attachments/assets/f96ab1f5-55c6-4020-97d4-8c81f729513d



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

## Starting the IMU Controller Pipeline

1. Run the motion planner with MoveIt and rviz2
```
ros2 launch robotic_arm_v3_config demo_with_controllers.launch.py
```

2. Run the IMU communication node
```
ros2 run mqtt_imu mqtt_imu_node
```

3. Run the IMU controller node

Run with the recommended callibration 
```
ros2 run esp32_controller esp32_controller --ros-args \
  -p callback_skip_rate:=5 \
  -p x_sensitivity:=3.0 \
  -p lock_y_axis:=true \
  -p default_y_position:=-1.2 \
  -p lock_wrist:=true \
  -p default_wrist_angle:=90 \
  -p default_z_position:=1.10

```

# ESP32 Controller - Quick Parameter Reference

| Parameter | Type | Default | Range/Options | Description |
|-----------|------|---------|---------------|-------------|
| `callback_skip_rate` | int | 5 | 1-100 | Commands sent every N callbacks (higher = slower) |
| `x_sensitivity` | double | 1.5 | 0.1-10.0 | X-axis responsiveness (higher = more sensitive) |
| `y_sensitivity` | double | 2.0 | 0.1-10.0 | Y-axis responsiveness (higher = more sensitive) |
| `lock_x_axis` | bool | false | true/false | Lock X at current position |
| `lock_y_axis` | bool | false | true/false | Lock Y at default position |
| `lock_wrist` | bool | false | true/false | Lock wrist at default angle |
| `wrist_sensitivity` | double | 1.0 | 0.1-5.0 | Wrist rotation responsiveness |
| `default_y_position` | double | -1.35 | -1.5 to -1.2 | Y position when locked (meters) |
| `default_wrist_angle` | int | 90 | 0-180 | Wrist angle when locked (degrees) |
| `default_z_position` | double | 1.11 | 0.5-2.0 | Fixed Z height (meters) |

## Quick Examples
```bash
# Default (all features enabled)
ros2 run esp32_controller esp32_controller

# X-axis only (Y and wrist locked)
ros2 run esp32_controller esp32_controller --ros-args \
  -p lock_y_axis:=true \
  -p lock_wrist:=true

# High sensitivity, fast updates
ros2 run esp32_controller esp32_controller --ros-args \
  -p callback_skip_rate:=2 \
  -p x_sensitivity:=3.0 \
  -p y_sensitivity:=3.0

# Safe/slow for testing
ros2 run esp32_controller esp32_controller --ros-args \
  -p callback_skip_rate:=10 \
  -p x_sensitivity:=0.5 \
  -p y_sensitivity:=0.5
```

## Workspace Bounds
- **X:** 0.6 to 0.77 meters
- **Y:** -1.5 to -1.2 meters  
- **Z:** Fixed at `default_z_position`

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


