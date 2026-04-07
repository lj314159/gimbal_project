# Autonomous Computer Vision Tracking Gimbal

A hardware-software integrated pan-tilt gimbal designed to replace imprecise manual tracking with autonomous target identification. The system is capable of maintaining a target in a centered frame across a 180-degree field of view with zero human intervention. 

## Technical Stack
* **Design & Manufacturing:** Onshape (CAD), Ultimaker S5 (3D Printing), PLA plastic.
* **Vision & Compute:** NVIDIA Jetson Xavier AGX, Arducam 2MP Global Shutter Camera. 
* **Software Pipeline:** MediaPipe (Inference), OpenCV (Image Processing, HUD Overlay).
* **Actuation & Microcontroller:** OpenRB-150, DYNAMIXEL XL430-W250-T Servos.

## System Architecture & State Machine
The firmware utilizes a deterministic finite state machine to manage system behavior and safety:

![State Diagram](state_diagram/state_diagram.png)

* **STARTUP:** Initial boot sequence handling hardware setup and controller initialization.
* **WAITING_FOR_CALIBRATION:** Awaits user commands to establish the zero/home positions for the pan and tilt axes.
* **READY:** Normal operation mode where the system accepts real-time movement and tracking commands.
* **RUNNING_TEST:** Executes an automated motion test sequence to validate mechanics before entering live tracking.
* **ERROR_STATE:** A safe-behavior fallback triggered by any faults (initialization, calibration, tracking, or test errors). A manual reset is required to return to the calibration state.

## Hardware & Mechanics
The physical assembly features a 3-legged base with integrated housing for the Dynamixel actuators to ensure stability. The mechanical design prioritizes structural rigidity and compactness to minimize the moment of inertia within the footprint of the servos. Motor mounts use reinforced geometry to eliminate flex during high-speed stops. 

## Control Methodology
The system always computes the correct absolute position directly from computer vision. It utilizes a closed-loop approach where the pixel distance from the target centroid to the frame center is calculated in real-time. 

To prevent unnecessary jitter and mechanical "hunting" during minor oscillations, a central deadband (buffer zone) is implemented. The gimbal remains stationary if the centroid is within this defined region. Once the target exits the buffer, the error is processed to update the servo position via the microcontroller. Internal torque sensing from the Dynamixel servos is used to compensate for the unbalanced center of mass and ensure smooth motion.

## Project Gallery

#### Live Test Demo
[![Watch the demo](https://img.youtube.com/vi/CTpUV5WMqHE/0.jpg)](https://www.youtube.com/watch?v=CTpUV5WMqHE)

#### Assembled Hardware
###### Front Isometric
![Front Isometric](image_gallery/front_isometric.png)
###### Top Down
![Top Down](image_gallery/top_down.png)
###### Bottom Isometric
![Bottom Isometric](image_gallery/bottom_isometric.png)
###### Folded Configuration
![Folded Configuration](image_gallery/folded.png)
###### Front Isometric Live
![Front Isometric Live](image_gallery/front_isometric_live.png)
###### Rear Isometric Live
![Rear Isometric Live](image_gallery/rear_isometric_live.png)
###### Bottom View Live
![Bottom View Live](image_gallery/bottom_live.png)
###### NVIDIA Jetson & OpenRB-150
![NVIDIA Jetson & OpenRB-150](image_gallery/jetson_and_openrb150.png)

## CAD Drawings Final Prototype
#### The final iteration utilizes a 3-legged base with integrated housing for the Dynamixel actuators for stability and aesthetics
###### Isometric View without yaw axis portion
![Final Gimbal 1](image_gallery/final_gimbal_1.png)
###### Internal Axis: Housed Actuator Assembly
![Final Gimbal 2](image_gallery/final_gimbal_2.png)
###### Main Chassis Plate: Leg Attachment Geometry
![Final Gimbal 3](image_gallery/final_gimbal_3.png)

## CAD Drawings Original Prototype
#### Initial proof-of-concept iteration focusing on component spacing, axis alignment, and basic structural stability
###### Prototype Assembly: Initial Prototype Layout
![Original Gimbal 1](image_gallery/original_gimbal_1.png)
###### Bottom View: Prototype Motor Mounting and Internal Housing
![Original Gimbal 2](image_gallery/original_gimbal_2.png)
###### Exploded View: All Reference Geometry Stack (Front)
![Original Gimbal 3](image_gallery/original_gimbal_5.png)
###### Exploded View: All Reference Geometry Stack (Rear)
![Original Gimbal 4](image_gallery/original_gimbal_4.png)
###### Axial Assembly: Component Alignment
![Original Gimbal 5](image_gallery/original_gimbal_3.png)
