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

### Assembled Hardware
![Front Isometric](image_gallery/front_isometric.png)
![Top Down](image_gallery/top_down.png)
![Bottom Isometric](image_gallery/bottom_isometric.png)
![Folded Configuration](image_gallery/folded.png)

### Live Setup & Compute
![Front Isometric Live](image_gallery/front_isometric_live.png)
![Rear Isometric Live](image_gallery/rear_isometric_live.png)
![Bottom View Live](image_gallery/bottom_live.png)
![NVIDIA Jetson & OpenRB-150](image_gallery/jetson_and_openrb150.png)

### CAD Drawings Final Prototype
![Final Gimbal 1](final_gimbal_1.png)
![Final Gimbal 2](final_gimbal_2.png)
![Final Gimbal 3](final_gimbal_3.png)

### CAD Drawings Original Prototype
![Original Gimbal 1](original_gimbal_1.png)
![Original Gimbal 2](original_gimbal_2.png)
![Original Gimbal 3](original_gimbal_3.png)
![Original Gimbal 4](original_gimbal_4.png)
![Original Gimbal 5](original_gimbal_5.png)
