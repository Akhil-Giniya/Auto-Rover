# Autonomous Underwater Submarine System Architecture

## 1. Full Architecture

The system is designed as a distributed, modular architecture split between a high-level cognitive layer (Raspberry Pi 4 Model B) and a low-level real-time control layer (ESP32). The system operates fully autonomously, untethered, and without external communication during missions.

### High-Level Layer (Raspberry Pi 4)
Responsible for heavy computation, vision, AI, and mission planning.
- **Vision Package:** Processes Pi Camera feed for object, gap, and color detection.
- **Sensor Fusion Package:** Combines IMU and Depth Sensor data for accurate orientation and depth estimation.
- **Localization Package:** Maintains estimated position using dead reckoning and visual odometry.
- **Path Planning Package:** Calculates optimal paths avoiding obstacles and targeting goals.
- **Autonomy Core:** High-level state machine orchestrating mission phases (Idle, Explore, Target, Navigate, Avoid, Align, Pick, Return).

### Low-Level Real-Time Layer (ESP32)
Responsible for deterministic hardware control.
- **HW Interface Layer:** Translates commands into PWM/actuator signals for thrusters and reads basic hardware state.
- **Manipulation Package:** Manages precise actuation of the robotic arm based on calculated pick positions.

---

## 2. Tech Stack Choice Justification

- **OS Layer:** Linux (Ubuntu Server/Debian) on Raspberry Pi, FreeRTOS on ESP32.
  *Justification:* Linux provides the vast ecosystem needed for OpenCV, Python, and middleware. FreeRTOS guarantees real-time determinism for motor control.
- **Language:** Python 3 (Pi - High Level/AI) and C++ (Pi - Core algorithms & ESP32).
  *Justification:* Python accelerates AI/Vision iteration. C++ ensures low latency for EKF and real-time control.
- **Vision:** OpenCV + Lightweight TensorRT/TFLite models.
  *Justification:* Necessary for handling underwater light distortion, color correction, and running on edge hardware (Pi 4).
- **Control Theory:** Extended Kalman Filter (EKF) for sensor fusion.
  *Justification:* Standard, robust method for dealing with noisy underwater IMU and pressure sensors.
- **State Machine:** SMACH or YASMIN (or custom Python implementation).
  *Justification:* Deterministic and testable mission logic.

---

## 3. Communication Framework

- **Inter-Process Communication (IPC) on Raspberry Pi: ZeroMQ (ZMQ)**
  *Justification:* ROS2 is heavy and complex to set up deterministically on low-resource edge devices without a stable network. ZeroMQ provides lightweight, brokerless, and extremely fast pub/sub and req/rep messaging between our independent modules. Protocol Buffers or MessagePack will be used for serialization to maintain strict data contracts.
- **Pi to ESP32 Communication: UART / Serial with Checksums (e.g., Mavlink or custom struct)**
  *Justification:* Direct, reliable, and requires no network stack. A simple binary protocol ensures low latency and robustness.

---

## 4. Package-by-Package Roadmap

1. **hw_interface:** Establish reliable comms between Pi and ESP32. Get thrusters spinning.
2. **sensor_fusion:** Read IMU/Depth, apply EKF, output stable Pose/Depth topics.
3. **vision:** Calibrate camera, apply underwater color correction, implement object/color detection.
4. **localization:** Integrate sensor fusion and vision to track X, Y, Z, Yaw over time.
5. **path_planning:** Implement vector-field or simple A* for gap navigation and target approach.
6. **manipulation:** Implement ESP32 arm inverse kinematics and test pickup triggers.
7. **autonomy_core:** Tie all modules together via the state machine. Execute full mission loop.

---

## 5. Testing Plan

Each module will follow a strict Test-Driven approach:
- **Unit Tests:** Run via `pytest` or `Catch2`. Focus on algorithmic correctness (e.g., EKF math, A* paths).
- **Runtime Tests (`run.sh` / `debug.sh`):** Module can be spun up in isolation, generating mock data if inputs are missing.
- **Simulation Fallback:** A simple 2D Python simulator will publish mock ZeroMQ sensor data and subscribe to thruster commands to verify control logic without hardware.
- **Fault Injection:** Randomly dropping ZeroMQ messages or simulating sensor timeouts to test module recovery and timeout detection.

---

## 6. Integration Strategy

1. **Dry-Run Mode:** Run all modules on Pi with zero thruster output. Verify states transition correctly via logs.
2. **Pool Testing (Tethered for debugging):** Run basic depth hold and heading hold. Adjust EKF and PID tunings.
3. **Vision Integration:** Test color and object detection in the pool.
4. **Waypoint Navigation:** Command open-loop movements and verify closed-loop response.
5. **Full Autonomous Untethered Mission:** Execute the full state machine.
