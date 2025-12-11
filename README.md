# UAV Flight Controller

A complete autonomous flight control system demonstrating advanced embedded systems design for unmanned aerial vehicles. Designed for defense applications with Jetson Nano AI integration and Teensy 4.1 real-time control.

![Status](https://img.shields.io/badge/status-development-orange)
![Platform](https://img.shields.io/badge/platform-Teensy%204.1%20%7C%20Jetson%20Nano-blue)
![License](https://img.shields.io/badge/license-MIT-green)

## Overview

This project implements a modular flight controller architecture with sensor fusion, PID control, and AI-based vision navigation. The system targets tactical UAV applications similar to STM Defence's KARGU and TOGAN platforms.

### Target Applications
- Autonomous reconnaissance UAVs
- Loitering munition systems
- Surveillance and target tracking
- Formation flying and swarm coordination

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    UAV FLIGHT CONTROLLER                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐         ┌──────────────────┐              │
│  │   Teensy 4.1     │  UART   │   Jetson Nano    │              │
│  │  Flight Control  │◄───────►│  AI Navigation   │              │
│  │                  │         │                   │              │
│  │  • IMU Fusion    │         │  • Object Detect  │              │
│  │  • PID Control   │         │  • Path Planning  │              │
│  │  • Motor Mixing  │         │  • Visual SLAM    │              │
│  │  • Safety Logic  │         │  • GCS Comms      │              │
│  └────────┬─────────┘         └────────┬──────────┘              │
│           │                            │                         │
│           ▼                            ▼                         │
│  ┌─────────────────────────────────────────────────────┐        │
│  │              Hardware Interfaces                     │        │
│  │  [IMU] [GPS] [Baro] [ESCs] [Camera] [Radio]         │        │
│  └─────────────────────────────────────────────────────┘        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Features

### Flight Control
- **Sensor Fusion**: Extended Kalman Filter for IMU/GPS/Baro
- **PID Controllers**: Rate and attitude control loops
- **Motor Mixing**: Quad/Hex/Octo configurations
- **Flight Modes**: Stabilize, Altitude Hold, Loiter, Auto

### AI Navigation (Jetson Nano)
- **Object Detection**: YOLO-based target recognition
- **Visual Odometry**: Camera-based position estimation
- **Path Planning**: Obstacle avoidance and waypoint navigation
- **Target Tracking**: Real-time object following

### Communication
- **MAVLink Protocol**: Ground station compatibility
- **Telemetry**: Real-time status and sensor data
- **RC Input**: PPM/SBUS receiver support
- **Failsafe**: Return-to-launch, geofencing

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Teensy 4.1 | ARM Cortex-M7, 600 MHz |
| Jetson Nano | Quad-core ARM A57, 128 CUDA |
| IMU | MPU9250 (Accel/Gyro/Mag) |
| Barometer | BMP280/MS5611 |
| GPS | u-blox M8N |
| Camera | CSI or USB webcam |

## Project Structure

```
UAVFlightController/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── control_theory.md
│   └── hardware_setup.md
├── firmware/
│   └── teensy/
│       ├── src/
│       ├── include/
│       └── platformio.ini
├── jetson/
│   ├── src/
│   ├── include/
│   └── CMakeLists.txt
└── common/
    └── mavlink/
```

## Control Loop Architecture

```
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│  Pilot  │───►│ Position│───►│  Rate   │───►│  Motor  │
│ Commands│    │   PID   │    │   PID   │    │  Mixer  │
└─────────┘    └─────────┘    └─────────┘    └─────────┘
     ▲              │              │              │
     │              ▼              ▼              ▼
     │         ┌─────────────────────────────────────┐
     │         │        Sensor Fusion (EKF)          │
     └─────────│    IMU + GPS + Barometer           │
               └─────────────────────────────────────┘
```

## Author

**Cihan Erdem Gündoğdu** - Embedded Systems Engineer
