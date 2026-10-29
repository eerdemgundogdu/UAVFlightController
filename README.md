# UAVFlightController

Teensy 4.1 flight controller firmware (500 Hz control loop using i.MX RT1062 GPT hardware timers) and Jetson Nano companion nodes for visual odometry and serial telemetry.

## Structure

- `firmware/teensy/`: PlatformIO project for Teensy 4.1 (`main.cpp`, `sensors.cpp`, `sensor_fusion.cpp`, `control.cpp`, `mavlink.cpp`). Uses `ARM_DWT_CYCCNT` and `GPT1` for loop timing, cascaded attitude/rate PID and MAVLink framing.
- `jetson/`: C++ companion application (`visual_odometry.cpp`, `object_detector.cpp`, `serial_comm.cpp`).

## Build

```bash
cd firmware/teensy
pio run

cd ../../jetson
mkdir -p build && cd build
cmake .. && make
```
