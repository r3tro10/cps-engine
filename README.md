# CPS Engine

A C++20 learning project evolving toward a simulation-first Cyber-Physical Systems and Physical AI platform.

The long-term goal is to connect perception, state estimation, planning, control, and simulated robot actuation in a closed loop:

**Sense → Estimate → Plan → Actuate → Observe**

The current version is an educational sensor-management prototype. The planned system will grow into a Dockerized ROS 2 application controlling a simulated 6-DOF robotic arm using C++ control software and YOLO/ONNX perception.

## Current Status

The repository currently demonstrates:

- Modern C++20 project configuration with CMake
- Sensor object construction and destruction
- Exclusive ownership with `std::unique_ptr`
- Ownership transfer using `std::move`
- Non-copyable sensor objects
- Automatic cleanup through RAII
- A manager class responsible for a collection of sensors
- Simulated telemetry reads from multiple named sensors

The current implementation does not yet include real hardware drivers, robot control, ROS 2, kinematics, computer vision, or machine-learning inference.

## Current Demonstration

The executable creates three simulated sensors:

- `LiDAR_Front`
- `IMU_Primary`
- `Depth_Cam_0`

Ownership is transferred from `main()` to `SensorManager`. The manager then reads from every sensor and automatically releases them when it goes out of scope.

This demonstrates the ownership flow:

```text
main()
  |
  | creates sensors
  v
std::unique_ptr<Sensor>
  |
  | std::move()
  v
SensorManager
  |
  | readAllSensors()
  v
sensor telemetry
  |
  | manager destruction
  v
automatic resource cleanup
```

## Build and Run

### Requirements

- CMake 3.10 or newer
- A C++20-compatible compiler
- macOS, Linux, or another platform supported by the compiler and CMake

### Build

From the project root:

```bash
cmake -S . -B build
cmake --build build
```

### Run

```bash
./build/cps_engine
```

The output includes simulated hardware connection messages, telemetry reads, and cleanup messages during shutdown. Heap addresses printed by the demo will vary between runs.

## Project Structure

```text
.
├── CMakeLists.txt
├── main.cpp
├── sensor.hpp
├── sensor.cpp
├── sensor_manager.hpp
├── sensor_manager.cpp
├── isensor.hpp
└── learning/
    ├── concepts.md
    ├── mistakes.md
    ├── questions.md
    └── review.md
```

### Components

- `main.cpp`  
  Application entry point and current sensor-fleet demonstration.

- `sensor.hpp` / `sensor.cpp`  
  Concrete simulated sensor implementation. It currently allocates a small heap buffer to demonstrate resource ownership and cleanup.

- `sensor_manager.hpp` / `sensor_manager.cpp`  
  Owns sensors in a `std::vector<std::unique_ptr<Sensor>>` and provides fleet-wide reading.

- `isensor.hpp`  
  Initial abstract sensor interface. It is currently prepared for future use but is not yet implemented by `Sensor`.

- `CMakeLists.txt`  
  Defines the C++20 executable and build configuration.

## Planned Architecture

The project will evolve in stages toward the following architecture:

```text
Camera / Sensors
       |
       v
Perception and State Estimation
       |
       v
Target Pose / Robot State
       |
       v
Kinematics and Planning
       |
       v
Controller
       |
       v
Actuator Interface
       |
       v
Simulated 6-DOF Robot Arm
```

The reusable control and robotics code will remain independent from ROS 2 where possible. ROS 2 and simulator-specific code will act as integration layers around the core library.

## Development Roadmap

### Phase 1: Core Control

- Define measurement, command, timestamp, frame, and status types
- Replace concrete sensor dependencies with interfaces
- Add an actuator interface
- Add a deterministic simulated arm backend
- Implement forward and inverse kinematics using Eigen
- Add bounded controller commands and safety limits
- Add unit tests with a deterministic fake clock

### Phase 2: ROS 2 Simulation

- Add a Docker-based ROS 2 development environment
- Add a generic 6-DOF arm URDF
- Connect joint-state feedback to the core library
- Send controller commands to the simulated arm
- Add launch files, parameters, and RViz visualization
- Measure control-loop rate and timing jitter

### Phase 3: Physical AI Perception

- Add camera or depth input
- Integrate YOLO/ONNX object detection
- Transform detected objects into robot coordinates
- Convert detections into end-effector targets
- Connect perception output to inverse kinematics and control
- Demonstrate the simulated arm tracking a detected object

### Phase 4: Reliability and Deployment

- Add handling for stale data, failed inference, simulator disconnects, and invalid targets
- Separate perception and control scheduling
- Add bounded queues or atomic state snapshots
- Add continuous integration for builds and tests
- Document measured performance and known limitations
- Add a recorded end-to-end demonstration

## Engineering Goals

This project is intended to demonstrate practical skills in:

- Modern C++
- Resource ownership and RAII
- Object-oriented design
- Hardware abstraction
- Robotics software architecture
- Kinematics and coordinate frames
- Feedback control
- Simulation-first development
- Perception-to-action pipelines
- Testing and failure handling
- Docker and ROS 2 integration

## Scope

This is currently a learning and portfolio project, not a production robotics engine.

The initial implementation will focus on simulation and reproducibility. Physical motor drivers, CAN or serial communication, hard real-time guarantees, safety certification, and deployment to a physical robot will only be added after a specific hardware platform and safety model have been selected.

## License

No license has been selected yet.
```

This version presents the existing code honestly while giving placement reviewers a clear view of the intended engineering direction.

Created 5 todos
