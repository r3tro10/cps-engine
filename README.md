# CPS-Engine: C++20 Cyber-Physical Control & HAL Framework

A modular C++20 control and hardware abstraction engine designed for deterministic closed-loop actuation, sensor fleet lifecycle management, and spatial robotics simulation.

The end-to-end processing pipeline follows: **Simulate/Step → Observe State → Compute Effort → Actuate Command**.

---

## Implemented Architecture (Phase 1)

* **Closed-Loop PID Control (`PIDController`):** Bounded discrete-time feedback controller featuring actuator saturation clamping (`std::clamp`), integral anti-windup, zero/negative time-step fail-safes, and derivative-on-measurement to eliminate setpoint derivative kick.
* **Hardware Abstraction Layer (`IActuator` / `ISensor`):** Pure virtual, non-copyable hardware contracts decoupling high-level control loops from underlying driver physics and telemetry buses.
* **Deterministic 50 Hz Execution Loop:** Monotonic clock synchronization (`std::chrono::steady_clock`) driving second-order damped actuator dynamics to sub-microradian steady-state convergence.
* **RAII Resource Management (`SensorManager`):** Exclusive hardware handle ownership via `std::unique_ptr` and move semantics (`std::move`), guaranteeing deterministic teardown without manual memory management.
* **Zero-Sleep Verification Suite (CTest):** Deterministic unit tests verifying saturation bounds, bad time-step rejection, derivative-kick suppression, and closed-loop convergence in under 10 ms.
* **3D Spatial Kinematics (`KinematicsSolver`):** Forward kinematics via `forwardKinematics` returning `Eigen::Matrix4d` and inverse kinematics via `inverseKinematics` returning `std::optional<Eigen::Vector3d>`, with positive link-length validation.



## Project Structure

```text
cps-engine/
├── CMakeLists.txt
├── include/
│   ├── actuators/
│   │   ├── iactuator.hpp
│   │   └── mock_actuator.hpp
│   ├── control/
│   │   └── pid.hpp
│   ├── core/
│   │   └── types.hpp
│   ├── kinematics/
│   │   └── kinematics.hpp
│   └── sensors/
├── src/
│   ├── actuators/mock_actuator.cpp
│   ├── control/pid.cpp
│   ├── kinematics/kinematics.cpp
│   ├── sensor.cpp
│   ├── sensor_manager.cpp
│   └── main.cpp
└── tests/
    ├── test_kinematics.cpp
    └── test_pid.cpp
```

## Class Diagram

```mermaid
classDiagram
    direction LR

    class JointState {
        <<struct>>
        +position: double
        +velocity: double
        +effort: double
    }

    class ControlCommand {
        <<struct>>
        +setpoint: double
        +timestamp_ns: uint64_t
    }

    class IActuator {
        <<interface>>
        +~IActuator()
        +setCommand(target_effort: double) void
        +getState() JointState
        +update(dt: double) void
        #IActuator()
    }

    class MockActuator {
        -state_: JointState
        +setCommand(target_effort: double) void
        +getState() JointState
        +update(dt: double) void
    }

    class ISensor {
        <<interface>>
        +~ISensor()
        +read_telemetry() double
        +get_name() string
        #ISensor()
    }

    class Sensor {
        -name_: string
        -heap_buffer_: int*
        +Sensor(name: string)
        +~Sensor()
        +read_telemetry() void
    }

    class SensorManager {
        -sensors_: vector~unique_ptr~Sensor~~
        +SensorManager()
        +~SensorManager()
        +add_sensor(sensor: unique_ptr~Sensor~) void
        +read_all_sensors() void
    }

    class PIDController {
        -kp_: double
        -ki_: double
        -kd_: double
        -min_out_: double
        -max_out_: double
        -integral_err_: double
        -prev_measured_: double
        -is_first_step_: bool
        +PIDController(kp: double, ki: double, kd: double, min_out: double, max_out: double)
        +compute(setpoint: double, measured: double, dt: double) double
    }

    class ArmGeometry {
        <<struct>>
        +base_height: double
        +upper_arm_length: double
        +forearm_length: double
    }

    class KinematicsSolver {
        -geometry_: ArmGeometry
        +KinematicsSolver(geometry: ArmGeometry)
        +forwardKinematics(q: Vector3d) Matrix4d
        +inverseKinematics(target_xyz: Vector3d) optional~Vector3d~
    }

    MockActuator ..|> IActuator : implements
    MockActuator *-- JointState : holds
    SensorManager *-- Sensor : owns via unique_ptr
    KinematicsSolver *-- ArmGeometry : configured by
```

## Sequence Diagram

```mermaid
sequenceDiagram
    autonumber
    actor main as Main (main.cpp)
    participant sm as SensorManager
    participant sensor as Sensor Nodes (LiDAR, IMU, Cam)
    participant motor as MockActuator
    participant pid as PIDController

    rect rgb(40, 44, 52)
        Note over main, sensor: Boot Sequence & RAII Ownership Transfer
        main->>+sensor: make_unique<Sensor>(name)
        sensor-->>-main: unique_ptr<Sensor> (allocates heap_buffer_)
        main->>sm: add_sensor(std::move(sensor))
        main->>+sm: read_all_sensors()
        sm->>+sensor: read_telemetry()
        sensor-->>-sm: log telemetry
        sm-->>-main: cycle complete
    end

    rect rgb(35, 45, 55)
        Note over main, pid: Deterministic 50 Hz Closed-Loop Control (300 Steps)
        loop Every 20ms Tick (dt = 0.02s)
            main->>motor: update(dt)
            main->>+motor: getState()
            motor-->>-main: JointState (position, velocity, effort)
            main->>+pid: compute(target_rad, state.position, dt)
            pid-->>-main: effort: double (clamped)
            main->>motor: setCommand(effort)
        end
    end

    rect rgb(50, 38, 42)
        Note over main, sensor: Scope Exit & Deterministic RAII Teardown
        main->>sm: ~SensorManager()
        sm->>sensor: ~Sensor() (delete[] heap_buffer_)
    end

```


---


## Active Development Roadmap
- Phase 2 (Implemented): 3D Forward & Inverse Kinematics solver utilizing Eigen3 homogeneous transformations and workspace boundary validation.
- Phase 3 (Planned): Containerized ROS 2 joint-state publisher and RViz 6-DOF manipulator visualization.

## Build and Run

### Requirements
- CMake 3.10+
- C++20 compatible compiler (Clang / GCC / MSVC)
- Eigen3
- macOS or Linux

### Build & Verification
From the project root:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/cps_engine
```
<details>
<summary>View Sample Run Output</summary>

```text
=== CPS Engine: Boot Sequence ===
[Hardware Connected] LiDAR_Front initialized. Heap buffer allocated at: 0x104eea030
[LIFECYCLE] Transferring ownership to SensorManager...
[Step 0] Pos: 0 rad | Err: 1.57 rad | Effort: 10
...
[Step 299] Pos: 1.57 rad | Err: 2.63767e-07 rad | Effort: 1.93673e-05
[SHUTDOWN] System halt. Initiating RAII teardown...
```
</details>


---
