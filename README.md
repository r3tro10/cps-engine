# CPS Engine

A C++20 Cyber-Physical Systems (CPS) and Physical AI control prototype featuring a deterministic 50 Hz closed-loop joint control, RAII hardware abstraction, and sensor lifecycle management.

The end-to-end processing pipeline follows: **Sense → Estimate → Plan → Actuate → Observe**.

---

## Key Features & Architecture
- **Closed-Loop Control & Physics (`src/control/`, `src/actuators/`):** Features a discrete-time `PIDController` utilizing derivative-on-measurement, first-step guards, and integral anti-windup/saturation. Paired with an `IActuator` interface and a `MockActuator` that simulates 1-DOF joint dynamics via semi-implicit Euler integration.
- **Real-Time Scheduling:** Executes a 50 Hz control loop (`dt = 0.02s`) via `std::chrono::steady_clock` and `std::this_thread::sleep_until`.
- **RAII Sensor Management (`src/sensor*.cpp`):** Implements strict resource ownership using `std::unique_ptr`, non-copyable semantics, and deterministic teardown.

## Class Diagram

```mermaid
classDiagram
    direction LR

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
        -heap_buffer_: int_ptr
        +Sensor(name: string)
        +~Sensor()
        +read_telemetry() void
    }

    class SensorManager {
        -sensors_: vector~Sensor~
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

    MockActuator ..|> IActuator : implements
    SensorManager *-- Sensor : owns via unique_ptr
```

## Sequence Diagram

```mermaid
sequenceDiagram
    autonumber
    actor main as Main Loop (main.cpp)
    participant motor as Motor (MockActuator)
    participant pid as Controller (PIDController)

    loop Every 20ms Tick (300 Steps)
        main->>motor: update(dt)

        main->>+motor: getState()
        motor-->>-main: JointState (position, velocity, effort)

        main->>+pid: compute(target_rad, state.position, dt)
        pid-->>-main: effort: double

        main->>motor: setCommand(effort)
    end
```


---

## Build and Run

### Requirements
- CMake 3.10+
- C++20 compatible compiler (Clang / GCC / MSVC)
- macOS or Linux

### Build & Execution
From the project root:
```bash
cmake -S . -B build
cmake --build build
./build/cps_engine
```

---
