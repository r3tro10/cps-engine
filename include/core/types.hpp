#pragma once

#include <cstdint>

struct JointState
{
    double position = 0.0; // [rad]
    double velocity = 0.0; // [rad/s]

    // TODO(r3tro10): Confirm if driver reports raw motor current or calculated torque [Nm]
    double effort = 0.0;
};

struct ControlCommand
{
    double setpoint = 0.0;     // Target joint position [rad]
    uint64_t timestamp_ns = 0; // Monotonic system clock for jitter tracking
};
