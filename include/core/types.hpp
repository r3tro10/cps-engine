#pragma
#include <cstdint>
struct JointState
{
    double position = 0.0; // measured in radians
    double velocity = 0.0; // angular speed measured in radians per second (rad/s)
    double effort = 0.0; // motor torque OR current (in Nm?)
};

struct ControlCommand
{
    double setpoint = 0.0; // target position for joint measured in radians
    uint64_t timestamp_ns = 0; // exact nanosecond timestamp
};
