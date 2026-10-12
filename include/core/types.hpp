#include <cstdint>
struct JointState
{
    double position; // measured in radians
    double velocity; // angluar speed measured in radians per second (rad/s)
    double effort; // motor torque OR current (in Nm?)
};

struct ControlCommand
{
    double setpoint; // target poistion for joint measured in radians
    uint64_t timestamp_ns;
};