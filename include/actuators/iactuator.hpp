#pragma once

#include "../core/types.hpp"

class IActuator
{
public:
    // Provides the definition of JointState.
    virtual ~IActuator() = default;

    // Without a virtual destructor, the derived destructor may not run correctly.
    virtual void setCommand(double target_effort) = 0;

    // Requires every concrete actuator to implement command handling.
    virtual JointState getState() const = 0;

    // Requires every concrete actuator to provide telemetry, while promising not to modify the actuator during the read.
    virtual void update(double dt) = 0;

    // Requires every concrete actuator to implement time-step processing.
    IActuator(const IActuator &) = delete;
    IActuator &operator=(const IActuator &) = delete;

    // Prevents copying and assigning actuator objects.
protected:
    IActuator() = default;
};
