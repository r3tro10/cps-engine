#include "actuators/mock_actuator.hpp"

void MockActuator::setCommand(double target_effort)
{
    state_.effort = target_effort;
}

JointState MockActuator::getState() const
{
    return state_;
}

void MockActuator::update(double dt)
{
    state_.velocity += state_.effort * dt;
    state_.velocity *= 0.95;
    state_.position += state_.velocity * dt;
}