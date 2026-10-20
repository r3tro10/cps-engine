#pragma once

#include "iactuator.hpp"

class MockActuator : public IActuator
{
private:
    JointState state_{};

public:
    void setCommand(double target_effort) override;
    JointState getState() const override;
    void update(double dt) override;
};