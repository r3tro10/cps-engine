#pragma once
#include <types.hpp>

class IActuator
{
private:
    /* data */
public:
    IActuator(/* args */);
    virtual ~IActuator() = default; // vitual destructor
    
    virtual void setCommand(double targeteffort) = 0; // pure vitual function
};

IActuator::IActuator(/* args */)
{
}

IActuator::~IActuator()
{
}
