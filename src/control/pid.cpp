#include "control/pid.hpp"
#include <algorithm>

double PIDController::compute(double setpoint, double measured, double dt)
{
    const double error = setpoint - measured;

    integral_err_ += error * dt;

    const double derivative = (error - prev_err_) / dt;

    const double raw_output =
        (kp_ * error) +
        (ki_ * integral_err_) +
        (kd_ * derivative);

    prev_err_ = error;

    return std::clamp(raw_output, min_out_, max_out_);
}

PIDController::PIDController(
    double kp,
    double ki,
    double kd,
    double min_out,
    double max_out)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      min_out_(min_out),
      max_out_(max_out)
{
}