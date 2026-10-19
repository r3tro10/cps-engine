#include "control/pid.hpp"
#include <algorithm>

double PIDController::compute(double setpoint, double measured, double dt)
{
    if (dt <= 0.0)
    {
        return 0.0; // Fail-safe: zero effort if time hasn't advanced
    }

    const double error = setpoint - measured;

    integral_err_ += error * dt;
    integral_err_ = std::clamp(integral_err_, min_out_, max_out_); // Prevent integral windup for when errors accumulate very quickly

    double derivative = 0.0;
    if (is_first_step_)
    {
        is_first_step_ = false;
    }
    else
    {
        const double derivative = (measured - prev_measured_) / dt;
    }

    const double raw_output = (kp_ * error) + (ki_ * integral_err_) + (kd_ * derivative);

    prev_measured_ = measured;

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