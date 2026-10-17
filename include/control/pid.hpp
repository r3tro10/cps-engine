#pragma once

class PIDController
{
// trailing _ to differentiate private members and parameters
private:
    double kp_;
    double ki_;
    double kd_;

    // min_out must be less than or equal to max_out
    double min_out_;
    double max_out_;

    double integral_err_ = 0.0;
    double prev_err_ = 0.0;

public:
    // Used a member-initiliser then define it in pid.cpp to properly initilise members to prevent undefined behavior
    PIDController(
        double kp, 
        double ki, 
        double kd, 
        double min_out, 
        double max_out);
    
    // dt must be greater than zero and is measured in seconds.
    double compute(double setpoint, double measured, double dt);
};