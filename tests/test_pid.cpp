#include <iostream>
#include <cassert>
#include <cmath>
#include "control/pid.hpp"
#include "actuators/mock_actuator.hpp"

void test_zero_negative_dt_safety() {
    PIDController pid(15.0, 0.0, 5.0, -10.0, 10.0);

    double out_zero = pid.compute(1.57, 0.0, 0.0);
    assert(!std::isnan(out_zero) && "PID output is NaN with dt = 0.0");
    assert(out_zero == 0.0 && "PID output should be 0.0 when dt <= 0.0");

    double out_neg = pid.compute(1.57, 0.0, -0.01);
    assert(!std::isnan(out_neg) && "PID output is NaN with negative dt");
    assert(out_neg == 0.0 && "PID output should be 0.0 when dt <= 0.0");
    
    std::cout << "[PASS] Test 1: Zero/Negative dt Safety Verified.\n";
}

void test_actuator_saturation_clamp() {
    PIDController pid(15.0, 0.0, 5.0, -10.0, 10.0);

    double out_saturated = pid.compute(100.0, 0.0, 0.02);
    assert(out_saturated == 10.0 && "PID output failed to clamp at upper limit");

    double out_neg_saturated = pid.compute(-100.0, 0.0, 0.02);
    assert(out_neg_saturated == -10.0 && "PID output failed to clamp at lower limit");

    std::cout << "[PASS] Test 2: Actuator Saturation Clamp Verified.\n";
}

void test_closed_loop_convergence() {
    // Match exact gains from src/main.cpp: Kp=15.0, Ki=0.0, Kd=5.0
    PIDController pid(15.0, 0.0, 5.0, -10.0, 10.0);
    MockActuator motor;

    const double target_rad = 1.57;
    const double dt = 0.02;
    const int steps = 300;

    for (int i = 0; i < steps; ++i) {
        motor.update(dt);
        JointState state = motor.getState();
        double effort = pid.compute(target_rad, state.position, dt);
        motor.setCommand(effort);
    }

    double final_position = motor.getState().position;
    double tracking_error = std::abs(target_rad - final_position);

    std::cout << "[INFO] Final convergence position after " << steps << " steps: " << final_position << " rad\n";
    std::cout << "[INFO] Tracking error: " << tracking_error << " rad\n";
    
    assert(tracking_error < 1e-4 && "PID control loop failed to converge within 1e-4 rad tolerance");
    std::cout << "[PASS] Test 3: Closed-Loop Convergence Verified.\n";
}

int main() {
    std::cout << "=== Running Deterministic PID Controller Test Suite ===\n";
    test_zero_negative_dt_safety();
    test_actuator_saturation_clamp();
    test_closed_loop_convergence();
    std::cout << "All unit tests passed successfully!\n";
    return 0;
}
