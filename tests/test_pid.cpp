#include <iostream>
#include <cmath>

// Force assertions to remain active even in CMake -DCMAKE_BUILD_TYPE=Release builds
#undef NDEBUG
#include <cassert>

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

void test_derivative_kick_prevention() {
    // Pure Derivative controller (Kp = 0.0, Ki = 0.0, Kd = 5.0)
    PIDController d_only(0.0, 0.0, 5.0, -10.0, 10.0);

    // Sudden step change in setpoint (0.0 -> 1.57) while motor is stationary at 0.0
    double out = d_only.compute(1.57, 0.0, 0.02);
    assert(out == 0.0 && "Derivative kick detected on initial step!");

    std::cout << "[PASS] Test 3: Derivative Kick Prevention Verified.\n";
}

void test_closed_loop_convergence() {
    PIDController pid(15.0, 0.0, 5.0, -10.0, 10.0);
    MockActuator motor;

    const double target_rad = 1.57;
    const double dt = 0.02;
    const int steps = 300;

    // Strict causality: Sense -> Plan -> Actuate -> Step Physics
    for (int i = 0; i < steps; ++i) {
        JointState state = motor.getState();
        double effort = pid.compute(target_rad, state.position, dt);
        motor.setCommand(effort);
        motor.update(dt);
    }

    double final_position = motor.getState().position;
    double tracking_error = std::abs(target_rad - final_position);

    std::cout << "[INFO] Final convergence position after " << steps << " steps: " << final_position << " rad\n";
    std::cout << "[INFO] Tracking error: " << tracking_error << " rad\n";

    assert(tracking_error < 1e-4 && "PID control loop failed to converge within 1e-4 rad tolerance");
    std::cout << "[PASS] Test 4: Closed-Loop Convergence Verified.\n";
}

int main() {
    std::cout << "=== Running Deterministic PID Controller Test Suite ===\n";
    test_zero_negative_dt_safety();
    test_actuator_saturation_clamp();
    test_derivative_kick_prevention();
    test_closed_loop_convergence();
    std::cout << "All unit tests passed successfully!\n";
    return 0;
}