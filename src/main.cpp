#include "sensor.hpp"
#include "sensor_manager.hpp"
#include "actuators/mock_actuator.hpp"
#include "control/pid.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

int main()
{
    std::cout << "=== CPS Engine: Boot Sequence ===\n\n";

    auto lidar = std::make_unique<Sensor>("LiDAR_Front");
    auto imu = std::make_unique<Sensor>("IMU_Primary");
    auto camera = std::make_unique<Sensor>("Depth_Cam_0");

    SensorManager manager;

    std::cout << "\n[LIFECYCLE] Transferring ownership to SensorManager...\n";

    manager.add_sensor(std::move(lidar));
    manager.add_sensor(std::move(imu));
    manager.add_sensor(std::move(camera));

    std::cout << "\n[EXECUTION] Starting telemetry acquisition cycle:\n";
    manager.read_all_sensors();

    PIDController pid(15.0, 0.0, 2.0, -10.0, 10.0);
    MockActuator motor;

    constexpr double target_rad = 1.57;
    constexpr double dt = 0.02;

    using clock = std::chrono::steady_clock;
    const auto loop_period = std::chrono::milliseconds(20);
    auto next_tick = clock::now();

    for (int step = 0; step < 300; ++step)
    {
        next_tick += loop_period;

        motor.update(dt);
        JointState state = motor.getState();

        double effort = pid.compute(target_rad, state.position, dt);
        motor.setCommand(effort);

        std::cout << "[Step " << step << "] "
                  << "Pos: " << state.position << " rad | "
                  << "Err: " << target_rad - state.position << " rad | "
                  << "Effort: " << effort << "\n";

        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "\n[SHUTDOWN] System halt. Initiating RAII teardown...\n";

    return 0;
}