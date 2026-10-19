#include "sensor.hpp"
#include "sensor_manager.hpp"
#include "actuators/iactuator.hpp"
#include "control/pid.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>

class MockActuator : public IActuator
{
private:
    JointState state_{};

public:
    void setCommand(double target_effort) override
    {
        state_.effort = target_effort;
    }
    JointState getState() const override
    {
        return state_;
    }
    void update(double dt) override
    {
        state_.velocity += state_.effort * dt;
        state_.velocity *= 0.95;
        state_.position += state_.velocity * dt;
    }
};

int main()
{
    std::cout << "=== CPS Engine: Boot Sequence ===\n\n";

    // 1. Instantiation: Create concrete hardware drivers on the heap
    // Note: Using make_unique for exception-safe memory allocation
    auto lidar = std::make_unique<Sensor>("LiDAR_Front");
    auto imu = std::make_unique<Sensor>("IMU_Primary");
    auto camera = std::make_unique<Sensor>("Depth_Cam_0");

    PIDController pid(2.0, 0.1, 0.5, -10.0, 10.0);
    MockActuator motor;

    const double target_rad = 1.57;
    const double dt = 0.02;
    for (int step = 0; step < 100; ++step) // increment step at the end
    {
        JointState state = motor.getState();
        double effort = pid.compute(target_rad, state.position, dt);
        motor.setCommand(effort);
        motor.update(dt);

        std::cout << state.position + effort;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    SensorManager manager;

    std::cout << "\n[LIFECYCLE] Transferring ownership to SensorManager...\n";

    // 2. Dependency Injection: Move ownership into the core manager.
    // 'std::move' signals that 'main' no longer controls these resources.
    manager.add_sensor(std::move(lidar));
    manager.add_sensor(std::move(imu));
    manager.add_sensor(std::move(camera));

    std::cout << "\n[EXECUTION] Starting telemetry acquisition cycle:\n";
    manager.read_all_sensors();

    // 3. Shutdown: Exiting scope triggers the Manager's destructor.
    // This automatically cascades the deletion of all contained Sensors.
    std::cout << "\n[SHUTDOWN] System halt. Initiating RAII teardown...\n";

    return 0;
}
