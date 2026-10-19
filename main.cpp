#include "sensor.hpp"
#include "sensor_manager.hpp"
#include <iostream>
#include <memory>

class MockActuator : public IActuator
{
};

int main()
{
    std::cout << "=== CPS Engine: Boot Sequence ===\n\n";

    // 1. Instantiation: Create concrete hardware drivers on the heap
    // Note: Using make_unique for exception-safe memory allocation
    auto lidar = std::make_unique<Sensor>("LiDAR_Front");
    auto imu = std::make_unique<Sensor>("IMU_Primary");
    auto camera = std::make_unique<Sensor>("Depth_Cam_0");

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
