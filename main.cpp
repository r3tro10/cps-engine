#include "sensor_manager.hpp"
#include <memory>
#include <iostream>

int main() {
    std::cout << "--- Booting CPS Fleet Engine ---\n";

    SensorManager manager;

    // Instantiate hardware components dynamically
    auto lidar = std::make_unique<Sensor>("LiDAR_Front");
    auto imu = std::make_unique<Sensor>("IMU_Primary");
    auto camera = std::make_unique<Sensor>("Depth_Cam_0");

    // Transfer ownership of sensors to the manager fleet
    manager.addSensor(std::move(lidar));
    manager.addSensor(std::move(imu));
    manager.addSensor(std::move(camera));

    // Notice: 'lidar', 'imu', and 'camera' in main() are now nullptr!
    // The manager is now the sole owner of all 3 heap objects.

    // Telemetry pass across all connected hardware
    manager.readAllSensors();

    std::cout << "\n--- Engine Shutting Down ---\n";
    return 0; 
    // When manager goes out of scope here, sensorFleet is destroyed,
    // which automatically calls the destructor (~Sensor()) for all 3 sensors!
}