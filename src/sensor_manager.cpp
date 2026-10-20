#include "sensor_manager.hpp"

void SensorManager::add_sensor(std::unique_ptr<Sensor> sensor)
{
    if (sensor)
    {
        // Move semantics: Transfer ownership from caller (Stack) to Manager (Vector)
        sensors_.push_back(std::move(sensor));
    }
}

void SensorManager::read_all_sensors()
{
    // Iterate through the fleet to execute the control loop cycle
    for (const auto &sensor : sensors_)
    {
        sensor->read_telemetry();
    }
}
