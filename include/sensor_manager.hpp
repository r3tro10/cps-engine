#pragma once

#include "sensor.hpp"
#include <vector>
#include <memory>

class SensorManager
{
public:
    SensorManager() = default;
    ~SensorManager() = default;

    void add_sensor(std::unique_ptr<Sensor> sensor);

    // Triggers a synchronous data acquisition cycle across all managed nodes
    void read_all_sensors();

private:
    // Container uses unique_ptr to enforce strict ownership (no dangling references)
    std::vector<std::unique_ptr<Sensor>> sensors_;
};
