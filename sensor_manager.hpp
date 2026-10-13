#pragma once

#include "sensor.hpp"
#include <vector>
#include <memory>

/**
 * @brief Orchestrates the lifecycle and execution of the sensor fleet.
 * 
 * ARCHITECTURE: Maintains exclusive ownership of all sensor resources.
 * This ensures a single point of truth for hardware state management.
 */
class SensorManager {
public:
    SensorManager() = default;
    ~SensorManager() = default;

    /**
     * @brief Ingests a new sensor node into the fleet.
     * @param sensor Unique pointer transferring full ownership to this manager.
     */
    void add_sensor(std::unique_ptr<Sensor> sensor);

    // Triggers a synchronous data acquisition cycle across all managed nodes
    void read_all_sensors();

private:
    // Container uses unique_ptr to enforce strict ownership (no dangling references)
    std::vector<std::unique_ptr<Sensor>> sensors_;
};
