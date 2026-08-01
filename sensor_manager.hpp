#pragma once
#include "sensor.hpp"
#include <vector>
#include <memory>

class SensorManager {
private:
    // A dynamic collection of exclusively owned sensors
    std::vector<std::unique_ptr<Sensor>> sensorFleet;

public:
    SensorManager() = default;
    ~SensorManager() = default;

    // Accept ownership of a sensor and store it in the fleet
    void addSensor(std::unique_ptr<Sensor> sensor);

    // Iterate through all managed sensors and trigger readData()
    void readAllSensors() const;
};