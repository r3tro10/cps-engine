#include "sensor_manager.hpp"
#include <iostream>

void SensorManager::addSensor(std::unique_ptr<Sensor> sensor) {
    // std::unique_ptr cannot be copied. 
    // We MUST use std::move to transfer ownership into the vector.
    sensorFleet.push_back(std::move(sensor));
}

void SensorManager::readAllSensors() const {
    std::cout << "\n--- Reading Telemetry From All Fleet Sensors ---\n";
    for (const auto& sensor : sensorFleet) {
        if (sensor != nullptr) {
            sensor->readData(); // Borrowed call to member function
        }
    }
}