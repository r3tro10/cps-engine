#include "sensor.hpp"
#include <iostream>

// Constructor: Allocates the heap memory
Sensor::Sensor(const std::string& name) : sensorName(name) {
    std::cout << "[Hardware] Connecting to " << sensorName << "...\n";
    rawDataBuffer = new int(0); // 🏗️ Allocate on the Heap
}

// Destructor: Safely deletes the heap memory
Sensor::~Sensor() {
    std::cout << "[Hardware] Disconnecting " << sensorName << " and freeing memory...\n";
    delete rawDataBuffer; // 💧 Prevent memory leak
}

// Read method
void Sensor::readData() const {
    std::cout << "[Hardware] Reading from " << sensorName 
              << " at heap address: " << rawDataBuffer << "\n";
}