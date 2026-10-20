#include "sensor.hpp"
#include <iostream>

// Constructor: Allocates the small heap buffer
Sensor::Sensor(std::string name) : name_(std::move(name))
{
    // Fulfilling the README: Allocating a small heap buffer (e.g., space for 10 integers)
    heap_buffer_ = new int[10]{0};

    std::cout << "[Hardware Connected] " << name_
              << " initialized. Heap buffer allocated at: " << heap_buffer_ << "\n";
}

// Destructor: Cleans up the buffer automatically when the object falls out of scope
Sensor::~Sensor()
{
    if (heap_buffer_ != nullptr)
    {
        delete[] heap_buffer_; // RAII: Fulfilling the "Automatic cleanup" promise
        std::cout << "[Hardware Disconnected] " << name_ << " released dynamic heap buffer.\n";
    }
}

void Sensor::read_telemetry() const
{
    std::cout << "[Telemetry] Reading from " << name_ << "...\n";
}
