#include "sensor.hpp"

Sensor::Sensor(std::string name) : name_(std::move(name)) {
    // LOG: Trace hardware initialization events for system audit
    std::cout << "[SYSTEM] Sensor initialized: " << name_ << "\n";
}

Sensor::~Sensor() {
    // LOG: Verify resource release sequence during shutdown
    std::cout << "[SYSTEM] Sensor destroyed (RAII Cleanup): " << name_ << "\n";
}

void Sensor::read_telemetry() {
    // TODO(r3tro10): Replace console output with actual serial/I2C buffer read
    std::cout << "Reading telemetry from: " << name_ << "...\n";
}

std::string Sensor::get_name() const {
    return name_;
}
