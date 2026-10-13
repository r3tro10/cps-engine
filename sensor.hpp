#pragma once

#include <string>
#include <iostream>

/**
 * @brief Concrete Sensor implementation.
 * Represents a physical hardware node in the simulation loop.
 * 
 * DESIGN NOTE: Uses strict RAII patterns to ensure hardware resources 
 * are released deterministically when the object goes out of scope.
 */
class Sensor {
public:
    // Explicit: Prevents accidental implicit type conversions (e.g., string -> Sensor)
    explicit Sensor(std::string name);

    // Default destructor handles automatic cleanup of stack members
    ~Sensor();

    // Simulates the I/O operation of fetching a data frame from hardware
    void read_telemetry();

    // Accessor for the immutable hardware ID
    std::string get_name() const;

private:
    std::string name_; // Unique hardware identifier used for logging/debugging
};
