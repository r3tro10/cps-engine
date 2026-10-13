#pragma once
#include <string>

class ISensor {
public:
    virtual ~ISensor() = default;

    // Pure virtual interface: must be implemented by concrete drivers
    virtual void readData() const = 0;
    virtual std::string getName() const = 0;

    // Non-copyable base to safeguard hardware ownership
    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;

protected:
    ISensor() = default;
};

#pragma once

#include <string>

/**
 * @brief Abstract hardware interface layer for physical system telemetries.
 * Forces derived nodes to implement safe lifecycle controls and explicit read procedures.
 */
class ISensor {
public:
    // Virtual destructor guarantees complete cleanup of derived allocations through base pointers
    virtual ~ISensor() = default;

    // Pure virtual method mapping to physical sensor data acquisition loops
    virtual void read_telemetry() = 0;
    
    // Returns immutable hardware identifier used for node tracking and routing
    virtual std::string get_name() const = 0;
};
