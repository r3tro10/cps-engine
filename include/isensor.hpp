#pragma once

#include <string>

/**
 * @brief Abstract hardware interface layer for physical system telemetries.
 * Forces derived nodes to implement safe lifecycle controls and explicit read procedures.
 */
class ISensor
{
public:
    // 1. Virtual Destructor: Guarantees complete cleanup of derived allocations through base pointers
    virtual ~ISensor() = default;

    // 2. Pure Virtual Interface: Must be implemented by concrete drivers
    // Note: read_telemetry should be non-const because reading hardware often clears registers or updates states
    virtual double read_telemetry() = 0;

    // Note: get_name is const because reading a static string identifier never updates object state
    virtual std::string get_name() const = 0;

    // 3. Non-copyable base to safeguard hardware ownership
    ISensor(const ISensor &) = delete;
    ISensor &operator=(const ISensor &) = delete;

protected:
    // Allow derived classes to be default-constructed or moved safely
    ISensor() = default;
    ISensor(ISensor &&) noexcept = default;
    ISensor &operator=(ISensor &&) noexcept = default;
};
