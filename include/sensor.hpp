#pragma once
#include <string>

class Sensor
{
public:
    // Constructor and Destructor
    Sensor(std::string name);
    // The ~Sensor destructor deletes the original unique_ptr to prevent tangling refrences after transfering ownership
    // Crucial for cleaning up our heap buffer (RAII)
    ~Sensor();
    // Sensor is destroyed right after being created


    // --- README Requirement: Non-copyable sensor objects ---
    Sensor(const Sensor &) = delete;            // Disallow copy constructor
    Sensor &operator=(const Sensor &) = delete; // Disallow copy assignment

    // --- Allow Move Semantics (so std::unique_ptr and std::move work) ---
    Sensor(Sensor &&) noexcept = default;
    Sensor &operator=(Sensor &&) noexcept = default;

    void read_telemetry() const;

private:
    std::string name_;

    // --- README Requirement: Allocates a small heap buffer ---
    int* heap_buffer_;
};
