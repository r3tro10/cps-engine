#pragma once
#include <string>

class Sensor {
private:
    std::string sensorName;
    int* rawDataBuffer; // Simulating a raw hardware memory buffer on the Heap

public:
    // Constructor: Called automatically when the object is created.
    // Use this to "connect" to hardware and allocate the heap memory.
    Sensor(const std::string& name);

    // Destructor (starts with ~): Called automatically when the object is destroyed.
    // Use this to "disconnect" and delete the heap memory.
    ~Sensor();

    // FATAL FLAW PREVENTION: Delete copy constructors.
    // If you copy this object, both copies will try to 'delete' the same rawDataBuffer
    // when destroyed, causing a "Double Free" crash.
    Sensor(const Sensor&) = delete;
    Sensor& operator=(const Sensor&) = delete;

    // A read-only method to print the data
    void readData() const;
};