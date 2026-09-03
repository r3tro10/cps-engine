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