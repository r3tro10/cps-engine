#include "sensor.hpp"

#include <iostream>

Sensor::Sensor(const std::string& name)
	: sensorName(name), rawDataBuffer(new int(0)) {}

Sensor::~Sensor() {
	delete rawDataBuffer;
}

void Sensor::readData() const {
	std::cout << "Sensor: " << sensorName << ", buffer address: "
			  << static_cast<const void*>(rawDataBuffer) << '\n';
}
