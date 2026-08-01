#pragma once

#include <string>

class Sensor {
private:
	std::string sensorName;
	int* rawDataBuffer;

public:
	Sensor(const std::string& name);
	~Sensor();

	Sensor(const Sensor&) = delete;
	Sensor& operator=(const Sensor&) = delete;

	void readData() const;
};
