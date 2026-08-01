#include "sensor.hpp"

#include <memory>

int main() {
	auto sensor = std::make_unique<Sensor>("cps_sensor");
	sensor->readData();
	return 0;
}
