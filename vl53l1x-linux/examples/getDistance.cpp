#include "VL53L1X.hpp"
#include <I2CBus.hpp>

#include <iostream>
#include <csignal>

static bool exitFlag = false;

void signalHandler(int signalNumber) {
	if (signalNumber == SIGINT) {
		exitFlag = true;
	}
}

int main() {
	auto i2c = I2CBus::makeShared("/dev/i2c-3");
	VL53L1X sensor(i2c);

	std::signal(SIGINT, signalHandler);

	// This MAY throw
	sensor.initialize();

	sensor.startRanging();
	sensor.setDistanceMode(VL53L1X::DISTANCE_MODE_SHORT);
	while (!exitFlag) {
		uint16_t distance = sensor.getDistance();
		std::cout << distance << std::endl;
	}

	sensor.stopRanging();

	return 0;
}
