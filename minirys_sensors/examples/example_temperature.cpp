#include "I2CBus.hpp"
#include "MCP9808.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

static bool exitFlag = false;

void signalHandler(int signalNumber) {
	if (signalNumber == SIGINT) {
		exitFlag = true;
	}
}

int main() {
	auto i2c = I2CBus::makeShared("/dev/i2c-1");
	MCP9808 sensor(i2c);

	std::signal(SIGINT, signalHandler);

	std::cout << "[TEMP] Initializing the sensor" << std::endl;
	// This MAY throw
	sensor.initialize();
	std::cout << "[TEMP] Waking up the sensor" << std::endl;
	sensor.wakeup();

	while (!exitFlag) {
		std::cout << "T = " << sensor.readTemperature() << "*C" << std::endl;
		std::this_thread::sleep_for(500ms);
	}

	std::cout << "[TEMP] Shutting down the sensor, bye!" << std::endl;
	sensor.shutdown();

	return 0;
}
