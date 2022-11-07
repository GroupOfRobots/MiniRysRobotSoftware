#include "MAX1161.hpp"
#include <I2CBus.hpp>

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
	MAX1161 sensor(i2c, MAX1161::MAX11613);

	std::signal(SIGINT, signalHandler);

	std::cout << "[ADC] Initializing the sensor" << std::endl;
	// This MAY throw
	sensor.initialize();

	while (!exitFlag) {
		std::cout << "U0 = " << sensor.readChannel(0) << "V";
		std::cout << " | U1 = " << sensor.readChannel(1) << "V";
		std::cout << " | U2 = " << sensor.readChannel(2) << "V";
		std::cout << std::endl;
		std::this_thread::sleep_for(500ms);
	}

	std::cout << "[ADC] Kbye!" << std::endl;

	return 0;
}
