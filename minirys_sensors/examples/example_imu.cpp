#include "LSM6.hpp"
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
	LSM6 sensor(i2c);

	std::signal(SIGINT, signalHandler);

	std::cout << "[IMU] Initializing the sensor" << std::endl;
	// This MAY throw
	sensor.initialize();

	std::cout << "[IMU] Starting data reading" << std::endl;
	while (!exitFlag) {
		std::cout << "AccX = " << sensor.readFloatAccelX() << "g | ";
		std::cout << "AccY = " << sensor.readFloatAccelY() << "g | ";
		std::cout << "AccZ = " << sensor.readFloatAccelZ() << "g | ";
		std::cout << "GyroX = " << sensor.readFloatGyroX() << "dps | ";
		std::cout << "GyroY = " << sensor.readFloatGyroY() << "dps | ";
		std::cout << "GyroZ = " << sensor.readFloatGyroZ() << "dps | ";
		std::cout << "T = " << sensor.readTempC() << "*C | ";
		std::cout << std::endl;
		std::this_thread::sleep_for(500ms);
	}

	std::cout << "[IMU] Bye!" << std::endl;

	return 0;
}
