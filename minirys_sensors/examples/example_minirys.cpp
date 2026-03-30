#include "MAX1161.hpp"
#include "LSM6.hpp"
#include "MCP9808.hpp"
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
	MAX1161 adc(i2c, MAX1161::MAX11613);
	LSM6 imu(i2c);
	MCP9808 temp(i2c);

	std::signal(SIGINT, signalHandler);

	std::cout << "[MiniRys sensors] Initializing the sensors" << std::endl;
	// This MAY throw
	adc.initialize();
	imu.initialize();
	temp.initialize();

	std::cout << "[MiniRys sensors] Waking up the sensors" << std::endl;
	temp.wakeup();

	while (!exitFlag) {
		std::cout << "[MiniRys]" << std::endl;

		std::cout << "\tBatt: Cell0 = " << adc.readChannel(0) * 2.5 << "V";
		std::cout << " | Cell1 = " << adc.readChannel(1) * 4.6 << "V";
		std::cout << " | Cell2 = " << adc.readChannel(2) * 6.6 << "V";
		std::cout << std::endl;

		std::cout << "\tTemp = " << temp.readTemperature() << "*C" << std::endl;

		std::cout << "\tIMU: AccX = " << imu.readFloatAccelX() << "g | ";
		std::cout << "AccY = " << imu.readFloatAccelY() << "g | ";
		std::cout << "AccZ = " << imu.readFloatAccelZ() << "g | ";
		std::cout << "GyroX = " << imu.readFloatGyroX() << "dps | ";
		std::cout << "GyroY = " << imu.readFloatGyroY() << "dps | ";
		std::cout << "GyroZ = " << imu.readFloatGyroZ() << "dps | ";
		std::cout << std::endl;

		std::this_thread::sleep_for(500ms);
	}

	std::cout << "[ADC] Kbye!" << std::endl;
	temp.shutdown();

	return 0;
}
