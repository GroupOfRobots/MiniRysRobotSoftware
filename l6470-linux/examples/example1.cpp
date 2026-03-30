#include "L6470.hpp"
#include <SPIBus.hpp>

#include <chrono>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <thread>

using namespace std::chrono_literals;

static bool exitFlag = false;

void signalHandler(int signalNumber) {
	if (signalNumber == SIGINT) {
		exitFlag = true;
	}
}

int main() {
	std::signal(SIGINT, signalHandler);

	auto spiBus = SPIBus::makeShared("/dev/spidev0.0", SPIBus::SPIBUS_MODE_3, 5000000);
	auto resetPin = GPIOPin::makeShared("/sys/class/gpio/gpio22");

	L6470 driver(2, spiBus, resetPin);

	while (!exitFlag) {
		std::cout << "Loop" << std::endl;
		// Set integer output to hex
		std::cout << std::hex;

		auto statuses = driver.getStatus();
		// get status
		std::cout << "\t status 0: " << statuses[0].rawStatus << std::endl;
		std::cout << "\t status 1: " << statuses[1].rawStatus << std::endl;

		// get config
		auto configs = driver.getParam(L6470::REG_ADDR_CONFIG);
		std::cout << "\t config 0: " << configs[0] << std::endl;
		std::cout << "\t config 1: " << configs[1] << std::endl;

		// get abs_pos
		auto absPositions = driver.getPosition();
		std::cout << "\t abs_pos 0: " << absPositions[0] << std::endl;
		std::cout << "\t abs_pos 1: " << absPositions[1] << std::endl;

		// set and re-get abs_pos
		std::cout << "\t seting abs_pos" << std::endl;
		driver.setPosition(123456, 0);
		driver.setPosition(789012, 1);
		auto absPositions2 = driver.getPosition();
		std::cout << "\t abs_pos 0: " << absPositions2[0] << std::endl;
		std::cout << "\t abs_pos 1: " << absPositions2[1] << std::endl;

		// reset
		std::cout << "\t soft reset" << std::endl;
		driver.resetDevice();

		std::cout << "\t hw reset" << std::endl;
		driver.resetDeviceHw();

		std::this_thread::sleep_for(1s);
	}

	std::cout << "KBye!" << std::endl;

	return 0;
}
