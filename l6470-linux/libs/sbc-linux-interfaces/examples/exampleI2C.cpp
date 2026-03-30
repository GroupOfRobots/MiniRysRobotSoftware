#include "I2CBus.hpp"

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
	std::signal(SIGINT, signalHandler);

	auto i2c = I2CBus("/dev/i2c-3");

	i2c.write16Reg16(0x10, 0x0000, 0xffff);
	i2c.write16Reg16(0x10, 0x0000, 0xffff, false);
	std::cout << "Register 0x00 of device 0x10: " << i2c.read16Reg16(0x10, 0x0000) << std::endl;
	std::cout << "Register 0x00 of device 0x10: " << i2c.read16Reg16(0x10, 0x0000, false) << std::endl;

	return 0;
}
