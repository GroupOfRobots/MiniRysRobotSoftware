#include "GPIOPin.hpp"

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

	// Static
	auto gpio = GPIOPin("/sys/class/gpio/gpio23");
	// SharedPtr
	auto gpio2 = GPIOPin::makeShared(24);

	while (!exitFlag) {
		gpio.toggle();
		gpio2->toggle();

		std::this_thread::sleep_for(1s);
	}

	std::cout << "KBye!" << std::endl;

	return 0;
}
