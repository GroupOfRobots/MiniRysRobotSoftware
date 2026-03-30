#include "PWMPin.hpp"

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

	// PWMPin::SharedPtr
	auto pwm = PWMPin::makeShared(0, 0);

	pwm->setFrequency(10);
	pwm->setDuty(0.0f);
	pwm->enable();

	float flag = 1;

	while (!exitFlag) {
		pwm->setDuty(0.75f + 0.25f * flag);
		flag *= -1;

		std::this_thread::sleep_for(1s);
	}

	pwm->setPeriod(0);
	pwm->setDuty(0.0f);
	pwm->disable();
	std::cout << "KBye!" << std::endl;

	return 0;
}
