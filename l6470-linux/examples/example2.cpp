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

	driver.resetDevice();
	// Set 62.5kHz PWM frequency
	driver.setOscillatorMode(L6470::OSC_MODE_INT_16MHZ_OSCOUT_16MHZ);
	driver.setPWMFrequency(L6470::PWM_DIV_1, L6470::PWM_MULT_2);
	// Other CONFIG register values
	driver.setSlewRate(L6470::SLEW_RATE_260V_US);
	driver.setOCShutdown(true);
	driver.setVoltageCompensation(false);
	driver.setSwitchMode(L6470::SWITCH_MODE_USER);
	// Setup stepping, speeds etc
	driver.configStepMode(L6470::STEP_MODE_64);
	driver.setMaxSpeed(400);
	driver.setMinSpeed(0);
	driver.setFullSpeed(2000);
	driver.setAcceleration(8000);
	driver.setDeceleration(8000);
	// Current/voltage settings
	driver.setOCThreshold(2000);
	driver.setAccelerationKVAL(0x96);
	driver.setDecelerationKVAL(0x96);
	driver.setRunKVAL(0x96);
	driver.setHoldKVAL(0x32);
	// Disable BEMF compensation and the FLAG (alarm) pin
	driver.setParam(L6470::REG_ADDR_ST_SLP, 0x00);
	driver.setParam(L6470::REG_ADDR_FN_SLP_ACC, 0x00);
	driver.setParam(L6470::REG_ADDR_FN_SLP_DEC, 0x00);
	driver.setParam(L6470::REG_ADDR_ALARM_EN, 0x00);

	bool direction = true;

	auto configs = driver.getParam(L6470::REG_ADDR_CONFIG);
	auto ocThresholds = driver.getOCThreshold();
	std::cout << std::hex << "Config: 0: " << configs[0] << " | 1: " << configs[1] << std::endl;
	std::cout << std::hex << "OC threshold: 0: " << ocThresholds[0] << " | 1: " << ocThresholds[1] << std::endl;
    std::vector<float> speeds = {250.0, 250.0};
    std::vector<L6470::Direction> dirs = {L6470::DIRECTION_FWD, L6470::DIRECTION_REV};
    std::vector<L6470::Direction> dirs2 = {L6470::DIRECTION_REV, L6470::DIRECTION_FWD};

	while (!exitFlag) {
		std::cout << "Loop" << std::endl;
		// Set integer output to hex
		std::cout << std::hex << std::setfill(' ') << std::setw(5);

		auto statuses = driver.getStatus();
		auto positions = driver.getPosition();

		// get status
		std::cout << "\t motor: APos _hiZ busy _dir cmdN cmdW UVlo thwr thsd __oc stlA stlB scka" << std::endl;
		std::cout << "\t     0: "
			<< std::setfill(' ') << std::setw(5) << positions[0]
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].hiZ
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].busy
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].direction
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].lastCmdNotPerformed
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].lastCmdWrong
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].undervoltageLockout
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].thermalWarning
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].thermalShutdown
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].overcurrent
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].stepLossA
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].stepLossB
			<< std::setfill(' ') << std::setw(5) << (int)statuses[0].stepClockActive
			<< std::endl;
		std::cout << "\t     1: "
			<< std::setfill(' ') << std::setw(5) << positions[1]
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].hiZ
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].busy
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].direction
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].lastCmdNotPerformed
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].lastCmdWrong
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].undervoltageLockout
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].thermalWarning
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].thermalShutdown
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].overcurrent
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].stepLossA
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].stepLossB
			<< std::setfill(' ') << std::setw(5) << (int)statuses[1].stepClockActive
			<< std::endl;

		driver.run(speeds,  direction ? dirs : dirs2);

		direction = !direction;
		std::this_thread::sleep_for(10s);
	}

	std::cout << "\t soft reset" << std::endl;
	driver.softStop();
	driver.resetDevice();
	resetPin->unset();

	std::cout << "KBye!" << std::endl;

	return 0;
}
