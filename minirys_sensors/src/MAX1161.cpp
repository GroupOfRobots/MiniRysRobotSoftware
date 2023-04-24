#include "MAX1161.hpp"

#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

MAX1161::MAX1161(I2CBus::SharedPtr i2cBus, MAXType type):
	i2cBus(std::move(i2cBus)),
	initialized(false) {
	// Get the address based on the type
	switch (type) {
		case MAX11612:
		case MAX11613:
			this->address = MAX1161::DEFAULT_MAX11612_13_ADDRESS;
			break;
		case MAX11614:
		case MAX11615:
			this->address = MAX1161::DEFAULT_MAX11614_15_ADDRESS;
			break;
		case MAX11616:
		case MAX11617:
			this->address = MAX1161::DEFAULT_MAX11616_17_ADDRESS;
			break;
		default:
			throw std::runtime_error("Invalid MAX type");
	}
	// Get the internal voltage reference based on the type
	switch (type) {
		case MAX11612:
		case MAX11614:
		case MAX11616:
			this->vRef = MAX1161::VREF_MAX11612_14_16;
			break;
		case MAX11613:
		case MAX11615:
		case MAX11617:
			this->vRef = MAX1161::VREF_MAX11613_15_17;
			break;
		default:
			throw std::runtime_error("Invalid MAX type");
	}
}

void MAX1161::initialize() {
	SetupByte setup {};
	// Don't reset
	setup.reset = 0x1;
	// Unipolar (hardcoded for now)
	setup.polarity = 0x0;
	// Internal clock (hardcoded for now)
	setup.clock = 0x0;
	// Reference voltage: internal, AIN_/REF as analog input, REF not connected, internal reference always on (hardcoded for now)
	setup.reference = MAX1161::VREF_INTERNAL_AIN_ON;
	// Register bit: this is a setup byte
	setup.registerBit = 0x1;

	// Note: MAX1161* doesn't use typical i2c registers - instead it just waits for a byte.
	this->i2cBus->write(this->address, setup.data, nullptr, 0);

	// Sleep for 10ms (internal reference power-up, see datasheet section "Reference Voltage: Internal Reference", page 19)
	std::this_thread::sleep_for(10ms);

	this->initialized = true;
}

uint16_t MAX1161::readChannelRaw(int channel, bool differential) {
	ConfigByte config {};
	// Read type
	config.singleEnded = !differential;
	// Channel
	config.channel = channel;
	// Single read (hardcoded for now)
	config.scanMode = MAX1161::SCAN_MODE_SINGLE;
	// Register bit: this is a configuration byte
	config.registerBit = 0x0;

	return this->i2cBus->read16(this->address, config.data, true) & MAX1161::READ_MASK;
}

float MAX1161::readChannel(int channel, bool differential) {
	uint16_t rawValue = this->readChannelRaw(channel, differential);
	return this->vRef * static_cast<float>(std::ldexp(rawValue, -MAX1161::READ_BITS));
}
