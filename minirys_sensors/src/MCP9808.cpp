#include "MCP9808.hpp"
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

MCP9808::MCP9808(I2CBus::SharedPtr i2cBus, uint8_t address):
	i2cBus(std::move(i2cBus)),
	address(address) {}

void MCP9808::initialize() {
	if (this->i2cBus->read16(this->address, RegisterAddresses::MANUFACTURER_ID) != MCP9808::REFERENCE_MANUFACTURER_ID) {
		throw(std::runtime_error("Invalid manufacturer ID received"));
	}
	if (this->i2cBus->read16(this->address, RegisterAddresses::DEVICE_ID) != MCP9808::REFERENCE_DEVICE_ID) {
		throw(std::runtime_error("Invalid device ID received"));
	}

	this->i2cBus->write16(this->address, RegisterAddresses::CONFIG, 0x00);
}

float MCP9808::readTemperature() {
	float temp = NAN;
	uint16_t t = this->i2cBus->read16(this->address, RegisterAddresses::AMBIENT_TEMP);

	if (t != 0xFFFF) {
		temp = static_cast<float>(t & 0x0FFF);
		temp /= 16.0f;
		if (t & 0x1000) {
			temp -= 256.0f;
		}
	}

	return temp;
}

void MCP9808::shutdown() {
	MCP9808Config config {};
	config.rawConfig = this->i2cBus->read16(this->address, RegisterAddresses::CONFIG);
	config.shutdown = 1;
	this->i2cBus->write16(this->address, RegisterAddresses::CONFIG, config.rawConfig);
}

void MCP9808::wakeup() {
	MCP9808Config config {};
	config.rawConfig = this->i2cBus->read16(this->address, RegisterAddresses::CONFIG);
	config.shutdown = 0;
	this->i2cBus->write16(this->address, RegisterAddresses::CONFIG, config.rawConfig);
	std::this_thread::sleep_for(MCP9808::WAKEUP_TIME);
}

uint8_t MCP9808::getResolution() {
	return this->i2cBus->read8(this->address, RegisterAddresses::RESOLUTION);
}

void MCP9808::setResolution(uint8_t value) {
	this->i2cBus->write8(this->address, RegisterAddresses::RESOLUTION, value & MCP9808::RESOLUTION_MASK);
}
