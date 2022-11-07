#include "PWMPin.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <thread>

using namespace std::chrono_literals;

PWMPin::PWMPin(std::string pinDirectory):
	pinDirectory(std::move(pinDirectory)) {}

PWMPin::PWMPin(uint8_t chipNumber, uint8_t pwmNumber, const std::string& pwmBaseDirectory):
	pinDirectory(pwmBaseDirectory + "/pwmchip" + std::to_string(chipNumber) + "/pwm" + std::to_string(pwmNumber)) {
	std::string exportFilePath = pwmBaseDirectory + "/pwmchip" + std::to_string(chipNumber) + "/export";
	std::ofstream exportFile;
	exportFile.open(exportFilePath.c_str(), std::ofstream::out);
	if (!exportFile.is_open() || !exportFile.good()) {
		exportFile.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ exportFilePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	exportFile << std::to_string(pwmNumber).c_str();
	exportFile.close();
	std::this_thread::sleep_for(1ms);
}

void PWMPin::enable() {
	this->write(1, "enable");
}

void PWMPin::disable() {
	this->write(0, "enable");
}

void PWMPin::setFrequency(float frequency) {
	// Save current duty cycle
	auto duty = this->getDuty();
	// Temporarily set duty to 0 (to avoid setting lower period than duty cycle)
	this->setDuty(0.0f);
	// Write new period/frequency
	this->write(PWMPin::SEC_TO_NANO / frequency, "period");
	// Restore the previous duty cycle
	this->setDuty(duty);
}

void PWMPin::setPeriod(uint32_t period) {
	// The same operation as in setFrequency()
	auto duty = this->getDuty();
	this->setDuty(0.0f);
	this->write(period, "period");
	this->setDuty(duty);
}

void PWMPin::setDuty(float duty) {
	uint32_t dutyRaw = this->getPeriod() * std::min(std::max(0.0f, duty), 1.0f);
	this->write(dutyRaw, "duty_cycle");
}

uint32_t PWMPin::getPeriod() {
	return this->readIntFile("period");
}

float PWMPin::getFrequency() {
	return static_cast<float>(PWMPin::SEC_TO_NANO) / static_cast<float>(this->getPeriod());
}

float PWMPin::getDuty() {
	uint32_t duty = this->readIntFile("duty_cycle");
	uint32_t period = this->readIntFile("period");
	return static_cast<float>(duty) / static_cast<float>(period);
}

void PWMPin::write(uint32_t value, const std::string& fileName) {
	std::lock_guard<std::mutex> guard(this->mutex);

	auto filePath = this->pinDirectory + "/" + fileName;
	std::ofstream file;
	file.open(filePath.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ filePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	file << value;
	file.close();
}

uint32_t PWMPin::readIntFile(const std::string& fileName) {
	std::lock_guard<std::mutex> guard(this->mutex);

	auto filePath = this->pinDirectory + "/" + fileName;
	std::ifstream file;
	file.open(filePath.c_str(), std::ofstream::in);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ filePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	std::string fileValue;
	file >> fileValue;
	file.close();

	return std::stoi(fileValue);
}
