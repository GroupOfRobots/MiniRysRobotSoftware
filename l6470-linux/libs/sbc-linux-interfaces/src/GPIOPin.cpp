#include "GPIOPin.hpp"

#include <cstring>
#include <fstream>

GPIOPin::GPIOPin(std::string gpioDirectory):
	gpioDirectory(std::move(gpioDirectory)) {}

GPIOPin::GPIOPin(uint8_t gpioNumber, bool output, const std::string& gpioBaseDirectory):
	gpioDirectory(gpioBaseDirectory + "/gpio" + std::to_string(gpioNumber)) {
	std::string exportFilePath = gpioBaseDirectory + "/export";
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
	exportFile << std::to_string(gpioNumber).c_str();
	exportFile.close();

	this->setDirection(output);
}

void GPIOPin::setDirection(bool output) {
	std::string directionFilePath = this->gpioDirectory + "/direction";
	std::ofstream directionFile;
	directionFile.open(directionFilePath.c_str(), std::ofstream::out);
	if (!directionFile.is_open() || !directionFile.good()) {
		directionFile.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ directionFilePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	if (output) {
		directionFile << "out";
	} else {
		directionFile << "in";
	}
	directionFile.close();
}

bool GPIOPin::read() {
	std::lock_guard<std::mutex> guard(this->gpioMutex);

	auto valuePath = this->gpioDirectory + "/value";
	std::ifstream file;
	file.open(valuePath.c_str(), std::ofstream::in);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ valuePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	std::string fileValue;
	file >> fileValue;
	file.close();

	return fileValue == "1";
}

void GPIOPin::set() {
	this->write('1');
}

void GPIOPin::unset() {
	this->write('0');
}

void GPIOPin::toggle() {
	if (this->read()) {
		this->unset();
	} else {
		this->set();
	}
}

void GPIOPin::write(const char& value) {
	std::lock_guard<std::mutex> guard(this->gpioMutex);

	auto valuePath = this->gpioDirectory + "/value";
	std::ofstream file;
	file.open(valuePath.c_str(), std::ofstream::out);
	if (!file.is_open() || !file.good()) {
		file.close();
		throw(std::runtime_error(
			std::string("Failed opening file: ")
			+ valuePath
			+ ", reason: "
			+ std::strerror(errno)
		));
	}
	file << value;
	file.close();
}
