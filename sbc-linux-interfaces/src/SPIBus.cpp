#include "SPIBus.hpp"

// std::memcpy, strerror()
#include <cstring>
// open(), O_RDWR
#include <fcntl.h>
// close()
#include <unistd.h>
// ioctl()
#include <sys/ioctl.h>

SPIBus::SPIBus(
	const std::string& devicePath,
	SPIMode mode,
	uint32_t speed,
	bool lsbFirst
):
	speed(speed) {
	// Try to open the device
	this->fileDescriptor = open(devicePath.c_str(), O_RDWR);
	if (this->fileDescriptor < 0) {
		throw std::runtime_error(std::string("Error opening SPI device file: ") + strerror(errno));
	}

	uint8_t spiMode = (uint8_t)mode | (lsbFirst * SPI_LSB_FIRST);
	if (ioctl(this->fileDescriptor, SPI_IOC_WR_MODE, &spiMode) < 0) {
		close(this->fileDescriptor);
		throw std::runtime_error(std::string("Error setting SPI mode: ") + strerror(errno));
	}

	uint8_t spiBits = 8;
	if (ioctl(this->fileDescriptor, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0) {
		close(this->fileDescriptor);
		throw std::runtime_error(std::string("Error setting SPI bits per word: ") + strerror(errno));
	}

	if (ioctl(this->fileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		close(this->fileDescriptor);
		throw std::runtime_error(std::string("Error setting SPI speed: ") + strerror(errno));
	}
}

SPIBus::~SPIBus() {
	if (this->ok()) {
		close(this->fileDescriptor);
	}
}

bool SPIBus::ok() const {
	return this->fileDescriptor >= 0;
}

void SPIBus::read(uint8_t* rxBuffer, uint32_t length) {
	this->transfer(nullptr, rxBuffer, length);
}

void SPIBus::write(uint8_t* txBuffer, uint32_t length) {
	this->transfer(txBuffer, nullptr, length);
}

void SPIBus::transfer(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length) {
	this->transfer(SPITransfer(txBuffer, rxBuffer, length));
}

void SPIBus::transfer(SPITransfer transfer) {
	this->transfer(std::vector<SPITransfer>(1, transfer));
}

// This method shouldn't be marked const as it writes to the SPI via ioctl()
// NOLINTNEXTLINE readability-make-member-function-const
void SPIBus::transfer(const std::vector<SPITransfer>& transfersData) {
	if (!this->ok()) {
		throw std::runtime_error(std::string("SPI bus was not initialized successfully"));
	}

	std::vector<spi_ioc_transfer> rawTransfers;
	for (const auto& transferData: transfersData) {
		spi_ioc_transfer rawTransfer {};
		// NOLINTNEXTLINE cppcoreguidelines-pro-type-reinterpret-cast
		rawTransfer.tx_buf = reinterpret_cast<uint64_t>(transferData.txBuffer);
		// NOLINTNEXTLINE cppcoreguidelines-pro-type-reinterpret-cast
		rawTransfer.rx_buf = reinterpret_cast<uint64_t>(transferData.rxBuffer);
		rawTransfer.len = transferData.length;
		rawTransfer.speed_hz = this->speed;
		rawTransfer.delay_usecs = 0;
		rawTransfer.bits_per_word = 8;
		rawTransfer.cs_change = 0;
		rawTransfers.emplace_back(rawTransfer);
	}

	// Note: this ioctl returns the size of the returned message
	ioctl(this->fileDescriptor, SPI_IOC_MESSAGE(rawTransfers.size()), rawTransfers.data());
	if (errno == EINTR) {
		// The call was interrupted, can't help with that.
		return;
	}
	if (errno) {
		throw std::runtime_error(std::string("Error transferring via SPI: ") + strerror(errno));
	}
}
