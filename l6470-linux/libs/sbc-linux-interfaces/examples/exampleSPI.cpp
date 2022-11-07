#include "SPIBus.hpp"
#include <iostream>
#include <vector>

int main() {
	auto spi = SPIBus::makeShared("/dev/spidev0.0");

	uint8_t tx[1] = {0x00};
	uint8_t rx[1] = {};
	spi->transfer(tx, rx, 1);

	std::cout << "Received data: " << (int)(rx[0]) << std::endl;

	std::vector<SPIBus::SPITransfer> transfers;
	transfers.emplace_back(tx, rx, 1);
	spi->transfer(transfers);

	std::cout << "Received data: " << (int)(transfers[0].rxBuffer[0]) << std::endl;

	return 0;
}
