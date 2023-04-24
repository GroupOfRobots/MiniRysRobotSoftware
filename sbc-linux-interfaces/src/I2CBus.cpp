#include "I2CBus.hpp"

// std::memcpy, strerror()
#include <cstring>
// open(), O_RDWR
#include <fcntl.h>
// close()
#include <unistd.h>
// I2C_SLAVE
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
// ioctl()
#include <sys/ioctl.h>

I2CBus::I2CBus(const std::string& devicePath) {
	this->fileDescriptor = open(devicePath.c_str(), O_RDWR);
	if (this->fileDescriptor < 0) {
		throw std::runtime_error(std::string("Error opening i2c device file: ") + strerror(errno));
	}
}

I2CBus::~I2CBus() {
	if (this->fileDescriptor >= 0) {
		close(this->fileDescriptor);
	}
}

void I2CBus::transfer(uint8_t address, uint8_t* txBuffer, uint16_t txLength, uint8_t* rxBuffer, uint16_t rxLength) {
	if (this->fileDescriptor < 0) {
		throw std::runtime_error("I2C device file not opened");
	}

	i2c_msg registerSelectMsgs[1] = {{
		.addr = address,
		.flags = 0,
		.len = txLength,
		.buf = txBuffer
	}};
	i2c_rdwr_ioctl_data registerSelectMsgSet = {
		.msgs = registerSelectMsgs,
		.nmsgs = 1,
	};
	i2c_msg registerReadMsgs[1] = {{
		.addr = address,
		.flags = I2C_M_RD,
		.len = rxLength,
		.buf = rxBuffer,
	}};
	i2c_rdwr_ioctl_data registerReadMsgSet = {
		.msgs = registerReadMsgs,
		.nmsgs = 1,
	};

	std::lock_guard<std::mutex> guard(this->lock);

	int result = ioctl(this->fileDescriptor, I2C_RDWR, &registerSelectMsgSet);
	if (result < 0) {
		throw std::runtime_error("I2C write error: " + std::to_string(result));
	}
	if (rxLength) {
		result = ioctl(this->fileDescriptor, I2C_RDWR, &registerReadMsgSet);
		if (result < 0) {
			throw std::runtime_error("I2C read error: " + std::to_string(result));
		}
	}
}
