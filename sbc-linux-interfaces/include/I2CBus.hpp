#pragma once

#include <cstring>
#include <memory>
#include <mutex>
#include <string>

/**
 * A class for interfacing with SBC's I2C bus via i2cdev.
 *
 * This class enables reading and writing to/from a I2C bus of a Single-Board Computer (e.g. Raspberry Pi).
 * The i2cdev device must be available in the filesystem (typically as `/dev/i2c-X`)
 * and the user running the program must have permissions to access it (typically by being in the `i2c` group).
 *
 * Note: the I2CBus::read, I2CBus::readValue, I2CBus::write and I2CBus::writeValue methods
 * take the register address type as a template.
 * This was introduced in order to allow interfacing with devices that are not SMBus-compiant,
 * i.e. have register addresses longer than 8-bytes (e.g. the ST's VL53L1X sensors)
 */
class I2CBus: public std::enable_shared_from_this<I2CBus> {
public:
	/**
	 * A shared_ptr alias (use as I2CBus::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<I2CBus>;

	/**
	 * A shared_ptr to a constant alias (use as I2CBus::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const I2CBus>;

	/**
	 * Initializes a new I2C bus interface.
	 *
	 * May throw if the i2cdev file is inaccessible.
	 *
	 * @param devicePath The path to the i2cdev device, e.g. `"/dev/i2c-1"`
	 */
	explicit I2CBus(const std::string& devicePath);

	~I2CBus();

	/**
	 * Read data from a register of an I2C device.
	 *
	 * @param address I2C address of the device to read from
	 * @param registerAddr The address of the register to read
	 * @param data Pointer to the data receive buffer
	 * @param length Length of the data to receive
	 *
	 * @tparam RegT Register address type (e.g. uint8_t, uint16_t)
	 */
	template<typename RegT>
	void read(uint8_t address, RegT registerAddr, uint8_t* data, uint16_t length) {
		// Prepare the write buffer = the n bytes of the register address
		uint8_t writeBuffer[sizeof(registerAddr)] = {};
		for (ssize_t i = sizeof(registerAddr) - 1; i >= 0; --i) {
			writeBuffer[i] = registerAddr & 0xFF;
			registerAddr = registerAddr >> 8;
		}

		this->transfer(address, writeBuffer, sizeof(registerAddr), data, length);
	}

	/**
	 * Write data to a register of an I2C device.
	 *
	 * @param address I2C address of the device to write to
	 * @param registerAddr The address of the register to write
	 * @param data Pointer to the data to write buffer
	 * @param length Length of the data to write
	 *
	 * @tparam RegT Register address type (e.g. uint8_t, uint16_t)
	 */
	template<typename RegT>
	void write(uint8_t address, RegT registerAddr, const uint8_t* data, uint16_t length) {
		// Prepare the write buffer: first the n bytes of the register address, then data
		uint16_t registerLength = sizeof(registerAddr);
		// Note: can't use registerLength here
		uint8_t writeBuffer[sizeof(registerAddr) + length];
		memset(writeBuffer, 0, sizeof(registerAddr) + length);
		// Fill the n bytes of the register address, MSB first (but beggining at the LSB)
		for (int i = registerLength - 1; i >= 0; --i) {
			writeBuffer[i] = registerAddr & 0xFF;
			registerAddr = registerAddr >> 8;
		}
		// Copy the data to be written
		for (int i = 0; i < length; ++i) {
			writeBuffer[i + registerLength] = data[i];
		}

		// Transfer with no read
		this->transfer(address, writeBuffer, registerLength + length, nullptr, 0);
	}

	/**
	 * Read and return a value from a register.
	 *
	 * A convenience method for @ref I2CBus::read.
	 *
	 * @param address I2C address of the device to read from
	 * @param registerAddr The address of the register to read
	 * @param lsbFirst Whether the data is returned LSB first (default true, n/a for 1-byte data)
	 *
	 * @tparam RegT Register address type (e.g. uint8_t, uint16_t)
	 * @tparam DataT Returned data type (e.g. uint8_t, uint32_t)
	 *
	 * @return The read data, converted into DataT.
	 */
	template<typename RegT, typename DataT>
	DataT readValue(uint8_t address, RegT registerAddr, bool lsbFirst = true) {
		uint8_t data[sizeof(DataT)] = {};

		this->read(address, registerAddr, data, sizeof(DataT));

		uint8_t shift = 0;
		if (lsbFirst) {
			shift = 8 * (sizeof(DataT) - 1);
		}

		DataT retVal = 0;
		for (size_t i = 0; i < sizeof(DataT); ++i) {
			retVal += static_cast<DataT>(data[i]) << shift;
			if (lsbFirst) {
				shift -= 8;
			} else {
				shift += 8;
			}
		}
		return retVal;
	}

	/**
	 * Write a value to a register.
	 *
	 * A convenience method for @ref I2CBus::write.
	 *
	 * @param address I2C address of the device to write to
	 * @param registerAddr The address of the register to write
	 * @param data The data (value) to write
	 * @param lsbFirst Whether the data should be written LSB first (default true, n/a for 1-byte data)
	 *
	 * @tparam RegT Register address type (e.g. uint8_t, uint16_t)
	 * @tparam DataT Written data type (e.g. uint8_t, uint32_t)
	 */
	template<typename RegT, typename DataT>
	void writeValue(uint8_t address, RegT registerAddr, DataT data, bool lsbFirst = true) {
		uint8_t buffer[sizeof(DataT)];

		if (lsbFirst) {
			for (ssize_t i = sizeof(DataT) - 1; i >= 0; --i) {
				buffer[i] = data & 0xFF;
				data = data >> 8;
			}
		} else {
			for (size_t i = 0; i < sizeof(DataT); ++i) {
				buffer[i] = data & 0xFF;
				data = data >> 8;
			}
		}
		this->write(address, registerAddr, buffer, sizeof(DataT));
	}

	/**
	 * Read a 8-bit value from a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint8_t, uint8_t>.
	 */
	template<typename ... Args>
	uint8_t read8(Args&& ... args) {
		return this->readValue<uint8_t, uint8_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Read a 16-bit value from a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint8_t, uint16_t>.
	 */
	template<typename ... Args>
	uint16_t read16(Args&& ... args) {
		return this->readValue<uint8_t, uint16_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Read a 32-bit value from a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint8_t, uint32_t>.
	 */
	template<typename ... Args>
	uint32_t read32(Args&& ... args) {
		return this->readValue<uint8_t, uint32_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Read a 8-bit value from a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint16_t, uint8_t>.
	 */
	template<typename ... Args>
	uint8_t read8Reg16(Args&& ... args) {
		return this->readValue<uint16_t, uint8_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Read a 16-bit value from a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint16_t, uint16_t>.
	 */
	template<typename ... Args>
	uint16_t read16Reg16(Args&& ... args) {
		return this->readValue<uint16_t, uint16_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Read a 32-bit value from a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::readValue<uint16_t, uint32_t>.
	 */
	template<typename ... Args>
	uint32_t read32Reg16(Args&& ... args) {
		return this->readValue<uint16_t, uint32_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 8-bit value to a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint8_t, uint8_t>.
	 */
	template<typename ... Args>
	void write8(Args&& ... args) {
		this->writeValue<uint8_t, uint8_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 16-bit value to a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint8_t, uint16_t>.
	 */
	template<typename ... Args>
	void write16(Args&& ... args) {
		this->writeValue<uint8_t, uint16_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 32-bit value to a 8-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint8_t, uint32_t>.
	 */
	template<typename ... Args>
	void write32(Args&& ... args) {
		this->writeValue<uint8_t, uint32_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 8-bit value to a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint16_t, uint8_t>.
	 */
	template<typename ... Args>
	void write8Reg16(Args&& ... args) {
		this->writeValue<uint16_t, uint8_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 16-bit value to a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint16_t, uint16_t>.
	 */
	template<typename ... Args>
	void write16Reg16(Args&& ... args) {
		this->writeValue<uint16_t, uint16_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Write a 32-bit value to a 16-bit register address.
	 *
	 * A convenience alias for I2CBus::writeValue<uint16_t, uint32_t>.
	 */
	template<typename ... Args>
	void write32Reg16(Args&& ... args) {
		this->writeValue<uint16_t, uint32_t>(std::forward<Args>(args) ...);
	}

	/**
	 * Create a SharedPtr instance of the I2CBus.
	 *
	 * Usage: `I2CBus::makeShared(args...)`.
	 * See constructors (@ref I2CBus::I2CBus) for details.
	 */
	template<typename ... Args>
	static I2CBus::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<I2CBus>(std::forward<Args>(args) ...);
	}

private:
	int fileDescriptor;
	std::mutex lock;

	void transfer(uint8_t address, uint8_t* txBuffer, uint16_t txLength, uint8_t* rxBuffer, uint16_t rxLength);
};
