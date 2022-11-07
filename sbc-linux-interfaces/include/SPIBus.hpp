#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

// SPIDEV defines (SPI_MODE_*, SPI_IOC_WR_* etc)
// Note: include as C for proper linking, see:
// https://stackoverflow.com/questions/31572738/spi-ioc-messagen-macro-giving-me-fits
extern "C" {
	#include <linux/spi/spidev.h>
}

/**
 * A class for interfacing with SBC's SPI bus via spidev.
 *
 * This class enables reading, writing and transferring (combined write-read)
 * to/from a SPI bus of a Single-Board Computer (e.g. Raspberry Pi).
 * The spidev device must be available in the filesystem (typically as `/dev/spidevX.Y`)
 * and the user running the program must have permissions to access it (typically by being in the `spi` group).
 */
class SPIBus: public std::enable_shared_from_this<SPIBus> {
public:
	/**
	 * A shared_ptr alias (use as SPIBus::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<SPIBus>;

	/**
	 * A shared_ptr to a constant alias (use as SPIBus::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const SPIBus>;

	/**
	 * SPI modes.
	 *
	 * An alias for the kernel-defined values.
	 * See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Mode_numbers for an explanation.
	 */
	enum SPIMode : uint8_t {
		SPIBUS_MODE_0 = SPI_MODE_0,
		SPIBUS_MODE_1 = SPI_MODE_1,
		SPIBUS_MODE_2 = SPI_MODE_2,
		SPIBUS_MODE_3 = SPI_MODE_3,
	};

	/**
	 * A single SPI transfer (combined write-read) structure.
	 */
	struct SPITransfer {
		uint8_t* txBuffer;
		uint8_t* rxBuffer;
		uint32_t length;

		SPITransfer(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length):
			txBuffer(txBuffer),
			rxBuffer(rxBuffer),
			length(length) {}
	};

	/**
	 * Initializes and configures a new SPI bus instance.
	 *
	 * May throw if the spidev file is inaccessible or if setting the configuration fails.
	 *
	 * @param devicePath The path to the spidev device, e.g. `"/dev/spidev0.0"`
	 * @param mode The SPI mode to set
	 * @param speed The speed to set, in bits per second (e.g. 5000000)
	 * @param lsbFirst Whether the interface should send/receive the least significant bytes first.
	 */
	explicit SPIBus(const std::string& devicePath, SPIMode mode, uint32_t speed, bool lsbFirst = false);

	~SPIBus();

	/**
	 * A helper method for checking if the interface is still ok.
	 *
	 * @return True if the interface was configured successfully and is still open.
	 */
	bool ok() const;

	/**
	 * Read from the SPI bus.
	 *
	 * Equivalent to `transfer(SPITransfer(nullptr, rxBuffer, length))`.
	 *
	 * @param rxBuffer The receive buffer
	 * @param length The length of the data to receive
	 */
	void read(uint8_t* rxBuffer, uint32_t length);

	/**
	 * Write to the SPI bus.
	 *
	 * Equivalent to `transfer(SPITransfer(txBuffer, nullptr, length))`.
	 *
	 * @param txBuffer The write buffer
	 * @param length The length of the data to receive
	 */
	void write(uint8_t* txBuffer, uint32_t length);

	/**
	 * Perform a transfer (combo write-read) on the SPI bus.
	 *
	 * Equivalent to `transfer(SPITransfer(txBuffer, rxBuffer, length))`.
	 *
	 * @param txBuffer The write buffer
	 * @param rxBuffer The receive buffer
	 * @param length The length of the data to send/receive
	 */
	void transfer(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t length);

	/**
	 * Perform a transfer (combo write-read) on the SPI bus.
	 *
	 * @param transfer The transfer to perform.
	 */
	void transfer(SPITransfer transfer);

	/**
	 * Perform multiple transfers back-to-back on the SPI bus.
	 *
	 * @param transfers The transfers to perform.
	 */
	void transfer(const std::vector<SPITransfer>& transfers);

	/**
	 * Create a SharedPtr instance of the SPIBus.
	 *
	 * Usage: `SPIBus::makeShared(args...)`.
	 * See constructors (@ref SPIBus::SPIBus) for details.
	 */
	template<typename ... Args>
	static SPIBus::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<SPIBus>(std::forward<Args>(args) ...);
	}

private:
	int fileDescriptor;
	std::mutex lock;
	uint32_t speed;
};
