#pragma once

#include <memory>
#include <mutex>
#include <string>

/**
 * A class for interfacing with SBC's GPIO pins.
 *
 * This class enables reading and writing values to/from a GPIO pins of a Single-Board Computer (e.g. Raspberry Pi).
 * The pin must be available as a file in the filesystem (typically as `/sys/class/gpio/gpioX/value`)
 * and the user running the program must have permissions to access it (typically by being in the `gpio` group).
 *
 * To make a pin available, one typically has to export it, e.g. via:
 * @code echo <pin_number> > /sys/class/gpio/export` @endcode,
 * then setup the direction via:
 * @code echo <in|out> > /sys/class/gpio/gpio<number>/direction` @endcode.
 *
 * The methods of this class may throw an exception if the file is unaccessible or if a read/write operation fails.
 */
class GPIOPin: public std::enable_shared_from_this<GPIOPin> {
public:
	/**
	 * A shared_ptr alias (use as GPIOPin::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<GPIOPin>;
	/**
	 * A shared_ptr to a constant alias (use as GPIOPin::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const GPIOPin>;

	/**
	 * Constructs a new instance for managing a GPIO pin.
	 *
	 * @param gpioDirectory The path to the GPIO pin directory, e.g. `"/sys/class/gpio/gpio22"`
	 */
	explicit GPIOPin(std::string gpioDirectory);

	/**
	 * Export a GPIO pin (make it available in the system) and constructs a new management instance.
	 *
	 * @param gpioNumber The number of the GPIO to export
	 * @param output Whether the exported pin should be output or input
	 * @param gpioBaseDirectory The GPIO base directory, e.g. "/sys/class/gpio" for RPi-s
	 */
	explicit GPIOPin(uint8_t gpioNumber, bool output = true, const std::string& gpioBaseDirectory = "/sys/class/gpio");

	/**
	 * Set the direction of the GPIO pin.
	 *
	 * @param output Whether to set the pin as output (input if false)
	 */
	void setDirection(bool output);

	/**
	 * Read the current GPIO pin value.
	 *
	 * Available for both input and output GPIOs.
	 *
	 * @return True if pin is high ("1")
	 */
	bool read();

	/**
	 * Set the GPIO pin (set it high).
	 *
	 * Available only for output GPIOs.
	 */
	void set();

	/**
	 * Unset the GPIO pin (set it low).
	 *
	 * Available only for output GPIOs.
	 */
	void unset();

	/**
	 * Toggle the GPIO pin (set it low if it was high and vice versa).
	 *
	 * Available only for output GPIOs.
	 */
	void toggle();

	/**
	 * Create a SharedPtr instance of the GPIOPin.
	 *
	 * Usage: `GPIOPin::makeShared(args...)`.
	 * See constructors (@ref GPIOPin::GPIOPin) for details.
	 */
	template<typename ... Args>
	static GPIOPin::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<GPIOPin>(std::forward<Args>(args) ...);
	}

private:
	const std::string gpioDirectory;

	std::mutex gpioMutex;

	void write(const char& value);
};
