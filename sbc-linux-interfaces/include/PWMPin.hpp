#pragma once

#include <memory>
#include <mutex>
#include <string>

/**
 * A class for interfacing with SBC's PWM pins (via the Sysfs interface).
 *
 * This class enables managing PWM pins of a Single-Board Computer (e.g. Raspberry Pi).
 * The pin must be available as a file in the filesystem (typically as a `/sys/class/pwm/pwmchipY/pwmX` directory)
 * and the user running the program must have permissions to access it (typically by being in the `gpio` group).
 *
 * To make a pin available, one typically has to export it, e.g. via:
 * @code echo <pin_number> > /sys/class/pwm/pwmchip0` @endcode.
 * Alternatively, this class provides a constructor that tries to export the pin.
 *
 * The methods of this class may throw an exception if the file is unaccessible or if a read/write operation fails.
 */
class PWMPin: public std::enable_shared_from_this<PWMPin> {
public:
	/**
	 * A shared_ptr alias (use as PWMPin::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<PWMPin>;
	/**
	 * A shared_ptr to a constant alias (use as PWMPin::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const PWMPin>;

	/**
	 * Construct a new instance for managing a PWM pin.
	 *
	 * @param pinDirectory The path to the PWM pin directory, e.g. `"/sys/class/pwm/pwmchip0/pwm0"`
	 */
	explicit PWMPin(std::string pinDirectory);

	/**
	 * Export a PWM pin (make it available in the system) and construct a new management instance.
	 *
	 * @param chipNumber The number of the PWM chip to use
	 * @param pwmNumber The number of the PWM pin to export
	 * @param pwmBaseDirectory The PWM base directory, typically "/sys/class/pwm" for Sysfs PWM drivers
	 */
	explicit PWMPin(uint8_t chipNumber, uint8_t pwmNumber, const std::string& pwmBaseDirectory = "/sys/class/pwm");

	/**
	 * Enable the PWM pin output.
	 */
	void enable();

	/**
	 * Disable the PWM pin output.
	 */
	void disable();

	/**
	 * Set the PWM pin's output frequency (1/period).
	 *
	 * @param frequency The frequency to set, in Hz
	 */
	void setFrequency(float frequency);

	/**
	 * Set the PWM pin's output period (1/frequency).
	 *
	 * @param period The period to set, in nanoseconds
	 */
	void setPeriod(uint32_t period);

	/**
	 * Set the PWM pin's duty cycle (fill).
	 *
	 * @param duty The duty cycle to set (cropped to [0.0 - 1.0] range)
	 */
	void setDuty(float duty);

	/**
	 * Retrieve the currently set PWM period (1/frequency).
	 *
	 * @returns The currently set period, in nanoseconds
	 */
	uint32_t getPeriod();

	/**
	 * Retrieve the currently set PWM frequency (1/period).
	 *
	 * @returns The currently set frequency, in Hz
	 */
	float getFrequency();

	/**
	 * Retrieve the currently set duty cycle (fill).
	 *
	 * @returns The currently set duty cycle, should be in the 0.0-1.0 range
	 */
	float getDuty();

	/**
	 * Create a SharedPtr instance of the PWMPin.
	 *
	 * Usage: `PWMPin::makeShared(args...)`.
	 * See constructors (@ref PWMPin::PWMPin) for details.
	 */
	template<typename ... Args>
	static PWMPin::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<PWMPin>(std::forward<Args>(args) ...);
	}

private:
	static constexpr uint32_t SEC_TO_NANO = 1000000000;

	const std::string pinDirectory;

	std::mutex mutex;

	void write(uint32_t value, const std::string& fileName);

	uint32_t readIntFile(const std::string& fileName);
};
