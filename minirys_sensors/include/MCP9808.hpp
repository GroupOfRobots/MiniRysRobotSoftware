#pragma once

#include <I2CBus.hpp>

#include <chrono>
#include <memory>

/**
 * MCP9808 temperature sensor interface class.
 *
 * Based upon Adafruit's library and the Microship's documentation for the sensor.
 *
 * Docs: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP9808-0.5C-Maximum-Accuracy-Digital-Temperature-Sensor-Data-Sheet-DS20005095B.pdf
 */
class MCP9808 {
public:
	/**
	 * A shared_ptr alias (use as MCP9808::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<MCP9808>;

	/**
	 * A shared_ptr to a constant alias (use as MCP9808::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const MCP9808>;

	/**
	 * MCP9808 configuration.
	 *
	 * Not used in any public interface yet.
	 */
	union MCP9808Config {
		struct {
			uint8_t alertMode : 1;
			uint8_t alertPolarity : 1;
			uint8_t alertSelect : 1;
			uint8_t alertControl : 1;
			uint8_t alertStatus : 1;
			uint8_t interruptClear : 1;
			uint8_t alarmWindowLock : 1;
			uint8_t criticalTripLock : 1;
			uint8_t shutdown : 1;
			uint8_t unused : 7;
		}

		__attribute__((packed));

		uint16_t rawConfig;
	};

	/**
	 * Create a new MCP9808 intance.
	 *
	 * @param[in] i2cBus The I2C bus to use
	 * @param[in] address The device's I2C address
	 */
	explicit MCP9808(I2CBus::SharedPtr i2cBus, uint8_t address = MCP9808::DEFAULT_DEVICE_ADDRESS);

	/**
	 * Initialize the sensor.
	 *
	 * This method verifies that a sensor is available on the bus under the specified address
	 * and resets the sensor's config (enabling the sensor).
	 */
	void initialize();

	/**
	 * Shut down the sensor.
	 *
	 * This method sets the 'shutdown' configuration bit of the sensor to 1.
	 */
	void shutdown();

	/**
	 * Wake up the sensor.
	 *
	 * This method sets the 'shutdown' configuration bit of the sensor to 0,
	 * then waits (in a locking manner) for the sensor to boot up.
	 */
	void wakeup();

	/**
	 * Read the 16-bit temperature register and return the Centigrade temperature as a float.
	 *
	 * @return Temperature in Centigrade.
	 */
	float readTemperature();

	/**
	 * Get the sensor's resolution.
	 *
	 * @return The resolution.
	 */
	uint8_t getResolution();

	/**
	 * Set the sensor's resolution.
	 *
	 * @param value The resolution to set.
	 */
	void setResolution(uint8_t value);

	/**
	 * Create a SharedPtr instance of the MCP9808.
	 *
	 * Usage: `MCP9808::makeShared(args...)`.
	 * See constructor (@ref MCP9808::MCP9808()) for details.
	 */
	template<typename ... Args>
	static MCP9808::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<MCP9808>(std::forward<Args>(args) ...);
	}

private:
	static constexpr uint8_t DEFAULT_DEVICE_ADDRESS = 0x18;
	static constexpr uint16_t REFERENCE_MANUFACTURER_ID = 0x0054;
	static constexpr uint16_t REFERENCE_DEVICE_ID = 0x0400;
	static constexpr auto WAKEUP_TIME = std::chrono::milliseconds(250);
	static constexpr uint8_t RESOLUTION_MASK = 0x03;

	enum RegisterAddresses : uint8_t {
		CONFIG = 0x01,
		ALERT_UPPER_TEMP = 0x02,
		ALERT_LOWER_TEMP = 0x03,
		CRITICAL_TEMP = 0x04,
		AMBIENT_TEMP = 0x05,
		MANUFACTURER_ID = 0x06,
		DEVICE_ID = 0x07,
		RESOLUTION = 0x08,
	};

	I2CBus::SharedPtr i2cBus;
	uint8_t address;
};
