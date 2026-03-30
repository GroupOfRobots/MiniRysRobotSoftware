#pragma once

#include <I2CBus.hpp>

#include <memory>

/**
 * MAX11612-MAX11617 ADC interface class.
 *
 * Based upon the Maxim Integrated's documentation for the MAX1161x series.
 *
 * Docs: https://datasheets.maximintegrated.com/en/ds/MAX11612-MAX11617.pdf
 */
class MAX1161 {
public:
	/**
	 * A shared_ptr alias (use as MAX1161::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<MAX1161>;

	/**
	 * A shared_ptr to a constant alias (use as MAX1161::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const MAX1161>;

	/**
	 * Available types of the MAX1161*, passed to the ctor (MAX1161::MAX1161()).
	 *
	 * The selected type determines the default address and the reference voltage (VRef).
	 */
	enum MAXType {
		MAX11612 = 0,
		MAX11613,
		MAX11614,
		MAX11615,
		MAX11616,
		MAX11617,
	};

	/**
	 * Create a new MAX1161x ADC instance.
	 *
	 * @param i2cBus The I2C bus to use
	 * @param type The type of the MAX to handle
	 */
	explicit MAX1161(I2CBus::SharedPtr i2cBus, MAXType type);

	/**
	 * Initialize the ADC.
	 *
	 * This method setups the ADC by sending a SETUP byte with the following settings:
	 * - Unipolar measurement
	 * - Internal clock
	 * - Internal reference voltage, AIN_/REF as analog input, REF not connected
	 */
	void initialize();

	/**
	 * Read a channel as a raw value, as sent by the ADC.
	 *
	 * @param channel The channel to read
	 * @param differential Read type - differential (true) or single-ended (false, default)
	 *
	 * @return The raw ADC value
	 */
	uint16_t readChannelRaw(int channel, bool differential = false);

	/**
	 * Read a channel as voltage.
	 *
	 * @param channel The channel to read
	 * @param differential Read type - differential (true) or single-ended (false, default)
	 *
	 * @return The ADC value as voltage (in Volts)
	 */
	float readChannel(int channel, bool differential = false);

	/**
	 * Create a SharedPtr instance of the MAX1161.
	 *
	 * Usage: `MAX1161::makeShared(args...)`.
	 * See constructor (@ref MAX1161::MAX1161()) for details.
	 */
	template<typename ... Args>
	static MAX1161::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<MAX1161>(std::forward<Args>(args) ...);
	}

private:
	/**
	 * The default device I2C addresses.
	 */
	static constexpr uint8_t DEFAULT_MAX11612_13_ADDRESS = 0x34;
	static constexpr uint8_t DEFAULT_MAX11614_15_ADDRESS = 0x33;
	static constexpr uint8_t DEFAULT_MAX11616_17_ADDRESS = 0x35;
	static constexpr uint8_t SCAN_MODE_SINGLE = 0x03;
	static constexpr uint8_t VREF_INTERNAL_AIN_ON = 0x05;
	static constexpr float VREF_MAX11612_14_16 = 4.096f;
	static constexpr float VREF_MAX11613_15_17 = 2.048f;
	static constexpr int READ_BITS = 12;
	static constexpr uint16_t READ_MASK = 0X0FFF;

	union SetupByte {
		struct {
			uint8_t dontCare : 1;
			uint8_t reset : 1;
			uint8_t polarity : 1;
			uint8_t clock : 1;
			uint8_t reference : 3;
			uint8_t registerBit : 1;
		}

		__attribute__((packed));

		uint8_t data;
	};

	union ConfigByte {
		struct {
			uint8_t singleEnded : 1;
			uint8_t channel : 4;
			uint8_t scanMode : 2;
			uint8_t registerBit : 1;
		}

		__attribute__((packed));

		uint8_t data;
	};

	I2CBus::SharedPtr i2cBus;
	uint8_t address;
	float vRef;
	bool initialized;
};
