#pragma once

#include "LSM6_defs.hpp"
#include <I2CBus.hpp>

#include <memory>

/**
 * LSM6 (~DS3, ~DS33) interface class.
 *
 * Based upon SparkFun Electronic's library and ST's documentation for the LSM6DS3 model.
 * Only handles the basic stuff - no sensor hub etc is supported
 * (though e.g. using the pedometer or interrupts should be possible).
 *
 * Note: I have no idea whether there's a difference between the LSM6DS3 and the LSM6DS33 besides their package.
 *
 * Docs:
 * - LSM6DS3: https://www.st.com/resource/en/datasheet/lsm6ds3.pdf
 * - LSM6DS33: https://www.st.com/resource/en/datasheet/lsm6ds33.pdf
 */
class LSM6 {
public:
	/**
	 * A shared_ptr alias (use as LSM6::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<LSM6>;

	/**
	 * A shared_ptr to a constant alias (use as LSM6::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const LSM6>;

	/**
	 * Constructs a new instance of the sensor.
	 *
	 * It generates a default set of SensorSettings - if desired, over-ride after construction by passing to @ref updateSettings() before calling @ref initialize().
	 *
	 * @param i2cBus The I2C bus to use
	 * @param address The device's I2C address
	 */
	explicit LSM6(I2CBus::SharedPtr i2cBus, uint8_t address = LSM6::DEFAULT_DEVICE_ADDRESS);

	/**
	 * Initializes the sensor - checks, whether it's reachable on the bus and writes the currently saved settings (@ref writeSettings()).
	 *
	 * To change the sensor's default settings, use @ref getSettings(), adjust the received object and save it with @ref updateSettings().
	 */
	void initialize();

	/**
	 * Update the saved sensor's settings.
	 *
	 * @param newSettings The new settings
	 */
	void updateSettings(const LSM6_defs::SensorSettings& newSettings);

	/**
	 * Get a copy of currently saved settings.
	 *
	 * @return The currently saved sensor's settings.
	 */
	LSM6_defs::SensorSettings getSettings();

	/**
	 * Write the saved settings to the physical sensor.
	 */
	void writeSettings();

	/**
	 * Returns the raw acceleration in the X axis as a 16-bit signed integer.
	 *
	 * @return Raw X-axis acceleration
	 */
	int16_t readRawAccelX();

	/**
	 * Returns the raw acceleration in the Y axis as a 16-bit signed integer.
	 *
	 * @return Raw Y-axis acceleration
	 */
	int16_t readRawAccelY();

	/**
	 * Returns the raw acceleration in the Z axis as a 16-bit signed integer.
	 *
	 * @return Raw Z-axis acceleration
	 */
	int16_t readRawAccelZ();

	/**
	 * Returns the raw rotation in the X axis as a 16-bit signed integer.
	 *
	 * @return Raw X-axis rotation
	 */
	int16_t readRawGyroX();

	/**
	 * Returns the raw rotation in the Y axis as a 16-bit signed integer.
	 *
	 * @return Raw Y-axis rotation
	 */
	int16_t readRawGyroY();

	/**
	 * Returns the raw rotation in the Z axis as a 16-bit signed integer.
	 *
	 * @return Raw Z-axis rotation
	 */
	int16_t readRawGyroZ();

	/**
	 * Returns the raw temperature as a 16-bit signed integer.
	 *
	 * @return Raw temperature
	 */
	int16_t readRawTemp();

	/**
	 * Returns the acceleration in the X axis converted to Gs.
	 *
	 * @return X-axis acceleration [g]
	 */
	float readFloatAccelX();

	/**
	 * Returns the acceleration in the Y axis converted to Gs.
	 *
	 * @return Y-axis acceleration [g]
	 */
	float readFloatAccelY();

	/**
	 * Returns the acceleration in the Z axis converted to Gs.
	 *
	 * @return Z-axis acceleration [g]
	 */
	float readFloatAccelZ();

	/**
	 * Returns the rotation in the X axis converted to degrees-per-second.
	 *
	 * @return X-axis rotation [dps]
	 */
	float readFloatGyroX();

	/**
	 * Returns the rotation in the Y axis converted to degrees-per-second.
	 *
	 * @return Y-axis rotation [dps]
	 */
	float readFloatGyroY();

	/**
	 * Returns the rotation in the Z axis converted to degrees-per-second.
	 *
	 * @return Z-axis rotation [dps]
	 */
	float readFloatGyroZ();

	/**
	 * Returns the raw temperature converted to Centigrade.
	 *
	 * @return Temperature [C]
	 */
	float readTempC();

	/**
	 * Converts the raw acceleration value to Gs.
	 *
	 * @param rawValue The raw acceleration value
	 *
	 * @return Acceleration [g]
	 */
	float calcAccel(int16_t rawValue) const;

	/**
	 * Converts the raw rotation value to degrees-per-second.
	 *
	 * @param rawValue The raw rotation value
	 *
	 * @return Rotation [dps]
	 */
	float calcGyro(int16_t rawValue) const;

	/**
	 * Enable the sensor's FIFO.
	 */
	void fifoBegin();

	/**
	 * Disable the sensor's FIFO.
	 */
	void fifoEnd();

	/**
	 * Read a value from the sensor's FIFO.
	 *
	 * @return Raw value from the sensor's FIFO
	 */
	int16_t fifoRead();

	/**
	 * Get the status of the sensor's FIFO.
	 *
	 * @return Sensor's FIFO status
	 */
	LSM6_defs::FIFOStatus fifoGetStatus();

	/**
	 * Clear out the sensor's FIFO. Calls @ref fifoRead() repeatedly until the FIFO is empty.
	 */
	void fifoClear();

	/**
	 * Create a SharedPtr instance of the LSM6.
	 *
	 * Usage: `LSM6::makeShared(args...)`.
	 * See constructor (@ref LSM6::LSM6()) for details.
	 */
	template<typename ... Args>
	static LSM6::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<LSM6>(std::forward<Args>(args) ...);
	}

	/**
	 * The default settings the sensor will be initialized with (unless changed with @ref updateSettings() before @ref initialize()).
	 */
	static constexpr LSM6_defs::SensorSettings defaultSettings = {
		.gyroEnabled = true,
		.gyroRange = LSM6_defs::GYRO_RANGE_1000DPS,
		.gyroSampleRate = LSM6_defs::DATA_RATE_416HZ,
		.gyroFifoEnabled = true,
		.gyroFifoDecimation = LSM6_defs::FIFO_DECIMATION_NONE,
		.accelEnabled = true,
		.accelODRDisabled = true,
		.accelRange = LSM6_defs::ACCEL_RANGE_8G,
		.accelSampleRate = LSM6_defs::DATA_RATE_416HZ,
		.accelBandWidth = LSM6_defs::ACCEL_AA_BANDWIDTH_100HZ,
		.accelFifoEnabled = true,
		.accelFifoDecimation = LSM6_defs::FIFO_DECIMATION_NONE,
		.tempEnabled = false,
		.fifoThreshold = 3000,
		.fifoSampleRate = LSM6_defs::FIFO_ODR_10HZ,
		.fifoMode = LSM6_defs::FIFO_MODE_BYPASS,
	};

private:
	static constexpr uint8_t DEFAULT_DEVICE_ADDRESS = 0x6B;
	static constexpr uint8_t REFERENCE_WHO_I_AM = 0x69;
	// Temperature calculation values based on ST's docs, section 4.3 (Temperature sensor characteristics), page 24
	static constexpr float TEMPERATURE_DIVIDER = 16.0f;
	static constexpr float TEMPERATURE_OFFSET = 25.0f;

	I2CBus::SharedPtr i2cBus;
	uint8_t address;

	// IMU settings
	LSM6_defs::SensorSettings settings;

	bool initialized;

	float accelerometerMultiplier;
	float gyroscopeMultiplier;
};
