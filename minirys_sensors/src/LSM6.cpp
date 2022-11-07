#include "LSM6.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>

using namespace LSM6_defs;

LSM6::LSM6(I2CBus::SharedPtr i2cBus, uint8_t address):
	i2cBus(std::move(i2cBus)),
	address(address),
	settings{},
	initialized(false),
	accelerometerMultiplier(2.0f),
	gyroscopeMultiplier(125.0f) {
	this->settings = LSM6::defaultSettings;
}

void LSM6::initialize() {
	// Check the ID register to determine if the operation was a success.
	if (this->i2cBus->read8(this->address, REG_ADDR_WHO_AM_I_REG) != LSM6::REFERENCE_WHO_I_AM) {
		throw(std::runtime_error("Invalid WHO_I_AM received"));
	}
	this->initialized = true;

	this->writeSettings();
}

void LSM6::updateSettings(const SensorSettings& newSettings) {
	this->settings = newSettings;
}

SensorSettings LSM6::getSettings() {
	return this->settings;
}

void LSM6::writeSettings() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling writeSettings()!");
	}
	// Accelerometer settings
	AccelSettings accelSettings {};
	if (this->settings.accelEnabled) {
		accelSettings.aaBandwidth = this->settings.accelBandWidth;
		accelSettings.range = this->settings.accelRange;
		accelSettings.dataRate = this->settings.accelSampleRate;
	}
	this->i2cBus->write8(this->address, REG_ADDR_CTRL1_XL, accelSettings.rawData);
	switch (this->settings.accelRange) {
		case ACCEL_RANGE_2G:
			this->accelerometerMultiplier = 2.0f;
			break;
		case ACCEL_RANGE_4G:
			this->accelerometerMultiplier = 4.0f;
			break;
		case ACCEL_RANGE_8G:
			this->accelerometerMultiplier = 8.0f;
			break;
		case ACCEL_RANGE_16G:
			this->accelerometerMultiplier = 16.0f;
			break;
	}

	// Gyroscope settings
	GyroSettings gyroSettings {};
	if (this->settings.gyroEnabled) {
		gyroSettings.range = this->settings.gyroRange;
		gyroSettings.dataRate = this->settings.gyroSampleRate;
	}
	this->i2cBus->write8(this->address, REG_ADDR_CTRL2_G, gyroSettings.rawData);
	switch (this->settings.gyroRange) {
		case LSM6_defs::GYRO_RANGE_125DPS:
			this->gyroscopeMultiplier = 125.0f;
			break;
		case LSM6_defs::GYRO_RANGE_250DPS:
			this->gyroscopeMultiplier = 250.0f;
			break;
		case LSM6_defs::GYRO_RANGE_500DPS:
			this->gyroscopeMultiplier = 500.0f;
			break;
		case LSM6_defs::GYRO_RANGE_1000DPS:
			this->gyroscopeMultiplier = 1000.0f;
			break;
		case LSM6_defs::GYRO_RANGE_2000DPS:
			this->gyroscopeMultiplier = 2000.0f;
			break;
	}

	// Additional settings
	Control4Settings c4Settings {};
	c4Settings.accelBandwidthSelection = this->settings.accelODRDisabled;
	c4Settings.temperatureFIFOEnabled = this->settings.tempEnabled;
	this->i2cBus->write8(this->address, REG_ADDR_CTRL4_C, c4Settings.rawData);
}

int16_t LSM6::readRawAccelX() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawAccelX()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTX_L_XL, false));
}

int16_t LSM6::readRawAccelY() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawAccelY()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTY_L_XL, false));
}

int16_t LSM6::readRawAccelZ() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawAccelZ()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTZ_L_XL, false));
}

int16_t LSM6::readRawGyroX() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawGyroX()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTX_L_G, false));
}

int16_t LSM6::readRawGyroY() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawGyroY()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTY_L_G, false));
}

int16_t LSM6::readRawGyroZ() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawGyroZ()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUTZ_L_G, false));
}

int16_t LSM6::readRawTemp() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling readRawTemp()!");
	}
	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_OUT_TEMP_L, false));
}

float LSM6::readFloatAccelX() {
	return this->calcAccel(this->readRawAccelX());
}

float LSM6::readFloatAccelY() {
	return this->calcAccel(this->readRawAccelY());
}

float LSM6::readFloatAccelZ() {
	return this->calcAccel(this->readRawAccelZ());
}

float LSM6::readFloatGyroX() {
	return this->calcGyro(this->readRawGyroX());
}

float LSM6::readFloatGyroY() {
	return this->calcGyro(this->readRawGyroY());
}

float LSM6::readFloatGyroZ() {
	return this->calcGyro(this->readRawGyroZ());
}

float LSM6::readTempC() {
	return static_cast<float>(this->readRawTemp()) / LSM6::TEMPERATURE_DIVIDER + LSM6::TEMPERATURE_OFFSET;
}

float LSM6::calcAccel(int16_t rawValue) const {
	// Raw acceleration is a word (16 bits) covering the whole selected range
	return static_cast<float>(rawValue) * this->accelerometerMultiplier / INT16_MAX;
}

float LSM6::calcGyro(int16_t rawValue) const {
	// Raw rotation is a word (16 bits) covering the whole selected range
	return static_cast<float>(rawValue) * this->gyroscopeMultiplier / INT16_MAX;
}

void LSM6::fifoBegin() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling fifoBegin()!");
	}

	// Enable the FIFO in our saved settings
	this->settings.fifoMode = FIFO_MODE_CONTINUOUS_OVERWRITE_OLDER;

	// Setup a FIFO settings structure
	FIFOSettings fifoSettings {};
	fifoSettings.threshold = this->settings.fifoThreshold;
	if (this->settings.accelFifoEnabled) {
		fifoSettings.accelDecimation = this->settings.accelFifoDecimation;
	}
	if (this->settings.gyroFifoEnabled) {
		fifoSettings.gyroDecimation = this->settings.gyroFifoDecimation;
	}
	fifoSettings.dataRate = this->settings.fifoSampleRate;
	fifoSettings.mode = this->settings.fifoMode;

	// Write the settings
	this->i2cBus->write<uint8_t>(this->address, REG_ADDR_FIFO_CTRL1, fifoSettings.rawData, sizeof(FIFOSettings));
}

void LSM6::fifoEnd() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling fifoEnd()!");
	}

	// Set the FIFO mode to bypass (FIFO disabled)
	this->settings.fifoMode = FIFO_MODE_BYPASS;

	// Setup a FIFO settings structure (most content doesn't matter)
	FIFOSettings fifoSettings {};
	fifoSettings.mode = this->settings.fifoMode;

	// Write the new settings, disabling the FIFO
	this->i2cBus->write<uint8_t>(this->address, REG_ADDR_FIFO_CTRL1, fifoSettings.rawData, sizeof(FIFOSettings));
}

int16_t LSM6::fifoRead() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling fifoRead()!");
	}

	return static_cast<int16_t>(this->i2cBus->read16(this->address, REG_ADDR_FIFO_DATA_OUT_L), false);
}

FIFOStatus LSM6::fifoGetStatus() {
	if (!this->initialized) {
		throw std::runtime_error("[LSM6] Sensor not initialized before calling fifoGetStatus()!");
	}

	FIFOStatus status {};
	this->i2cBus->read<uint8_t>(this->address, REG_ADDR_FIFO_STATUS1, status.rawData, sizeof(FIFOStatus));
	return status;
}

void LSM6::fifoClear() {
	// Drain the FIFO data and dump it
	while (!this->fifoGetStatus().empty) {
		this->fifoRead();
	}
}
