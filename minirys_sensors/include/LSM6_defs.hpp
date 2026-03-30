#pragma once

#include <cstdint>

namespace LSM6_defs {
/**
 * LSM6 register addresses. Internal use only.
 */
enum RegisterAddresses : uint8_t {
	REG_ADDR_TEST_PAGE = 0x00,
	REG_ADDR_RAM_ACCESS = 0x01,
	REG_ADDR_SENSOR_SYNC_TIME = 0x04,
	REG_ADDR_SENSOR_SYNC_EN = 0x05,
	REG_ADDR_FIFO_CTRL1 = 0x06,
	REG_ADDR_FIFO_CTRL2 = 0x07,
	REG_ADDR_FIFO_CTRL3 = 0x08,
	REG_ADDR_FIFO_CTRL4 = 0x09,
	REG_ADDR_FIFO_CTRL5 = 0x0A,
	REG_ADDR_ORIENT_CFG_G = 0x0B,
	REG_ADDR_REFERENCE_G = 0x0C,
	REG_ADDR_INT1_CTRL = 0x0D,
	REG_ADDR_INT2_CTRL = 0x0E,
	REG_ADDR_WHO_AM_I_REG = 0x0F,
	REG_ADDR_CTRL1_XL = 0x10,
	REG_ADDR_CTRL2_G = 0x11,
	REG_ADDR_CTRL3_C = 0x12,
	REG_ADDR_CTRL4_C = 0x13,
	REG_ADDR_CTRL5_C = 0x14,
	REG_ADDR_CTRL6_G = 0x15,
	REG_ADDR_CTRL7_G = 0x16,
	REG_ADDR_CTRL8_XL = 0x17,
	REG_ADDR_CTRL9_XL = 0x18,
	REG_ADDR_CTRL10_C = 0x19,
	REG_ADDR_MASTER_CONFIG = 0x1A,
	REG_ADDR_WAKE_UP_SRC = 0x1B,
	REG_ADDR_TAP_SRC = 0x1C,
	REG_ADDR_D6D_SRC = 0x1D,
	REG_ADDR_STATUS_REG = 0x1E,
	REG_ADDR_OUT_TEMP_L = 0x20,
	REG_ADDR_OUT_TEMP_H = 0x21,
	REG_ADDR_OUTX_L_G = 0x22,
	REG_ADDR_OUTX_H_G = 0x23,
	REG_ADDR_OUTY_L_G = 0x24,
	REG_ADDR_OUTY_H_G = 0x25,
	REG_ADDR_OUTZ_L_G = 0x26,
	REG_ADDR_OUTZ_H_G = 0x27,
	REG_ADDR_OUTX_L_XL = 0x28,
	REG_ADDR_OUTX_H_XL = 0x29,
	REG_ADDR_OUTY_L_XL = 0x2A,
	REG_ADDR_OUTY_H_XL = 0x2B,
	REG_ADDR_OUTZ_L_XL = 0x2C,
	REG_ADDR_OUTZ_H_XL = 0x2D,
	REG_ADDR_SENSORHUB1_REG = 0x2E,
	REG_ADDR_SENSORHUB2_REG = 0x2F,
	REG_ADDR_SENSORHUB3_REG = 0x30,
	REG_ADDR_SENSORHUB4_REG = 0x31,
	REG_ADDR_SENSORHUB5_REG = 0x32,
	REG_ADDR_SENSORHUB6_REG = 0x33,
	REG_ADDR_SENSORHUB7_REG = 0x34,
	REG_ADDR_SENSORHUB8_REG = 0x35,
	REG_ADDR_SENSORHUB9_REG = 0x36,
	REG_ADDR_SENSORHUB10_REG = 0x37,
	REG_ADDR_SENSORHUB11_REG = 0x38,
	REG_ADDR_SENSORHUB12_REG = 0x39,
	REG_ADDR_FIFO_STATUS1 = 0x3A,
	REG_ADDR_FIFO_STATUS2 = 0x3B,
	REG_ADDR_FIFO_STATUS3 = 0x3C,
	REG_ADDR_FIFO_STATUS4 = 0x3D,
	REG_ADDR_FIFO_DATA_OUT_L = 0x3E,
	REG_ADDR_FIFO_DATA_OUT_H = 0x3F,
	REG_ADDR_TIMESTAMP0_REG = 0x40,
	REG_ADDR_TIMESTAMP1_REG = 0x41,
	REG_ADDR_TIMESTAMP2_REG = 0x42,
	REG_ADDR_STEP_COUNTER_L = 0x4B,
	REG_ADDR_STEP_COUNTER_H = 0x4C,
	REG_ADDR_FUNC_SRC = 0x53,
	REG_ADDR_TAP_CFG1 = 0x58,
	REG_ADDR_TAP_THS_6D = 0x59,
	REG_ADDR_INT_DUR2 = 0x5A,
	REG_ADDR_WAKE_UP_THS = 0x5B,
	REG_ADDR_WAKE_UP_DUR = 0x5C,
	REG_ADDR_FREE_FALL = 0x5D,
	REG_ADDR_MD1_CFG = 0x5E,
	REG_ADDR_MD2_CFG = 0x5F,
};

/**
 * FIFO data rate decimation values, used in @ref LSM6_defs::SensorSettings
 */
enum FIFODecimationValues : uint8_t {
	FIFO_SENSOR_DISABLED = 0x00,
	FIFO_DECIMATION_NONE = 0x01,
	FIFO_DECIMATION_BY_2 = 0x02,
	FIFO_DECIMATION_BY_3 = 0x03,
	FIFO_DECIMATION_BY_4 = 0x04,
	FIFO_DECIMATION_BY_8 = 0x05,
	FIFO_DECIMATION_BY_16 = 0x06,
	FIFO_DECIMATION_BY_32 = 0x07,
};

enum FIFOModes : uint8_t {
	FIFO_MODE_BYPASS = 0x00,
	FIFO_MODE_FIFO = 0x01,
	FIFO_MODE_CONTINUOUS_UNTIL_TRIGGER_DEASSERTED = 0x03,
	FIFO_MODE_BYPASS_UNTIL_TRIGGER_DEASSERTED = 0x04,
	FIFO_MODE_CONTINUOUS_OVERWRITE_OLDER = 0x06,
};

enum FIFODataRates : uint8_t {
	FIFO_ODR_DISABLED = 0x00,
	FIFO_ODR_10HZ = 0x01,
	FIFO_ODR_25HZ = 0x02,
	FIFO_ODR_50HZ = 0x03,
	FIFO_ODR_100HZ = 0x04,
	FIFO_ODR_200HZ = 0x05,
	FIFO_ODR_400HZ = 0x06,
	FIFO_ODR_800HZ = 0x07,
	FIFO_ODR_1600HZ = 0x08,
	FIFO_ODR_3300HZ = 0x09,
	FIFO_ODR_6600HZ = 0x0A,
};

enum AccelAABandwidths {
	ACCEL_AA_BANDWIDTH_400HZ = 0x00,
	ACCEL_AA_BANDWIDTH_200HZ = 0x01,
	ACCEL_AA_BANDWIDTH_100HZ = 0x02,
	ACCEL_AA_BANDWIDTH_50HZ = 0x03,
};

enum AccelRanges {
	ACCEL_RANGE_2G = 0x00,
	ACCEL_RANGE_16G = 0x01,
	ACCEL_RANGE_4G = 0x02,
	ACCEL_RANGE_8G = 0x03,
};

enum DataRates : uint8_t {
	DATA_RATE_POWER_DOWN = 0x00,
	DATA_RATE_13HZ = 0x01,
	DATA_RATE_26HZ = 0x02,
	DATA_RATE_52HZ = 0x03,
	DATA_RATE_104HZ = 0x04,
	DATA_RATE_208HZ = 0x05,
	DATA_RATE_416HZ = 0x06,
	DATA_RATE_833HZ = 0x07,
	DATA_RATE_1660HZ = 0x08,
	DATA_RATE_3330HZ = 0x09,
	DATA_RATE_6660HZ = 0x0A,
};

enum GyroRanges {
	GYRO_RANGE_125DPS = 0x01,
	GYRO_RANGE_250DPS = 0x00,
	GYRO_RANGE_500DPS = 0x02,
	GYRO_RANGE_1000DPS = 0x04,
	GYRO_RANGE_2000DPS = 0x06,
};

// Registers REG_ADDR_FIFO_CTRL1:REG_ADDR_FIFO_CTRL5
union FIFOSettings {
	struct {
		// Watermark flag rises when the number of bytes written to FIFO after the next write is greater than or equal to the threshold level; Default: 000
		uint16_t threshold : 12;
		uint8_t unused1 : 2;
		// Write to FIFO: 0 - based on data-readiness; 1 at every step detected; Default: 0
		uint8_t writeMode : 1;
		// Enable pedometer step counter and timestamp as 3rd FIFO data set; Default: 0
		uint8_t pedoTimerEnabled : 1;
		// Decimation for accelerometer data, @ref FIFODecimationValues; Default: 000
		FIFODecimationValues accelDecimation : 3;
		// Decimation for gyroscope data, @ref FIFODecimationValues; Default: 000
		FIFODecimationValues gyroDecimation : 3;
		uint8_t unused2 : 2;
		uint8_t unused3 : 3;
		// Decimation for pedometer and timestamp data, @ref FIFODecimationValues; Default: 000
		FIFODecimationValues pedoTimerDecimation : 3;
		// Enable MSByte only memorization in FIFO; Default: 0
		uint8_t onlyHighData : 1;
		uint8_t unused4 : 1;
		FIFOModes mode : 3;
		FIFODataRates dataRate : 4;
		uint8_t unused5 : 1;
	}

	__attribute__((packed));

	uint8_t rawData[5];
};

// Register REG_ADDR_CTRL1_XL
union AccelSettings {
	struct {
		AccelAABandwidths aaBandwidth : 2;
		AccelRanges range : 2;
		DataRates dataRate : 4;
	}

	__attribute__((packed));

	uint8_t rawData;
};

// Register REG_ADDR_CTRL2_G
union GyroSettings {
	struct {
		uint8_t unused : 1;
		// Gyro full scale mode: 1 bit - 125dps enabled; 2 & 3 bit - selection
		GyroRanges range : 3;
		DataRates dataRate : 4;
	}

	__attribute__((packed));

	uint8_t rawData;
};

// Register REG_ADDR_CTRL4_C
union Control4Settings {
	struct {
		// 1: FIFO depth is limited to threshold level; 0: not limited; Default: 0
		uint8_t enableFIFOThreshold : 1;
		uint8_t unused : 1;
		// 1: SPI only; Default: 0
		uint8_t disableI2C : 1;
		// 1: accel and gyro data ready signals are masked on powerup until the sensor filters settle; Default: 0
		uint8_t dataReadyMaskEnabled : 1;
		// 1: Enable temperature as 3rd FIFO data set; Default: 0
		uint8_t temperatureFIFOEnabled : 1;
		// 1: All interrupts on the INT1 pad; 0: divided between INT1 and INT2; Default: 0
		uint8_t allInterruptsOnINT1 : 1;
		// 1: Enable gyroscope sleep mode; Default: 0
		uint8_t gyroSleepMode : 1;
		// 0: BW determined by ODR selection; 1: BW determined by AccelSettings.aaBandwidth; Default: 0
		uint8_t accelBandwidthSelection : 1;
	}

	__attribute__((packed));

	uint8_t rawData;
};

// Registers REG_ADDR_FIFO_STATUS1:REG_ADDR_FIFO_STATUS2
union FIFOStatus {
	struct {
		uint16_t unreadWords : 12;
		uint8_t empty : 1;
		uint8_t full : 1;
		uint8_t overrun : 1;
		uint8_t watermark : 1;
	}

	__attribute__((packed));

	uint8_t rawData[2];
};

// This struct holds the settings the driver uses to do calculations
struct SensorSettings {
	// Gyro settings
	bool gyroEnabled;
	GyroRanges gyroRange;
	DataRates gyroSampleRate;

	bool gyroFifoEnabled;
	FIFODecimationValues gyroFifoDecimation;

	// Accelerometer settings
	bool accelEnabled;
	bool accelODRDisabled;
	AccelRanges accelRange;
	DataRates accelSampleRate;
	AccelAABandwidths accelBandWidth;

	bool accelFifoEnabled;
	FIFODecimationValues accelFifoDecimation;

	// Temperature settings
	bool tempEnabled;

	// FIFO control data
	uint16_t fifoThreshold;
	FIFODataRates fifoSampleRate;
	FIFOModes fifoMode;
};
} // namespace LSM6_defs
