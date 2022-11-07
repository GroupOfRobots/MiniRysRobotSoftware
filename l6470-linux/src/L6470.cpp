#include "L6470.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <thread>

using namespace std::chrono_literals;

L6470::L6470(uint8_t driverCount, SPIBus::SharedPtr spiBus, GPIOPin::SharedPtr resetPin, GPIOPin::SharedPtr busyPin):
	driverCount(driverCount),
	spiBus(std::move(spiBus)),
	resetPin(std::move(resetPin)),
	busyPin(std::move(busyPin)) {
	// Prepare transfer vectors
	this->commands.resize(driverCount);
	this->nBytes.resize(driverCount);
	this->txValues.resize(driverCount);
	this->rxValues.resize(driverCount);

	// Reset the device
	this->resetDeviceHw();

	// Zero-out the transfer buffers
	this->resetBuffers();
}

std::vector<L6470::Status> L6470::getStatus(OptUInt8 index) {
	this->setCommand(L6470::COMMAND_GET_STATUS, index);
	this->setNBytes(2, index);
	this->transfer();
	auto rawStatuses = this->getValues(index);
	std::vector<L6470::Status> statuses;
	std::transform(rawStatuses.begin(), rawStatuses.end(), std::back_inserter(statuses), [](uint32_t rawStatus) {
		L6470::Status status {};
		status.rawStatus = static_cast<uint16_t>(rawStatus);
		return status;
	});
	this->resetBuffers();
	return statuses;
}

std::vector<L6470::Status> L6470::readStatus(OptUInt8 index) {
	return this->getParamsMapped<L6470::Status>(L6470::REG_ADDR_STATUS, [](uint32_t value) {
		L6470::Status status {};
		status.rawStatus = static_cast<uint16_t>(value);
		return status;
	}, index);
}

std::vector<bool> L6470::isBusy(OptUInt8 index) {
	return this->getParamsMapped<bool>(L6470::REG_ADDR_STATUS, [](uint32_t value) {
		return !(value & STATUS_MASK_BUSY);
	}, index);
}

bool L6470::isAnyBusy() {
	if (this->busyPin) {
		return !this->busyPin->read();
	}
	auto isBusyVec = this->isBusy();
	return std::any_of(isBusyVec.begin(), isBusyVec.end(), [](bool v) {
		return v;
	});
}

// ###############
// Operational commands
// ###############

std::vector<int32_t> L6470::getPosition(OptUInt8 index) {
	return this->getParamsMapped<int32_t>(L6470::REG_ADDR_ABS_POS, [](uint32_t value) {
		// Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if it's set,
		// set all the bits ABOVE 21 in order for the value to maintain its appropriate sign.
		if (value & TWO_COMP_22B_SIGN_BIT) {
			value |= TWO_COMP_22B_COMP_FIX;
		}
		return static_cast<int32_t>(value);
	}, index);
}

std::vector<float> L6470::getSpeed(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_SPEED, L6470::speedParse, index);
}

std::vector<L6470::Direction> L6470::getDirection(OptUInt8 index) {
	return this->getParamsMapped<L6470::Direction>(L6470::REG_ADDR_STATUS, [](uint32_t value) {
		return (value & STATUS_DIR_MASK) ? L6470::DIRECTION_FWD : L6470::DIRECTION_REV;
	}, index);
}

std::vector<int32_t> L6470::getMarkPosition(OptUInt8 index) {
	return this->getParamsMapped<int32_t>(L6470::REG_ADDR_MARK, [](uint32_t value) {
		// See the note in @ref getPosition() for the explanation on this transform.
		if (value & TWO_COMP_22B_SIGN_BIT) {
			value |= TWO_COMP_22B_COMP_FIX;
		}
		return static_cast<int32_t>(value);
	}, index);
}

void L6470::run(std::vector<float> speeds, std::vector<Direction> directions, OptUInt8 index) {
	if (std::any_of(speeds.begin(), speeds.end(), [](float speed) {
		return speed > SPEED_SPS_MAX || speed < 0;
	})) {
		throw std::runtime_error("Invalid speed");
	}

	std::vector<uint8_t> commands;
	std::vector<uint32_t> values;
	for (int i = 0; i < this->driverCount; ++i) {
		commands.push_back(static_cast<uint8_t>(L6470::COMMAND_RUN) | directions[i]);
		values.push_back(L6470::speedCalc(speeds[i]));
	}

	this->setCommand(commands, index);
	this->setNBytes(3, index);
	this->setValues(values, index);
	this->transfer();
	this->resetBuffers();
}

void L6470::run(float speed, const Direction& direction, OptUInt8 index) {
	this->run(
		std::vector<float>(this->driverCount, speed),
		std::vector<Direction>(this->driverCount, direction),
		index
	);
}

void L6470::move(std::vector<uint32_t> numSteps, std::vector<Direction> directions, OptUInt8 index) {
	if (std::any_of(numSteps.begin(), numSteps.end(), [](float steps) {
		return steps > STEPS_INT_MAX;
	})) {
		throw std::runtime_error("Invalid numSteps");
	}

	std::vector<uint8_t> commands(this->driverCount);
	for (int i = 0; i < this->driverCount; ++i) {
		commands[i] = static_cast<uint8_t>(L6470::COMMAND_MOVE) | directions[i];
	}

	this->setCommand(commands, index);
	this->setNBytes(3, index);
	this->setValues(numSteps, index);
	this->transfer();
	this->resetBuffers();
}

void L6470::move(uint32_t numSteps, const Direction& direction, OptUInt8 index) {
	this->move(
		std::vector<uint32_t>(this->driverCount, numSteps),
		std::vector<Direction>(this->driverCount, direction),
		index
	);
}

void L6470::goTo(std::vector<int32_t> positions, OptUInt8 index) {
	if (std::any_of(positions.begin(), positions.end(), [](float position) {
		return position > POSITION_MAX || position < POSITION_MIN;
	})) {
		throw std::runtime_error("Invalid position");
	}

	std::vector<uint32_t> values(this->driverCount);
	for (int i = 0; i < this->driverCount; ++i) {
		values[i] = static_cast<uint32_t>(positions[i]);
	}

	this->setCommand(L6470::COMMAND_GOTO, index);
	this->setNBytes(3, index);
	this->setValues(values, index);
	this->transfer();
	this->resetBuffers();
}

void L6470::goTo(int32_t position, OptUInt8 index) {
	this->goTo(std::vector<int32_t>(this->driverCount, position), index);
}

void L6470::goToDirection(std::vector<int32_t> positions, std::vector<Direction> directions, OptUInt8 index) {
	if (std::any_of(positions.begin(), positions.end(), [](float position) {
		return position > POSITION_MAX || position < POSITION_MIN;
	})) {
		throw std::runtime_error("Invalid position");
	}

	std::vector<uint8_t> commands(this->driverCount);
	std::vector<uint32_t> values(this->driverCount);
	for (int i = 0; i < this->driverCount; ++i) {
		commands[i] = static_cast<uint8_t>(L6470::COMMAND_GOTO_DIR) | directions[i];
		values[i] = static_cast<uint32_t>(positions[i]);
	}

	this->setCommand(commands, index);
	this->setNBytes(3, index);
	this->setValues(values, index);
	this->transfer();
	this->resetBuffers();
}

void L6470::goToDirection(int32_t position, const Direction& direction, OptUInt8 index) {
	this->goToDirection(
		std::vector<int32_t>(this->driverCount, position),
		std::vector<Direction>(this->driverCount, direction),
		index
	);
}

void L6470::goUntil(
	std::vector<float> speeds,
	std::vector<Direction> directions,
	std::vector<Action> actions,
	OptUInt8 index
) {
	if (std::any_of(speeds.begin(), speeds.end(), [](float speed) {
		return speed > SPEED_SPS_MAX || speed < 0;
	})) {
		throw std::runtime_error("Invalid speed");
	}

	std::vector<uint8_t> commands(this->driverCount);
	std::vector<uint32_t> values(this->driverCount);
	for (int i = 0; i < this->driverCount; ++i) {
		commands[i] = static_cast<uint8_t>(L6470::COMMAND_GO_UNTIL) | directions[i] | actions[i];
		values[i] = static_cast<uint32_t>(L6470::speedCalc(speeds[i]));
	}

	this->setCommand(commands, index);
	this->setNBytes(3, index);
	this->setValues(values, index);
	this->transfer();
	this->resetBuffers();
}

void L6470::goUntil(float speed, const Direction& direction, const Action& action, OptUInt8 index) {
	this->goUntil(
		std::vector<float>(this->driverCount, speed),
		std::vector<Direction>(this->driverCount, direction),
		std::vector<Action>(this->driverCount, action),
		index
	);
}

void L6470::releaseSw(std::vector<Direction> directions, std::vector<Action> actions, OptUInt8 index) {
	std::vector<uint8_t> commands(this->driverCount);
	for (int i = 0; i < this->driverCount; ++i) {
		commands[i] = static_cast<uint8_t>(L6470::COMMAND_RELEASE_SW) | directions[i] | actions[i];
	}
	this->runCommand(commands, index);
}

void L6470::releaseSw(const Direction& direction, const Action& action, OptUInt8 index) {
	uint8_t command = static_cast<uint8_t>(L6470::COMMAND_RELEASE_SW) | action | direction;
	this->runCommand(command, index);
}

void L6470::stepClock(const L6470::Direction& direction, OptUInt8 index) {
	uint8_t command = static_cast<uint8_t>(L6470::COMMAND_STEP_CLOCK) | direction;
	this->runCommand(command, index);
}

void L6470::goHome(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_GO_HOME, index);
}

void L6470::goMark(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_GO_MARK, index);
}

void L6470::setMark(int32_t newMark, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_MARK, newMark, index);
}

void L6470::setPosition(int32_t newPosition, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_ABS_POS, newPosition, index);
}

void L6470::resetPosition(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_RESET_POS, index);
}

void L6470::resetDevice(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_RESET_DEVICE, index);
}

void L6470::resetDeviceHw() {
	this->resetPin->unset();
	std::this_thread::sleep_for(5ms);
	this->resetPin->set();
	std::this_thread::sleep_for(5ms);
}

void L6470::softStop(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_SOFT_STOP, index);
}

void L6470::hardStop(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_HARD_STOP, index);
}

void L6470::softHiZ(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_SOFT_HIZ, index);
}

void L6470::hardHiZ(OptUInt8 index) {
	this->runCommand(L6470::COMMAND_HARD_HIZ, index);
}

// ###############
// Configuration commands
// ###############

void L6470::configSyncPin(
	const L6470::PinFunction& pinFunction,
	const L6470::SyncPulseDivisor& syncSteps,
	OptUInt8 index
) {
	auto syncPinConfigs = this->getParam(L6470::REG_ADDR_STEP_MODE, index);
	for (auto& config: syncPinConfigs) {
		// Clear the bits we're about to set
		config &= ~(STEP_MODE_SYNC_MASK);

		// OR in the arguments, masking the incoming data to just to be sure
		config |= ((pinFunction & STEP_MODE_PIN_FUNC_MASK) | (syncSteps & STEP_MODE_SYNC_STEP_MASK));
	}

	this->setParam(L6470::REG_ADDR_STEP_MODE, syncPinConfigs, index);
}

void L6470::configStepMode(const L6470::StepMode& stepMode, OptUInt8 index) {
	auto syncPinConfigs = this->getParam(L6470::REG_ADDR_STEP_MODE, index);
	for (auto& config: syncPinConfigs) {
		// Clear the bits we're about to set
		config &= ~(STEP_MODE_STEP_SEL_MASK);

		// OR in the arguments, masking the incoming data to just to be sure
		config |= (stepMode & STEP_MODE_STEP_SEL_MASK);
	}

	this->setParam(L6470::REG_ADDR_STEP_MODE, syncPinConfigs, index);
}

std::vector<L6470::StepMode> L6470::getStepMode(OptUInt8 index) {
	return this->getParamsMapped<L6470::StepMode>(L6470::REG_ADDR_STEP_MODE, [](uint32_t value) {
		return static_cast<L6470::StepMode>(value & STEP_MODE_STEP_SEL_MASK);
	}, index);
}

void L6470::setMaxSpeed(float stepsPerSecond, OptUInt8 index) {
	if (stepsPerSecond < MAX_SPEED_MIN || stepsPerSecond > MAX_SPEED_MAX) {
		throw std::runtime_error("Invalid stepsPerSecond");
	}
	this->setParam(L6470::REG_ADDR_MAX_SPEED, L6470::maxFsSpeedCalc(stepsPerSecond), index);
}

std::vector<float> L6470::getMaxSpeed(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_MAX_SPEED, L6470::maxFsSpeedParse, index);
}

void L6470::setMinSpeed(float stepsPerSecond, OptUInt8 index) {
	if (stepsPerSecond < MIN_SPEED_MIN || stepsPerSecond > MIN_SPEED_MAX) {
		throw std::runtime_error("Invalid stepsPerSecond");
	}
	uint32_t minSpeedRaw = L6470::minSpeedCalc(stepsPerSecond);
	auto minSpeeds = this->getParam(L6470::REG_ADDR_MIN_SPEED, index);
	for (auto& value: minSpeeds) {
		// MIN_SPEED also contains the LSPD_OPT flag, so we need to protect that.
		value = minSpeedRaw | (value & MIN_SPEED_LSPD_MASK);
	}

	this->setParam(L6470::REG_ADDR_MIN_SPEED, minSpeeds, index);
}

std::vector<float> L6470::getMinSpeed(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_MIN_SPEED, L6470::minSpeedParse, index);
}

void L6470::setFullSpeed(float stepsPerSecond, OptUInt8 index) {
	if (stepsPerSecond < FULL_SPEED_MIN || stepsPerSecond > FULL_SPEED_MAX) {
		throw std::runtime_error("Invalid stepsPerSecond");
	}
	this->setParam(L6470::REG_ADDR_FS_SPD, L6470::maxFsSpeedCalc(stepsPerSecond), index);
}

std::vector<float> L6470::getFullSpeed(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_FS_SPD, L6470::maxFsSpeedParse, index);
}

void L6470::setAcceleration(float acceleration, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_ACC, L6470::accelerationCalc(acceleration), index);
}

std::vector<float> L6470::getAcceleration(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_ACC, L6470::accelerationParse, index);
}

void L6470::setDeceleration(float deceleration, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_DECEL, L6470::decelerationCalc(deceleration), index);
}

std::vector<float> L6470::getDeceleration(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_DECEL, L6470::decelerationParse, index);
}

void L6470::setLowSpeedOptimization(bool enable, OptUInt8 index) {
	auto minSpeeds = this->getParam(L6470::REG_ADDR_MIN_SPEED, index);
	for (auto& value: minSpeeds) {
		if (enable) {
			// Set the LSPD_OPT bit
			value |= MIN_SPEED_LSPD_MASK;
		} else {
			// Clear the LSPD_OPT bit
			value &= ~(MIN_SPEED_LSPD_MASK);
		}
	}

	this->setParam(L6470::REG_ADDR_MIN_SPEED, minSpeeds, index);
}

std::vector<bool> L6470::getLowSpeedOptimization(OptUInt8 index) {
	return this->getParamsMapped<bool>(L6470::REG_ADDR_MIN_SPEED, [](uint32_t value) {
		return value & MIN_SPEED_LSPD_MASK;
	}, index);
}

void L6470::setOCThreshold(float threshold, OptUInt8 index) {
	threshold = std::min(std::max(threshold, 375.0f), 6000.0f);
	auto thresholdRaw = std::lround(threshold / 375.0f);
	this->setParam(L6470::REG_ADDR_OCD_TH, 0x0F & thresholdRaw, index);
}

std::vector<float> L6470::getOCThreshold(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_OCD_TH, [](uint32_t value) {
		return static_cast<float>(value & 0x0F) * 375.0f;
	}, index);
}

void L6470::setStallThreshold(float threshold, OptUInt8 index) {
	threshold = std::min(std::max(threshold, 31.25f), 4000.0f);
	auto thresholdRaw = std::lround(threshold / 31.25f);
	this->setParam(L6470::REG_ADDR_STALL_TH, 0x3F & thresholdRaw, index);
}

std::vector<float> L6470::getStallThreshold(OptUInt8 index) {
	return this->getParamsMapped<float>(L6470::REG_ADDR_STALL_TH, [](uint32_t value) {
		return static_cast<float>(value & 0x3F) * 31.25f;
	}, index);
}

void L6470::setPWMFrequency(
	const L6470::PWMDivisor& divisor,
	const L6470::PWMMultiplier& multiplier,
	OptUInt8 index
) {
	uint16_t fullDivisor = (static_cast<uint16_t>(divisor) << PWM_DIVISOR_SHIFT) & 0xE000;
	uint16_t fullMultiplier = (static_cast<uint16_t>(multiplier) << PWM_MULTIPLIER_SHIFT) & 0x1C00;
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// The divisor is set by config 15:13, so mask 0xE000 to clear them.
		configVal &= ~(0xE000);
		// The multiplier is set by config 12:10; mask is 0x1C00
		configVal &= ~(0x1C00);
		// Now we can OR in the masked-out versions of the values passed in.
		configVal |= (fullDivisor | fullMultiplier);
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<L6470::PWMDivisor> L6470::getPWMFrequencyDivisor(OptUInt8 index) {
	return this->getParamsMapped<L6470::PWMDivisor>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return static_cast<L6470::PWMDivisor>((value & 0xE000) >> PWM_DIVISOR_SHIFT);
	}, index);
}

std::vector<L6470::PWMMultiplier> L6470::getPWMFrequencyMultiplier(OptUInt8 index) {
	return this->getParamsMapped<L6470::PWMMultiplier>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return static_cast<L6470::PWMMultiplier>((value & 0x1C00) >> PWM_MULTIPLIER_SHIFT);
	}, index);
}

void L6470::setSlewRate(const L6470::SlewRate& slewRate, OptUInt8 index) {
	uint16_t fullSlewRate = (static_cast<uint16_t>(slewRate) << SLEW_RATE_SHIFT) & 0x0300;
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// These bits live in CONFIG 9:8, so the mask is 0x0300.
		configVal &= ~(0x0300);
		// Now, OR in the masked incoming value.
		configVal |= fullSlewRate;
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<L6470::SlewRate> L6470::getSlewRate(OptUInt8 index) {
	return this->getParamsMapped<L6470::SlewRate>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return static_cast<L6470::SlewRate>((value & 0x0300) >> SLEW_RATE_SHIFT);
	}, index);
}

void L6470::setOCShutdown(bool enabled, OptUInt8 index) {
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// This bit is CONFIG 7, mask is 0x0080
		configVal &= ~(0x0080);
		// Now, OR in the masked incoming value.
		configVal |= (0x0080 & (enabled ? OC_SHUTDOWN_ENABLED : OC_SHUTDOWN_DISABLED));
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<bool> L6470::getOCShutdown(OptUInt8 index) {
	return this->getParamsMapped<bool>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return (value & 0x0080) == OC_SHUTDOWN_ENABLED;
	}, index);
}

void L6470::setVoltageCompensation(bool voltageCompEnabled, OptUInt8 index) {
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// This bit is CONFIG 5, mask is 0x0020
		configVal &= ~(0x0020);
		// Now, OR in the masked incoming value.
		configVal |= (0x0020 & (voltageCompEnabled ? VOLTAGE_COMP_ENABLED : VOLTAGE_COMP_DISABLED));
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<bool> L6470::getVoltageCompensation(OptUInt8 index) {
	return this->getParamsMapped<bool>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return (value & 0x0020) == VOLTAGE_COMP_ENABLED;
	}, index);
}

void L6470::setSwitchMode(const L6470::SwitchMode& switchMode, OptUInt8 index) {
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// This bit is CONFIG 4, mask is 0x0010
		configVal &= ~(0x0010);
		// Now, OR in the masked incoming value.
		configVal |= (0x0010 & switchMode);
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<L6470::SwitchMode> L6470::getSwitchMode(OptUInt8 index) {
	return this->getParamsMapped<L6470::SwitchMode>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return static_cast<L6470::SwitchMode>(value & 0x0010);
	}, index);
}

void L6470::setOscillatorMode(const L6470::OscillatorMode& oscillatorMode, OptUInt8 index) {
	auto configs = this->getParam(L6470::REG_ADDR_CONFIG, index);
	for (auto& configVal: configs) {
		// These bits are CONFIG 3:0, mask is 0x000F
		configVal &= ~(0x000F);
		// Now, OR in the masked incoming value.
		configVal |= (0x000F & oscillatorMode);
	}

	this->setParam(L6470::REG_ADDR_CONFIG, configs, index);
}

std::vector<L6470::OscillatorMode> L6470::getOscillatorMode(OptUInt8 index) {
	return this->getParamsMapped<L6470::OscillatorMode>(L6470::REG_ADDR_CONFIG, [](uint32_t value) {
		return static_cast<L6470::OscillatorMode>(value & 0x000F);
	}, index);
}

void L6470::setAccelerationKVAL(uint8_t kvalInput, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_KVAL_ACC, kvalInput, index);
}

std::vector<uint8_t> L6470::getAccelerationKVAL(OptUInt8 index) {
	return this->getParamsMapped<uint8_t>(L6470::REG_ADDR_KVAL_ACC, [](uint32_t value) {
		return static_cast<uint8_t>(value);
	}, index);
}

void L6470::setDecelerationKVAL(uint8_t kvalInput, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_KVAL_DEC, kvalInput, index);
}

std::vector<uint8_t> L6470::getDecelerationKVAL(OptUInt8 index) {
	return this->getParamsMapped<uint8_t>(L6470::REG_ADDR_KVAL_DEC, [](uint32_t value) {
		return static_cast<uint8_t>(value);
	}, index);
}

void L6470::setRunKVAL(uint8_t kvalInput, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_KVAL_RUN, kvalInput, index);
}

std::vector<uint8_t> L6470::getRunKVAL(OptUInt8 index) {
	return this->getParamsMapped<uint8_t>(L6470::REG_ADDR_KVAL_RUN, [](uint32_t value) {
		return static_cast<uint8_t>(value);
	}, index);
}

void L6470::setHoldKVAL(uint8_t kvalInput, OptUInt8 index) {
	this->setParam(L6470::REG_ADDR_KVAL_HOLD, kvalInput, index);
}

std::vector<uint8_t> L6470::getHoldKVAL(OptUInt8 index) {
	return this->getParamsMapped<uint8_t>(L6470::REG_ADDR_KVAL_HOLD, [](uint32_t value) {
		return static_cast<uint8_t>(value);
	}, index);
}

// ###############
// Communication commands
// ###############

void L6470::setCommand(std::vector<uint8_t> commands, OptUInt8 index) {
	if (index == std::nullopt) {
		for (uint8_t i = 0; i < this->driverCount; i++) {
			this->commands[i] = commands[i];
		}
	} else {
		this->commands[index.value()] = commands[0];
	}
}

void L6470::setCommand(uint8_t command, OptUInt8 index) {
	this->setCommand(std::vector<uint8_t>(this->driverCount, command), index);
}

void L6470::setValues(std::vector<uint32_t> values, OptUInt8 index) {
	if (index == std::nullopt) {
		for (uint8_t i = 0; i < this->driverCount; i++) {
			this->txValues[i].fullValue = values[i];
		}
	} else {
		this->txValues[index.value()].fullValue = values[0];
	}
}

void L6470::setValues(uint32_t value, OptUInt8 index) {
	this->setValues(std::vector<uint32_t>(this->driverCount, value), index);
}

std::vector<uint32_t> L6470::getValues(OptUInt8 index) {
	std::vector<uint32_t> values;
	if (index != std::nullopt) {
		values.push_back(rxValues[index.value()].fullValue);
		return values;
	}

	std::transform(rxValues.begin(), rxValues.end(), std::back_inserter(values), [](TransferUnit tu) {
		return tu.fullValue;
	});
	return values;
}

void L6470::setNBytes(uint8_t n, OptUInt8 index) {
	if (index == std::nullopt) {
		for (uint8_t i = 0; i < this->driverCount; i++) {
			this->nBytes[i] = n;
		}
	} else {
		this->nBytes[index.value()] = n;
	}
}

void L6470::setNBytes() {
	/// TODO: refactor this using std::map or something...

	for (uint8_t i = 0; i < this->driverCount; i++) {
		// mask out GET_PARAM
		// 000xxxxx (set_param or noop)
		// 001xxxxx (get_param)
		/*
		 * first check of (???xxxxx >> 5) > 1 [command vs param]
		 * then check against ???????x >> 1 [command type]
		 */
		if ((commands[i] >> 5) < 2) {
			// param
			switch (commands[i] & 0x1F) {
				case L6470::REG_ADDR_ABS_POS:
				case L6470::REG_ADDR_MARK:
				case L6470::REG_ADDR_SPEED:
					nBytes[i] = 3;
					break;
				case L6470::REG_ADDR_EL_POS:
				case L6470::REG_ADDR_ACC:
				case L6470::REG_ADDR_DECEL:
				case L6470::REG_ADDR_MAX_SPEED:
				case L6470::REG_ADDR_MIN_SPEED:
				case L6470::REG_ADDR_FS_SPD:
				case L6470::REG_ADDR_INT_SPD:
				case L6470::REG_ADDR_CONFIG:
				case L6470::REG_ADDR_STATUS:
					nBytes[i] = 2;
					break;
				case L6470::REG_ADDR_KVAL_HOLD:
				case L6470::REG_ADDR_KVAL_RUN:
				case L6470::REG_ADDR_KVAL_ACC:
				case L6470::REG_ADDR_KVAL_DEC:
				case L6470::REG_ADDR_ST_SLP:
				case L6470::REG_ADDR_FN_SLP_ACC:
				case L6470::REG_ADDR_FN_SLP_DEC:
				case L6470::REG_ADDR_K_THERM:
				case L6470::REG_ADDR_ADC_OUT:
				case L6470::REG_ADDR_STALL_TH:
				case L6470::REG_ADDR_STEP_MODE:
				case L6470::REG_ADDR_ALARM_EN:
					nBytes[i] = 1;
					break;
				default:
					nBytes[i] = 0;
					break;
			}
		} else {
			// command
			// remove DIR bit
			uint8_t cmd = (commands[i] & 0xFE);
			switch (cmd) {
				case L6470::COMMAND_RUN:
				case L6470::COMMAND_MOVE:
				case L6470::COMMAND_GOTO:
				case L6470::COMMAND_GOTO_DIR:
				case L6470::COMMAND_GO_UNTIL:
				case 0x8A: // GO_UNTIL + ACT
				case L6470::COMMAND_GET_STATUS:
					nBytes[i] = 3;
					break;
				case L6470::COMMAND_STEP_CLOCK:
				case L6470::COMMAND_RELEASE_SW:
				case 0x9A: // RELEASE_SW + ACT
				case L6470::COMMAND_GO_HOME:
				case L6470::COMMAND_GO_MARK:
				case L6470::COMMAND_RESET_POS:
				case L6470::COMMAND_RESET_DEVICE:
				case L6470::COMMAND_SOFT_STOP:
				case L6470::COMMAND_HARD_STOP:
				case L6470::COMMAND_SOFT_HIZ:
				case L6470::COMMAND_HARD_HIZ:
				default:
					nBytes[i] = 0;
					break;
			}
		}
	}
}

void L6470::transfer() {
	std::vector<SPIBus::SPITransfer> transfers;

	// multiple transfers
	// 1) send commands
	uint8_t maxNBytes = 0;
	for (int i = 0; i < this->driverCount; i++) {
		transfers.emplace_back(&commands[i], nullptr, 1);
		// find maximum number of bytes to send
		maxNBytes = std::max(maxNBytes, nBytes[i]);
	}
	this->spiBus->transfer(transfers);

	if (maxNBytes == 0) {
		return;
	}

	// 2) send/receive values
	for (int j = maxNBytes - 1; j >= 0; --j) {
		transfers.clear();
		for (int i = 0; i < this->driverCount; i++) {
			transfers.emplace_back(&txValues[i].bytes[j], &rxValues[i].bytes[j], 1);

			/**
			 * Note: the original code that was here goes like this:
			 * - for each of the L6470s on the same bus, as indexed with 'i':
			 * - if the number of bytes to send is 0, just transfer a 0 and ignore the returned value
			 * - read the number of bytes we're going to send,
			 *  decrease by 1 (so we're counting from max-1 to 0), that's our OFFSET
			 * - get a uint8-type pointer to our (uint32-typed) value field, that's BP (Byte P-something?)
			 * - send the BP+OFFSET byte, save the return value RET
			 * - copy the RET value back to the same place we've taken our send byte from: BP+OFFSET
			 * - override the number of bytes to send with our calculated offset
			 *  (i.e. effectively just decreasing it by 1)
			 *
			 * Note 2: the original code was weird in few places (and by few I mean all of them):
			 * - it used the same space in the memory to both send and receive
			 * - and it used some casting trickery (aka the forbidden black magick)
			 *  to send/receive single bytes out of/into an uint32 value field
			 * - AND it did some very unclear pointer offset decrementing using
			 *  a variable that was tossed around like a dice in a casino
			 */
		}
		this->spiBus->transfer(transfers);
	}
}

// clear buffers after a transfer, will overwrite any results
void L6470::resetBuffers() {
	for (uint8_t i = 0; i < this->driverCount; i++) {
		this->commands[i] = 0;
		this->nBytes[i] = 0;
		this->txValues[i].fullValue = 0;
		this->rxValues[i].fullValue = 0;
	}
}

void L6470::runCommand(std::vector<uint8_t> commands, std::vector<uint32_t> values, OptUInt8 index) {
	this->setCommand(std::move(commands), index);
	this->setValues(std::move(values), index);
	this->setNBytes();
	this->transfer();
	this->resetBuffers();
}

void L6470::runCommand(uint8_t command, uint32_t value, OptUInt8 index) {
	this->runCommand(
		std::vector<uint8_t>(this->driverCount, command),
		std::vector<uint32_t>(this->driverCount, value),
		index
	);
}

void L6470::runCommand(std::vector<uint8_t> commands, OptUInt8 index) {
	this->setCommand(std::move(commands), index);
	this->setNBytes();
	this->transfer();
	this->resetBuffers();
}

void L6470::runCommand(uint8_t command, OptUInt8 index) {
	this->runCommand(std::vector<uint8_t>(this->driverCount, command), index);
}

void L6470::setParam(const L6470::RegisterAddress& param, std::vector<uint32_t> values, OptUInt8 index) {
	uint8_t command = static_cast<uint8_t>(L6470::COMMAND_SET_PARAM) | param;
	this->runCommand(std::vector<uint8_t>(this->driverCount, command), std::move(values), index);
}

void L6470::setParam(const L6470::RegisterAddress& param, uint32_t value, OptUInt8 index) {
	this->setParam(param, std::vector<uint32_t>(this->driverCount, value), index);
}

std::vector<uint32_t> L6470::getParam(const L6470::RegisterAddress& param, OptUInt8 index) {
	uint8_t command = static_cast<uint8_t>(L6470::COMMAND_GET_PARAM) | param;
	this->setCommand(command, index);
	this->setNBytes();
	this->transfer();
	auto retVal = this->getValues(index);
	this->resetBuffers();
	return retVal;
}

// ###############
// Private static methods - utility/conversion functions
// ###############

uint32_t L6470::accelerationCalc(float stepsPerSecPerSec) {
	// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is
	// 250ns (datasheet value) - 0x08A on boot.
	// Multiply desired steps/s/s by 0.068719476736 to get an appropriate value for this register.
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFE (0xFFF is reserved).
	auto acceleration = static_cast<uint32_t>(stepsPerSecPerSec * 0.068719476736f);
	if (acceleration > 0x00000FFE) {
		return 0x00000FFE;
	}
	return acceleration;
}

float L6470::accelerationParse(uint32_t stepsPerSecPerSec) {
	return static_cast<float>(stepsPerSecPerSec & 0x00000FFF) / 0.068719476736f;
}

uint32_t L6470::decelerationCalc(float stepsPerSecPerSec) {
	// The calculation for the DEC register is the same as for ACC. Value is 0x08A on boot.
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
	auto deceleration = static_cast<uint32_t>(stepsPerSecPerSec * 0.068719476736f);
	if (deceleration > 0x00000FFF) {
		return 0x00000FFF;
	}
	return deceleration;
}

float L6470::decelerationParse(uint32_t stepsPerSecPerSec) {
	return static_cast<float>(stepsPerSecPerSec & 0x00000FFF) / 0.068719476736f;
}

uint32_t L6470::minSpeedCalc(float stepsPerSec) {
	// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is
	// 250ns (datasheet value) - 0x000 on boot.
	// Multiply desired steps/s by 4.194304 to get an appropriate value for this register
	// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
	auto minSpeed = static_cast<uint32_t>(stepsPerSec * 4.194304f);
	if (minSpeed > MIN_SPEED_MIN_SPEED_MASK) {
		return MIN_SPEED_MIN_SPEED_MASK;
	}
	return minSpeed;
}

float L6470::minSpeedParse(uint32_t stepsPerSec) {
	return static_cast<float>(stepsPerSec & MIN_SPEED_MIN_SPEED_MASK) / 4.194304f;
}

uint32_t L6470::maxFsSpeedCalc(float stepsPerSec) {
	// The value in the MAX_SPD and FS_SPD registers is [(steps/s)*(tick)]/(2^-18) where tick is
	// 250ns (datasheet value) - 0x041 on boot.
	// Multiply desired steps/s by .065536 to get an appropriate value for this register
	// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
	auto speed = static_cast<uint32_t>(std::ceil(stepsPerSec * 0.065536f));
	if (speed > 0x000003FF) {
		return 0x000003FF;
	}
	return speed;
}

float L6470::maxFsSpeedParse(uint32_t stepsPerSec) {
	return static_cast<float>(stepsPerSec & 0x000003FF) / 0.065536f;
}

uint32_t L6470::speedCalc(float stepsPerSec) {
	auto speed = static_cast<uint32_t>(stepsPerSec * SPEED_SPS_TO_RAW);
	if (speed > SPEED_INT_MAX) {
		return SPEED_INT_MAX;
	}
	return speed;
}

float L6470::speedParse(uint32_t stepsPerSec) {
	return static_cast<float>(stepsPerSec & SPEED_INT_MAX) / SPEED_SPS_TO_RAW;
}
