#pragma once

#include <GPIOPin.hpp>
#include <SPIBus.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

/**
 * Interface to control multiple L6470 chips daisy chained on a single CS line.
 *
 * L6470s are ordered with index 0 being the last chip in the chain and driverCount-1 the first.
 *
 * All methods accepting an optional `index` parameter allow targeting a specific driver in the chain,
 * with the default value of `nullopt` meaning "all".
 *
 * All getters returning a vector return a vector with size of 1 if an index is applicable and given.
 */
class L6470: public std::enable_shared_from_this<L6470> {
public:
	/**
	 * A shared_ptr alias (use as L6470::SharedPtr)
	 */
	using SharedPtr = std::shared_ptr<L6470>;

	/**
	 * A shared_ptr to a constant alias (use as L6470::ConstSharedPtr)
	 */
	using ConstSharedPtr = std::shared_ptr<const L6470>;

	// Forward-declare enums (defined at the bottom of this file)
	enum Action : uint8_t;
	enum Command : uint8_t;
	enum Direction : uint8_t;
	enum MotorStatus : uint16_t;
	enum OscillatorMode : uint8_t;
	enum PinFunction : uint8_t;
	enum PWMDivisor : uint8_t;
	enum PWMMultiplier : uint8_t;
	enum RegisterAddress : uint8_t;
	enum SlewRate : uint8_t;
	enum StepMode : uint8_t;
	enum SwitchMode : uint8_t;
	enum SyncPulseDivisor : uint8_t;

	/**
	 * The STATUS register, as returned by the driver, divided into easily-accessible fields.
	 *
	 * Based on the datasheet section 9.1.22 (page 55).
	 */
	union Status {
		struct {
			/**
			 * Indicates that the bridges are in high impedance state.
			 * Any motion command makes the device exit from HiZ state.
			 * Active high.
			 */
			uint8_t hiZ : 1;

			/**
			 * Reflects the ~BUSY pin state, i.e. when a constant speed, positioning or motion command is under execution.
			 * Active low.
			 */
			uint8_t busy : 1;

			/**
			 * Indicates the SW input status.
			 * Note that in the typical application example it is shorted to VDD.
			 * Low for open, high for closed.
			 */
			uint8_t swInput : 1;

			/**
			 * Reports a switch turn on event (falling edge on SW input).
			 * Active high.
			 */
			uint8_t swInputEvent : 1;

			/**
			 * Indicates the current motor direction.
			 * 1 - forward, 0 - reverse.
			 */
			uint8_t direction : 1;

			/**
			 * Indicates the current motor status, see @ref L6470::MotorStatus for values.
			 */
			MotorStatus motorStatus : 2;

			/**
			 * Indicates that the command received cannot be preformed.
			 * This flag is raised e.g. by writing to the STATUS register.
			 * Active high.
			 */
			uint8_t lastCmdNotPerformed : 1;

			/**
			 * Indicates that the command received does not exist at all.
			 * Active high.
			 */
			uint8_t lastCmdWrong : 1;

			/**
			 * Indicates an undervoltage lockout or reset event (power-up included).
			 * Active low.
			 */
			uint8_t undervoltageLockout : 1;

			/**
			 * Indicates thermal warning event.
			 * Active low.
			 */
			uint8_t thermalWarning : 1;

			/**
			 * Indicates thermal shutdown event.
			 * Active low.
			 */
			uint8_t thermalShutdown : 1;

			/**
			 * Indicates overcurrent detection event.
			 * Active low.
			 */
			uint8_t overcurrent : 1;

			/**
			 * Indicates a stall was detected on bridge A.
			 * Active low.
			 */
			uint8_t stepLossA : 1;

			/**
			 * Indicates a stall was detected on bridge B.
			 * Active low.
			 */
			uint8_t stepLossB : 1;

			/**
			 * Indicates that the device is working in Step-clock mode.
			 * In this case the step-clock signal should be provided through the STCK input pin.
			 * Active high.
			 */
			uint8_t stepClockActive : 1;
		} __attribute__((packed));

		uint16_t rawStatus;
	};

	using OptUInt8 = std::optional<uint8_t>;

	L6470(
		uint8_t driverCount,
		SPIBus::SharedPtr spiBus,
		GPIOPin::SharedPtr resetPin,
		GPIOPin::SharedPtr busyPin = nullptr
	);

	/**
	 * Get the status of a driver, clearing warnings/error states.
	 *
	 * Fetches and returns the 16-bit values in the STATUS registers via the GET_STATUS command.
	 * Resets any warning flags and exits any error states.
	 * Using readStatus() does not clear these values.
	 *
	 * @param index The index of the driver to read the status of, none for all.
	 *
	 * @return Vector of statuses of the L6470s.
	 */
	std::vector<Status> getStatus(OptUInt8 index = std::nullopt);

	/**
	 * Get the status of a driver, without clearing warnings/error states.
	 *
	 * Fetches and returns the 16-bit values in the STATUS registers via the GET_PARAM command.
	 * Doesn't clear any warning flags and doesn't alter error states.
	 * Using getStatus() clears these values.
	 *
	 * @param index The index of the driver to read the status of, none for all.
	 *
	 * @return A vector of statuses of the L6470s.
	 */
	std::vector<Status> readStatus(OptUInt8 index = std::nullopt);

	/**
	 * Check whether a driver (or all of them) is busy.
	 *
	 * @param index The index of the driver to check, none for all.
	 *
	 * @return A vector of busy states (true if the specified index is busy).
	 */
	std::vector<bool> isBusy(OptUInt8 index = std::nullopt);

	/**
	 * An aggregating check for the business of all the drivers.
	 *
	 * @return True if any of the drivers is busy.
	 */
	bool isAnyBusy();

	/**
	 * @section Operational commands.
	 */

	/**
	 * Get the absolute position of a motor, in steps from the HOME position.
	 *
	 * Returns the content of the ABS_POS register, which is a signed 22-bit number indicating the number of steps
	 * the motor has traveled from the HOME position.
	 * HOME is defined by zeroing this register, and it is zero on startup.
	 *
	 * @param index The index of the motor to get the position of, none for all.
	 *
	 * @return A vector of positions.
	 */
	std::vector<int32_t> getPosition(OptUInt8 index = std::nullopt);

	/**
	 * Get the speed of a motor, in steps per second as a float.
	 *
	 * Returns the content of the SPEED register.
	 *
	 * @param index The index of the motor to get the speed of, none for all.
	 *
	 * @return A vector of speeds.
	 */
	std::vector<float> getSpeed(OptUInt8 index = std::nullopt);

	/**
	 * Get the currently set motor direction.
	 *
	 * Returns the direction as read from the STATUS_DIR bit of the STATUS register.
	 *
	 * @param index The index of the motor to get the direction of, none for all.
	 *
	 * @return A vector of directions.
	 */
	std::vector<Direction> getDirection(OptUInt8 index = std::nullopt);

	/**
	 * Get the MARK position of a motor, in steps from the HOME position.
	 *
	 * Just like getPosition(), but for MARK.
	 *
	 * @param index The index of the motor to get the MARK position of, none for all.
	 *
	 * @return A vector of MARK positions.
	 */
	std::vector<int32_t> getMarkPosition(OptUInt8 index = std::nullopt);

	/**
	 * Set one or more motors spinning in a specific direction.
	 *
	 * Maximum speed and minimum speed are set by the @ref setMaxSpeed() and @ref setMinSpeed() methods.
	 * Exceeding the full-speed value (@ref setFullSpeed()) will switch the device into full-step mode.
	 *
	 * @param speeds The speeds at which to spin the respective motors. Valid range is [0; 15625].
	 * @param directions The directions to spin the motors.
	 * @param index The index of the motor to spin, none for all.
	 */
	void run(std::vector<float> speeds, std::vector<Direction> directions, OptUInt8 index = std::nullopt);

	/**
	 * Alias for @ref run() for running all the specified motors with the same parameters.
	 *
	 * @param speed The speed at which to spin the motors. Valid range is [0; 15625].
	 * @param direction The direction to spin the motors.
	 * @param index The index of the motor to spin, none for all.
	 */
	void run(float speed, const Direction& direction, OptUInt8 index = std::nullopt);

	/**
	 * Send a motor a given number of steps in a given direction.
	 *
	 * The motor will accelerate according the acceleration and deceleration curves,
	 * and will run at max-speed (as set by @ref setMaxSpeed()).
	 * Stepping mode will adhere to full-speed value (set by @ref setFullSpeed()), as well.
	 *
	 * @param numSteps The number of steps to do. Max value is 0x3FFFFF (22 bits).
	 * @param directions The direction to spin the motor.
	 * @param index The index of the motor to move, none for all.
	 */
	void move(std::vector<uint32_t> numSteps, std::vector<Direction> directions, OptUInt8 index = std::nullopt);

	void move(uint32_t numSteps, const Direction& direction, OptUInt8 index = std::nullopt);

	/**
	 * Send a motor to an absolute position in the shortest possible fashion.
	 *
	 * Movement parameters (acceleration, deceleration etc) are the same as in @ref move().
	 *
	 * @param positions The position to move the motor to.
	 * @param index The index of the motor to move, none for all.
	 */
	void goTo(std::vector<int32_t> positions, OptUInt8 index = std::nullopt);

	void goTo(int32_t position, OptUInt8 index = std::nullopt);

	/**
	 * Send a motor to an absolute position with user constrained rotational direction.
	 *
	 * Movement parameters (acceleration, deceleration etc) are the same as in @ref goTo() and @ref move().
	 *
	 * @param positions The position to move the motor to.
	 * @param directions The direction to spin the motor.
	 * @param index The index of the motor to move, none for all.
	 */
	void goToDirection(
		std::vector<int32_t> positions,
		std::vector<Direction> directions,
		OptUInt8 index = std::nullopt
	);

	void goToDirection(int32_t position, const Direction& direction, OptUInt8 index = std::nullopt);

	/**
	 * Set a motor running in the given direction until a falling edge is detected on the SW pin.
	 *
	 * Depending on the SW_MODE bit in CONFIG (@ref setSwitchMode()),
	 * either a hard stop or a soft stop is performed at the falling edge,
	 * and depending on the value of `action` the value in the ABS_POS register
	 * is either reset to 0 or copied into the MARK register.
	 *
	 * @param speeds The speed at which to spin the motor.
	 * @param directions The direction to spin the motor.
	 * @param actions The action to do on the ABS_POS register.
	 * @param index The index of the motor to spin, none for all.
	 */
	void goUntil(
		std::vector<float> speeds,
		std::vector<Direction> directions,
		std::vector<Action> actions,
		OptUInt8 index = std::nullopt
	);

	void goUntil(float speed, const Direction& direction, const Action& action, OptUInt8 index = std::nullopt);

	/**
	 * Set a motor running at a minimal speed in the given direction until a falling edge is detected on the SW pin.
	 *
	 * The motion is produced at the higher of two speeds: the value in MIN_SPEED (@ref setMinSpeed()) or 5 steps/s.
	 * All other behavior is the same as in @ref goUntil().
	 *
	 * @param directions The direction to spin the motor.
	 * @param actions The action to do on the ABS_POS register.
	 * @param index The index of the motor to spin, none for all.
	 */
	void releaseSw(std::vector<Direction> directions, std::vector<Action> actions, OptUInt8 index = std::nullopt);

	void releaseSw(const Direction& direction, const Action& action, OptUInt8 index = std::nullopt);

	/**
	 * Put a driver in the external step clocking mode with a set direction.
	 *
	 * When active, pin 25, STCK, becomes the step clock for the device, and steps it in the given direction.
	 * Motion commands (run(), move(), etc) will cause the device to exit step clocking mode.
	 *
	 * @param directions The direction to spin the driver's motor.
	 * @param index The index of the driver to put in the external step clocking mode, none for all.
	 */
	void stepClock(std::vector<Direction> directions, OptUInt8 index = std::nullopt);

	void stepClock(const Direction& direction, OptUInt8 index = std::nullopt);

	/**
	 * Send a motor to its HOME position.
	 *
	 * Equivalent to `goTo(0)`, but requires less time to send.
	 * Note that no direction is provided - motion occurs through shortest path.
	 * If a direction is required, use @ref goToDirection().
	 *
	 * @param index The index of the motor to move, none for all.
	 */
	void goHome(OptUInt8 index = std::nullopt);

	/**
	 * Send a motor to its MARK position.
	 *
	 * Equivalent to `goTo(MARK)`, but requires less time to send.
	 * Note that no direction is provided - motion occurs through shortest path.
	 * If a direction is required, use @ref goToDirection().
	 *
	 * @param index The index of the motor to move, none for all.
	 */
	void goMark(OptUInt8 index = std::nullopt);

	/**
	 * Set the current saved MARK position of a motor.
	 *
	 * @param newMark The new MARK position to set.
	 * @param index The index of the motor to set the MARK of, none for all.
	 */
	void setMark(int32_t newMark, OptUInt8 index = std::nullopt);

	/**
	 * Set the current saved position of a motor.
	 *
	 * This command is invalid if the motor is running.
	 *
	 * @param newPosition The new position to set.
	 * @param index The index of the motor to set the position of, none for all.
	 */
	void setPosition(int32_t newPosition, OptUInt8 index = std::nullopt);

	/**
	 * Set the ABS_POS register of a motor to 0, effectively declaring the current position to be HOME.
	 *
	 * @param index The index of the motor to reset the position of, none for all.
	 */
	void resetPosition(OptUInt8 index = std::nullopt);

	/**
	 * Reset a driver to power up conditions.
	 *
	 * Equivalent to toggling the STBY pin or cycling power, but with the ability to only reset one device.
	 *
	 * @param index The index of the driver to reset, none for all.
	 */
	void resetDevice(OptUInt8 index = std::nullopt);

	/**
	 * Reset all chained drivers by cycling the reset (STBY) pin.
	 */
	void resetDeviceHw();

	/**
	 * Bring a motor to a halt using the deceleration curve.
	 *
	 * @param index The index of the motor to stop, none for all.
	 */
	void softStop(OptUInt8 index = std::nullopt);

	/**
	 * Stop a motor with infinite deceleration.
	 *
	 * @param index The index of the motor to stop, none for all.
	 */
	void hardStop(OptUInt8 index = std::nullopt);

	/**
	 * Decelerate a motor and put the bridges of a driver in Hi-Z state.
	 *
	 * @param index The index of the motor/driver to stop and set in Hi-Z, none for all.
	 */
	void softHiZ(OptUInt8 index = std::nullopt);

	/**
	 * Put the bridges of a driver in Hi-Z state immediately with no deceleration.
	 *
	 * @param index The index of the motor/driver to stop and set in Hi-Z, none for all.
	 */
	void hardHiZ(OptUInt8 index = std::nullopt);

	/**
	 * @section Configuration commands
	 */

	/**
	 * Setup the SYNC/BUSY pin to be either SYNC or BUSY and to a desired ticks per step level.
	 *
	 * @param pinFunction The new pin function.
	 * @param syncSteps The number of sync pulses per step.
	 * @param index The index of the driver to apply the setting to, none for all.
	 */
	void configSyncPin(
		const PinFunction& pinFunction,
		const SyncPulseDivisor& syncSteps,
		OptUInt8 index = std::nullopt
	);

	/**
	 * The L6470 chip supports microstepping for a smoother ride. This function
	 * provides an easy front end for changing the microstepping mode.
	 *
	 * @param stepMode The step mode
	 * @param index The index of the driver to apply the setting to, none for all.
	 */
	void configStepMode(const StepMode& stepMode, OptUInt8 index = std::nullopt);

	std::vector<StepMode> getStepMode(OptUInt8 index = std::nullopt);

	/**
	 * This is the maximum speed the L6470 will attempt to produce.
	 *
	 * @param stepsPerSecond The maximum speed, in steps per second.
	 * @param index The index of the motor to set the maximum speed of, none for all.
	 */
	void setMaxSpeed(float stepsPerSecond, OptUInt8 index = std::nullopt);

	std::vector<float> getMaxSpeed(OptUInt8 index = std::nullopt);

	/**
	 * Set the minimum speed allowable in the system.
	 *
	 * This is the speed a motion starts with;
	 * it will then ramp up to the designated speed or the max speed, using the acceleration profile.
	 *
	 * @param stepsPerSecond The minimum speed, in steps per second.
	 * @param index The index of the motor to set the minimum speed of, none for all.
	 */
	void setMinSpeed(float stepsPerSecond, OptUInt8 index = std::nullopt);

	std::vector<float> getMinSpeed(OptUInt8 index = std::nullopt);

	/**
	 * Above this threshold, the L6470 will cease microstepping and go to full-step mode.
	 *
	 * @param stepsPerSecond The full speed threshold, in steps per second.
	 * @param index The index of the driver/motor to set the full speed of, none for all.
	 */
	void setFullSpeed(float stepsPerSecond, OptUInt8 index = std::nullopt);

	std::vector<float> getFullSpeed(OptUInt8 index = std::nullopt);

	/**
	 * Set the acceleration rate, in steps per second per second.
	 *
	 * Any value larger than 29802 (0x0FFE) will disable acceleration,
	 * putting the chip in an "infinite" acceleration mode.
	 *
	 * @param acceleration The acceleration rate, in steps per second per second.
	 * @param index The index of the driver/motor to set the acceleration rate of, none for all.
	 */
	void setAcceleration(float acceleration, OptUInt8 index = std::nullopt);

	std::vector<float> getAcceleration(OptUInt8 index = std::nullopt);

	/**
	 * Set the deceleration rate, in steps per second per second.
	 *
	 * Same rules as for @ref setAcceleration().
	 *
	 * @param deceleration The deceleration rate, in steps per second per second.
	 * @param index The index of the driver/motor to set the deceleration rate of, none for all.
	 */
	void setDeceleration(float deceleration, OptUInt8 index = std::nullopt);

	std::vector<float> getDeceleration(OptUInt8 index = std::nullopt);

	/**
	 * Enable or disable the low-speed optimization option.
	 *
	 * With this option enabled, motion starts from 0 instead of MIN_SPEED (@ref setMinSpeed())
	 * and low-speed optimization keeps the driving sine wave prettier than normal until MIN_SPEED is reached.
	 *
	 * @param enable Whether to enable the low-speed optimization.
	 * @param index The index of the driver/motor to enable/disable the LSO of, none for all.
	 */
	void setLowSpeedOptimization(bool enable, OptUInt8 index = std::nullopt);

	std::vector<bool> getLowSpeedOptimization(OptUInt8 index = std::nullopt);

	/**
	 * Set the over-current threshold.
	 *
	 * The available range is from 375 mA to 6 A, in steps of 375 mA.
	 *
	 * If an overcurrent event is detected, it is noted in the Status (@ref getStatus() and @ref readStatus()).
	 * The driver's behavior on an overcurrent event is controlled by the @ref setOCShutdown().
	 *
	 * @param threshold The over-current threshold to set, in milliamperes.
	 * @param index The index of the driver to set the OC threshold of, none for all.
	 */
	void setOCThreshold(float threshold, OptUInt8 index = std::nullopt);

	std::vector<float> getOCThreshold(OptUInt8 index = std::nullopt);

	/**
	 * Set the stall detection threshold.
	 *
	 * The available range is from 31.25 mA to 4 A, in steps of 31.25 mA.
	 *
	 * If a stall event is detected, it is noted in the Status (@ref getStatus() and @ref readStatus()).
	 *
	 * @param threshold The stall threshold to set, in milliamperes.
	 * @param index The index of the driver to set the stall threshold of, none for all.
	 */
	void setStallThreshold(float threshold, OptUInt8 index = std::nullopt);

	std::vector<float> getStallThreshold(OptUInt8 index = std::nullopt);

	/**
	 * The next few functions are all breakouts for individual options within the single register CONFIG.
	 * We'll read CONFIG, blank some bits, then OR in the new value.
	 */

	/**
	 * Set a multiplier and divisor for the PWM frequency used when microstepping.
	 *
	 * See datasheet for more details;
	 * It's not clear what the frequency being multiplied/divided here is,
	 * but it is clearly *not* the actual clock frequency.
	 *
	 * @param divisor The PWM frequency divisor.
	 * @param multiplier The PWM frequency multiplier.
	 * @param index The index of the driver to set the PWM frequency of, none for all.
	 */
	void setPWMFrequency(const PWMDivisor& divisor, const PWMMultiplier& multiplier, OptUInt8 index = std::nullopt);

	std::vector<PWMDivisor> getPWMFrequencyDivisor(OptUInt8 index = std::nullopt);

	std::vector<PWMMultiplier> getPWMFrequencyMultiplier(OptUInt8 index = std::nullopt);

	/**
	 * Set the slew rate of the output, in V/us.
	 *
	 * @param slewRate The slew rate to set.
	 * @param index The index of the driver to set the slew rate of, none for all.
	 */
	void setSlewRate(const SlewRate& slewRate, OptUInt8 index = std::nullopt);

	std::vector<SlewRate> getSlewRate(OptUInt8 index = std::nullopt);

	/**
	 * Set whether a driver should shutdown on an over-current event.
	 *
	 * Set the overcurrent threshold value with @ref setOCThreshold().
	 *
	 * @param enabled Whether to enable the driver's overcurrent shutdown.
	 * @param index The index of the driver to enable/disable overcurrent shutdown of, none for all.
	 */
	void setOCShutdown(bool enabled, OptUInt8 index = std::nullopt);

	std::vector<bool> getOCShutdown(OptUInt8 index = std::nullopt);

	/**
	 * Set whether to enable motor voltage compensation.
	 *
	 * This is not at all straightforward - check out section 7 "Phase current control" (page 34) of the datasheet.
	 *
	 * @param enabled Whether to enable the motor voltage compensation.
	 * @param index The index of the driver to enable/disable the voltage compensation of, none for all.
	 */
	void setVoltageCompensation(bool enabled, OptUInt8 index = std::nullopt);

	std::vector<bool> getVoltageCompensation(OptUInt8 index = std::nullopt);

	/**
	 * Set the switch input mode of a driver.
	 *
	 * The switch input can either hard-stop the driver _or_ activate an interrupt.
	 *
	 * @param switchMode The switch mode to set.
	 * @param index The index of the driver to set the switch mode of, none for all.
	 */
	void setSwitchMode(const SwitchMode& switchMode, OptUInt8 index = std::nullopt);

	std::vector<SwitchMode> getSwitchMode(OptUInt8 index = std::nullopt);

	/**
	 * Set the oscillator mode of a driver.
	 *
	 * There are a number of clock options for L6470 - it can be configured to accept a clock,
	 * drive a crystal or resonator, and pass or not pass the clock signal downstream.
	 *
	 * Theoretically, you can use pretty much any frequency you want to drive it;
	 * practically, this library assumes it's being driven at 16MHz.
	 * Also, the device will use these bits to set the math used to figure out steps per second and stuff like that.
	 *
	 * @param oscillatorMode The oscillator mode to set.
	 * @param index The index of the driver to set the oscillator mode of, none for all.
	 */
	void setOscillatorMode(const OscillatorMode& oscillatorMode, OptUInt8 index = std::nullopt);

	std::vector<OscillatorMode> getOscillatorMode(OptUInt8 index = std::nullopt);

	/**
	 * @section KVAL
	 *
	 */

	/**
	 * Set the acceleration K_VAL value of a driver.
	 *
	 * The K_VAL is part of the voltage amplitude regulation and it is, effectively, the output voltage multiplier.
	 *
	 * The output voltage is calculated as `K_VAL * V_S / 256` (see datasheet section 9.1.10, page 44).
	 *
	 * Note from original developer of the SparkFun AutoDriver library:
	 * > The KVAL registers are... weird.
	 * > I don't entirely understand how they differ from the microstepping,
	 * > but if you have trouble getting the motor to run, tweaking KVAL has proven effective in the past.
	 *
	 * @param kval The K_VAL value.
	 * @param index The index of the driver to set the K_VAL of, none for all.
	 */
	void setAccelerationKVAL(uint8_t kval, OptUInt8 index = std::nullopt);

	std::vector<uint8_t> getAccelerationKVAL(OptUInt8 index = std::nullopt);

	/**
	 * Set the deceleration K_VAL value of a driver.
	 *
	 * See @ref setAccelerationKVAL() for notes and details.
	 *
	 * @param kval The K_VAL value.
	 * @param index The index of the driver to set the K_VAL of, none for all.
	 */
	void setDecelerationKVAL(uint8_t kval, OptUInt8 index = std::nullopt);

	std::vector<uint8_t> getDecelerationKVAL(OptUInt8 index = std::nullopt);

	/**
	 * Set the constant speed K_VAL value of a driver.
	 *
	 * See @ref setAccelerationKVAL() for notes and details.
	 *
	 * @param kval The K_VAL value.
	 * @param index The index of the driver to set the K_VAL of, none for all.
	 */
	void setRunKVAL(uint8_t kval, OptUInt8 index = std::nullopt);

	std::vector<uint8_t> getRunKVAL(OptUInt8 index = std::nullopt);

	/**
	 * Set the holding (stopped) K_VAL value of a driver.
	 *
	 * See @ref setAccelerationKVAL() for notes and details.
	 *
	 * @param kval The K_VAL value.
	 * @param index The index of the driver to set the K_VAL of, none for all.
	 */
	void setHoldKVAL(uint8_t kval, OptUInt8 index = std::nullopt);

	std::vector<uint8_t> getHoldKVAL(OptUInt8 index = std::nullopt);

	/**
	 * @section Communication commands
	 *
	 * Provide low level access to functions that setup and transfer command and value buffers to the L6470s
	 */

	/**
	 * Set a command to be transferred to a driver.
	 *
	 * @param commands The command to set, see @ref Command
	 * @param index The index of the driver to set the command for, none for all.
	 */
	void setCommand(std::vector<uint8_t> commands, OptUInt8 index = std::nullopt);

	void setCommand(uint8_t command, OptUInt8 index = std::nullopt);

	/**
	 * Set values (command's argument) to be transferred to a driver.
	 *
	 * @param values The value to set.
	 * @param index The index of the driver to set the command for, none for all.
	 */
	void setValues(std::vector<uint32_t> values, OptUInt8 index = std::nullopt);

	void setValues(uint32_t value, OptUInt8 index = std::nullopt);

	/**
	 * Get values previously read from the drivers.
	 *
	 * @param index The index of the driver that the value was read from, none for all.
	 */
	std::vector<uint32_t> getValues(OptUInt8 index = std::nullopt);

	/**
	 * Set the length of a driver's to-be-sent command argument to a specific value.
	 *
	 * @param n The command argument length, in bytes.
	 * @param index The index of the driver to set the command argument length for, none for all.
	 */
	void setNBytes(uint8_t n, OptUInt8 index = std::nullopt);

	/**
	 * Automatically set the lengths of the to-be-sent command parameters for all the drivers in the chain.
	 */
	void setNBytes();

	/**
	 * Transfer currently queued commands.
	 *
	 * Sets the values available via the @ref getValues().
	 */
	void transfer();

	/**
	 * Clear the transfer buffers (transmits/receives/commands/lengths).
	 *
	 * Use this after a transfer.
	 * This will overwrite any results.
	 */
	void resetBuffers();

	/**
	 * Run a command on a driver.
	 *
	 * @param commands The command to run, see @ref Command.
	 * @param index The index of the driver to run the command on, none for all.
	 */
	void runCommand(std::vector<uint8_t> commands, OptUInt8 index = std::nullopt);

	void runCommand(uint8_t command, OptUInt8 index = std::nullopt);

	/**
	 * Run a command with an argument on a driver.
	 *
	 * @param commands The command to run, see @ref Command.
	 * @param values The command's argument value.
	 * @param index The index of the driver to run the command on, none for all.
	 */
	void runCommand(std::vector<uint8_t> commands, std::vector<uint32_t> values, OptUInt8 index = std::nullopt);

	void runCommand(uint8_t command, uint32_t value, OptUInt8 index = std::nullopt);

	/**
	 * Set a parameter on a driver.
	 *
	 * @param param The parameter to set, see @ref RegisterAddress.
	 * @param values The parameter's values.
	 *  If index is not given, must be as long as the number of drivers in chain, else only the first value is taken.
	 * @param index The index of the driver to set the parameter on, none for all.
	 */
	void setParam(const RegisterAddress& param, std::vector<uint32_t> values, OptUInt8 index = std::nullopt);

	void setParam(const RegisterAddress& param, uint32_t value, OptUInt8 index = std::nullopt);

	/**
	 * Get a parameter's value from a driver.
	 *
	 * @param param The parameter to get, see @ref RegisterAddress.
	 * @param index The index of the driver to get the parameter from, none for all.
	 */
	std::vector<uint32_t> getParam(const RegisterAddress& param, OptUInt8 index = std::nullopt);

	/**
	 * Create a SharedPtr instance of the L6470.
	 *
	 * Usage: `L6470::makeShared(args...)`.
	 * See constructor (@ref L6470::L6470()) for details.
	 */
	template<typename ... Args>
	static L6470::SharedPtr makeShared(Args&& ... args) {
		return std::make_shared<L6470>(std::forward<Args>(args) ...);
	}

private:
	static constexpr uint32_t TWO_COMP_22B_SIGN_BIT = 0x00200000;
	static constexpr uint32_t TWO_COMP_22B_COMP_FIX = 0xFFC00000;
	/**
	 * The raw fixed-point speed is a 20-bit value.
	 *
	 * Based on datasheet section 9.1.4, page 42.
	 */
	static constexpr uint32_t SPEED_INT_MAX = 0xFFFFF;
	/**
	 * The maximum speed in steps per seconds.
	 *
	 * Based on datasheet section 9.1.4, page 42.
	 */
	static constexpr float SPEED_SPS_MAX = 15625.0f;
	/**
	 * Speed conversion value, steps-per-seconds to raw fixed point as used by the driver.
	 *
	 * Based on datasheet section 9.1.4, page 42.
	 * When issuing RUN command, the raw 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 250ns.
	 * (250ns / 1s) / 2^(-28) ~= 67.108864
	 */
	static constexpr float SPEED_SPS_TO_RAW = 67.108864f;
	/**
	 * The maximum "distance" possible to move in steps (for @ref move()).
	 *
	 * Based on datasheet section 9.2.7, page 61.
	 */
	static constexpr uint32_t STEPS_INT_MAX = 0x3FFFFF;
	/**
	 * The maximum possible position (2^21 - 1).
	 *
	 * Based on datasheet section 9.1.1, page 41.
	 */
	static constexpr int32_t POSITION_MAX = 0X1FFFFF;
	/**
	 * The minimum possible position (-2^21).
	 *
	 * Based on datasheet section 9.1.1, page 41.
	 */
	static constexpr int32_t POSITION_MIN = -0x200000;
	/**
	 * The sync pin settings mask for the STEP_MODE register.
	 *
	 * Based on datasheet section 9.1.19, page 47.
	 */
	static constexpr uint8_t STEP_MODE_SYNC_MASK = 0xF0;
	/**
	 * The step mode mask for the STEP_MODE register.
	 *
	 * Based on datasheet section 9.1.19, page 47.
	 */
	static constexpr uint8_t STEP_MODE_STEP_SEL_MASK = 0x07;
	/**
	 * The pin function mask for the STEP_MODE register.
	 *
	 * Based on datasheet section 9.1.19, page 47.
	 */
	static constexpr uint8_t STEP_MODE_PIN_FUNC_MASK = 0x80;
	/**
	 * The sync pin step mode mask for the STEP_MODE register.
	 *
	 * Based on datasheet section 9.1.19, page 47.
	 */
	static constexpr uint8_t STEP_MODE_SYNC_STEP_MASK = 0x70;
	/**
	 * The minimum value for MAX_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.7, page 43.
	 */
	static constexpr float MAX_SPEED_MIN = 15.25f;
	/**
	 * The maximum value for MAX_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.7, page 43.
	 */
	static constexpr float MAX_SPEED_MAX = 15610.0f;
	/**
	 * The minimum value for MIN_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.8, page 43.
	 */
	static constexpr float MIN_SPEED_MIN = 0.0f;
	/**
	 * The maximum value for MIN_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.8, page 43.
	 */
	static constexpr float MIN_SPEED_MAX = 976.3f;
	/**
	 * The low speed option mask for the MIN_SPEED register.
	 *
	 * Based on datasheet section 9.1.8, page 43.
	 */
	static constexpr uint16_t MIN_SPEED_LSPD_MASK = 0x1000;
	/**
	 * The minimum speed mask for the MIN_SPEED register.
	 *
	 * Based on datasheet section 9.1.8, page 43.
	 */
	static constexpr uint16_t MIN_SPEED_MIN_SPEED_MASK = 0x0FFF;
	/**
	 * The minimum value for FULL_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.9, page 44.
	 */
	static constexpr float FULL_SPEED_MIN = 7.63f;
	/**
	 * The maximum value for FULL_SPEED possible to set.
	 *
	 * Based on datasheet section 9.1.9, page 44.
	 */
	static constexpr float FULL_SPEED_MAX = 15625.0f;

	static constexpr int PWM_MULTIPLIER_SHIFT = 10;
	static constexpr int PWM_DIVISOR_SHIFT = 13;
	static constexpr int SLEW_RATE_SHIFT = 8;
	static constexpr uint16_t STATUS_DIR_MASK = 0x0010;

	// Disable bridges shutdown on overcurrent detection
	static constexpr uint8_t OC_SHUTDOWN_DISABLED = 0x00;
	// Enable bridges shutdown on overcurrent detection
	static constexpr uint8_t OC_SHUTDOWN_ENABLED = 0x80;
	// Disable motor voltage compensation
	static constexpr uint8_t VOLTAGE_COMP_DISABLED = 0x00;
	// Enable motor voltage compensation
	static constexpr uint8_t VOLTAGE_COMP_ENABLED = 0x20;

	static constexpr uint16_t STATUS_MASK_BUSY = 0x0002;

	uint8_t driverCount;
	SPIBus::SharedPtr spiBus;
	GPIOPin::SharedPtr csPin;
	GPIOPin::SharedPtr resetPin;
	GPIOPin::SharedPtr busyPin;

	// Transfer buffers for daisy-chained SPI messages
	union TransferUnit {
		uint32_t fullValue;
		uint8_t bytes[4];
	};

	std::vector<uint8_t> commands;
	std::vector<uint8_t> nBytes;
	std::vector<TransferUnit> txValues;
	std::vector<TransferUnit> rxValues;

	template<typename T, typename MapperOperator>
	std::vector<T> getParamsMapped(const RegisterAddress& param, MapperOperator mapper, OptUInt8 index = std::nullopt) {
		auto rawValues = this->getParam(param, index);
		std::vector<T> mappedValues;
		std::transform(rawValues.begin(), rawValues.end(), std::back_inserter(mappedValues), mapper);
		return mappedValues;
	}

	static uint32_t accelerationCalc(float stepsPerSecPerSec);

	static float accelerationParse(uint32_t stepsPerSecPerSec);

	static uint32_t decelerationCalc(float stepsPerSecPerSec);

	static float decelerationParse(uint32_t stepsPerSecPerSec);

	static uint32_t minSpeedCalc(float stepsPerSec);

	static float minSpeedParse(uint32_t stepsPerSec);

	static uint32_t maxFsSpeedCalc(float stepsPerSec);

	static float maxFsSpeedParse(uint32_t stepsPerSec);

	static uint32_t speedCalc(float stepsPerSec);

	static float speedParse(uint32_t stepsPerSec);
};

enum L6470::Direction : uint8_t {
	DIRECTION_FWD = 0x01,
	DIRECTION_REV = 0x00,
};

enum L6470::Action : uint8_t {
	// Reset the ABS_POS register
	ACTION_RESET_ABSPOS = 0x00,
	// Copy the ABS_POS register into the MARK register
	ACTION_COPY_ABSPOS = 0x08,
};

/**
 * The !BUSY/SYNC pin can be configured to be low when the chip is executing a command,
 * _or_ to output a pulse on each full step clock (with some divisor).
 */
enum L6470::PinFunction : uint8_t {
	PIN_FUNCTION_BUSY = 0x00,
	PIN_FUNCTION_SYNC = 0x80,
};

/**
 * Divisors for SYNC pulse outputs
 */
enum L6470::SyncPulseDivisor : uint8_t {
	// two per full step
	SYNC_DIVISOR_FS_2 = 0x00,
	// one per full step
	SYNC_DIVISOR_FS = 0x10,
	// one per two full steps
	SYNC_DIVISOR_2FS = 0x20,
	// one per four full steps
	SYNC_DIVISOR_4FS = 0x30,
	// one per eight full steps
	SYNC_DIVISOR_8FS = 0x40,
	// one per 16 full steps
	SYNC_DIVISOR_16FS = 0x50,
	// one per 32 full steps
	SYNC_DIVISOR_32FS = 0x60,
	// one per 64 full steps
	SYNC_DIVISOR_64FS = 0x70,
};

/**
 * Number of microsteps per full step
 */
enum L6470::StepMode : uint8_t {
	STEP_MODE_FS = 0x00,
	STEP_MODE_2 = 0x01,
	STEP_MODE_4 = 0x02,
	STEP_MODE_8 = 0x03,
	STEP_MODE_16 = 0x04,
	STEP_MODE_32 = 0x05,
	STEP_MODE_64 = 0x06,
	STEP_MODE_128 = 0x07,
};

enum L6470::PWMMultiplier : uint8_t {
	PWM_MULT_0_625 = 0x00,
	PWM_MULT_0_75 = 0x01,
	PWM_MULT_0_875 = 0x02,
	PWM_MULT_1 = 0x03,
	PWM_MULT_1_25 = 0x04,
	PWM_MULT_1_5 = 0x05,
	PWM_MULT_1_75 = 0x06,
	PWM_MULT_2 = 0x07,
};

enum L6470::PWMDivisor : uint8_t {
	PWM_DIV_1 = 0x00,
	PWM_DIV_2 = 0x01,
	PWM_DIV_3 = 0x02,
	PWM_DIV_4 = 0x03,
	PWM_DIV_5 = 0x04,
	PWM_DIV_6 = 0x05,
	PWM_DIV_7 = 0x06,
};

/**
 * The output falling slew rate.
 *
 * The rates don't directly translate to the output rising slew rates - see datasheet, table 5, page 12.
 */
enum L6470::SlewRate : uint8_t {
	SLEW_RATE_75V_US = 0x01,
	SLEW_RATE_110V_US = 0x02,
	SLEW_RATE_260V_US = 0x03,
	SLEW_RATE_320V_US = 0x00,
};

/**
 * External switch input functionality.
 */
enum L6470::SwitchMode : uint8_t {
	// Default; hard stop motor on switch.
	SWITCH_MODE_HARDSTOP = 0x00,
	// Tie to the GoUntil and ReleaseSW commands to provide jog function. See page 25 of datasheet.
	SWITCH_MODE_USER = 0x10,
};

enum L6470::OscillatorMode : uint8_t {
	// Internal 16MHz, no output
	OSC_MODE_INT_16MHZ = 0x00,
	// Default; internal 16MHz, 2MHz output
	OSC_MODE_INT_16MHZ_OSCOUT_2MHZ = 0x08,
	// Internal 16MHz, 4MHz output
	OSC_MODE_INT_16MHZ_OSCOUT_4MHZ = 0x09,
	// Internal 16MHz, 8MHz output
	OSC_MODE_INT_16MHZ_OSCOUT_8MHZ = 0x0A,
	// Internal 16MHz, 16MHz output
	OSC_MODE_INT_16MHZ_OSCOUT_16MHZ = 0x0B,
	// External 8MHz crystal
	OSC_MODE_EXT_8MHZ_XTAL_DRIVE = 0x04,
	// External 16MHz crystal
	OSC_MODE_EXT_16MHZ_XTAL_DRIVE = 0x05,
	// External 24MHz crystal
	OSC_MODE_EXT_24MHZ_XTAL_DRIVE = 0x06,
	// External 32MHz crystal
	OSC_MODE_EXT_32MHZ_XTAL_DRIVE = 0x07,
	// External 8MHz crystal, output inverted
	OSC_MODE_EXT_8MHZ_OSCOUT_INVERT = 0x0C,
	// External 16MHz crystal, output inverted
	OSC_MODE_EXT_16MHZ_OSCOUT_INVERT = 0x0D,
	// External 24MHz crystal, output inverted
	OSC_MODE_EXT_24MHZ_OSCOUT_INVERT = 0x0E,
	// External 32MHz crystal, output inverted
	OSC_MODE_EXT_32MHZ_OSCOUT_INVERT = 0x0F,
};

enum L6470::MotorStatus : uint16_t {
	MOTOR_STATUS_STOPPED = 0x00,
	MOTOR_STATUS_ACCELERATION = 0x01,
	MOTOR_STATUS_DECELERATION = 0x02,
	MOTOR_STATUS_CONST_SPD = 0x03,
};

enum L6470::RegisterAddress : uint8_t {
	REG_ADDR_ABS_POS = 0x01,
	REG_ADDR_EL_POS = 0x02,
	REG_ADDR_MARK = 0x03,
	REG_ADDR_SPEED = 0x04,
	REG_ADDR_ACC = 0x05,
	REG_ADDR_DECEL = 0x06,
	REG_ADDR_MAX_SPEED = 0x07,
	REG_ADDR_MIN_SPEED = 0x08,
	REG_ADDR_FS_SPD = 0x15,
	REG_ADDR_KVAL_HOLD = 0x09,
	REG_ADDR_KVAL_RUN = 0x0A,
	REG_ADDR_KVAL_ACC = 0x0B,
	REG_ADDR_KVAL_DEC = 0x0C,
	REG_ADDR_INT_SPD = 0x0D,
	REG_ADDR_ST_SLP = 0x0E,
	REG_ADDR_FN_SLP_ACC = 0x0F,
	REG_ADDR_FN_SLP_DEC = 0x10,
	REG_ADDR_K_THERM = 0x11,
	REG_ADDR_ADC_OUT = 0x12,
	REG_ADDR_OCD_TH = 0x13,
	REG_ADDR_STALL_TH = 0x14,
	REG_ADDR_STEP_MODE = 0x16,
	REG_ADDR_ALARM_EN = 0x17,
	REG_ADDR_CONFIG = 0x18,
	REG_ADDR_STATUS = 0x19,
};

enum L6470::Command : uint8_t {
	COMMAND_NOP = 0x00,
	COMMAND_SET_PARAM = 0x00,
	COMMAND_GET_PARAM = 0x20,
	COMMAND_RUN = 0x50,
	COMMAND_STEP_CLOCK = 0x58,
	COMMAND_MOVE = 0x40,
	COMMAND_GOTO = 0x60,
	COMMAND_GOTO_DIR = 0x68,
	COMMAND_GO_UNTIL = 0x82,
	COMMAND_RELEASE_SW = 0x92,
	COMMAND_GO_HOME = 0x70,
	COMMAND_GO_MARK = 0x78,
	COMMAND_RESET_POS = 0xD8,
	COMMAND_RESET_DEVICE = 0xC0,
	COMMAND_SOFT_STOP = 0xB0,
	COMMAND_HARD_STOP = 0xB8,
	COMMAND_SOFT_HIZ = 0xA0,
	COMMAND_HARD_HIZ = 0xA8,
	COMMAND_GET_STATUS = 0xD0,
};
