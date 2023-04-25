/**
 * @file motorsclass.h
 *
 */
/*
 * Based on https://github.com/sparkfun/L6470-AutoDriver/tree/master/Libraries/Arduino
 */
/* Copyright (C) 2023 by Jakub Ostrysz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <memory>
#include <optional>
#include <vector>
#include <cmath>
#include <array>
#include "l6470.h"
#include "l6470constants.h"
#include "bcm2835.hpp"

#define GPIO_BUSY_IN	RPI_V2_GPIO_P1_13
#define GPIO_RESET_OUT 	RPI_V2_GPIO_P1_15

class Motors: public L6470 {
public:
    struct motorStatus{
        bool hiZ;
        bool busy;
        bool motorAcceleration;
        bool motorDeceleration;
        bool direction;
        bool motorStopped;
        bool motorConstSpeed;
        bool underVoltageLockout;
        bool thermalWarning;
        bool thermalShutdown;
        bool overCurrent;
        bool stepLossA;
        bool stepLossB;
    };

	Motors(uint8_t, uint8_t);

	~Motors(void);

	int busyCheck(void);
	void setSpeeds(const std::vector<float>&, std::optional<u_int8_t> index = std::nullopt);
    unsigned long spdCalc(float);
    std::vector<unsigned long> getSpeeds(std::optional<u_int8_t> index = std::nullopt);
	void stop(void);
    std::vector<int32_t> getPosition(std::optional<u_int8_t> index = std::nullopt);
    std::vector<uint8_t> getMicroStepMode(std::optional<u_int8_t> index = std::nullopt);
	void resetPosition(std::optional<u_int8_t> index = std::nullopt);
    void resetDevice(std::optional<u_int8_t> index = std::nullopt);
    std::vector<Motors::motorStatus> getMotorStatus(std::optional<u_int8_t> index = std::nullopt);
    void setOscillatorMode(const TL6470ConfigOsc&, std::optional<u_int8_t> index = std::nullopt);
    void configStepSelMode(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setMaximumSpeed(float, std::optional<u_int8_t> index = std::nullopt);
    void setMinimumSpeed(float, std::optional<u_int8_t> index = std::nullopt);
    void setFullStepModeSpeed(float, std::optional<u_int8_t> index = std::nullopt);
    void setAcceleration(float, std::optional<u_int8_t> index = std::nullopt);
    void setDeceleration(float, std::optional<u_int8_t> index = std::nullopt);
    void setPWMFrequency(int, int, std::optional<u_int8_t> index = std::nullopt);
    void setPowSlewRate(int, std::optional<u_int8_t> index = std::nullopt);
    void setOverCurrentThreshold(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setOverCurrentShutdown(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setVoltageCompensation(int, std::optional<u_int8_t> index = std::nullopt);
    void setSwitchModeConfig(int, std::optional<u_int8_t> index = std::nullopt);
    void setBackEMF(std::optional<u_int8_t> index = std::nullopt);
    void setAccCurrentKVAL(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setDecCurrentKVAL(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setRunCurrentKVAL(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    void setHoldCurrentKVAL(uint8_t, std::optional<u_int8_t> index = std::nullopt);
    bool IsConnected(int);

private:
	uint8_t SPIXfer(uint8_t);
	uint8_t m_nPosition; //0-left 1-right
	uint8_t m_nSpiChipSelect;
	uint8_t m_nResetPin;
	uint8_t m_nBusyPin;
    uint8_t m_nCount;
	bool l_bIsBusy;
	bool l_bIsConnected;
	bool r_bIsBusy;
	bool r_bIsConnected;
};

enum motors : int {
    LEFT_MOTOR = 0,
    RIGHT_MOTOR = 1
};
