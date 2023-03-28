/**
 * @file autodriver.h
 *
 */
/*
 * Based on https://github.com/sparkfun/L6470-AutoDriver/tree/master/Libraries/Arduino
 */
/* Copyright (C) 2017 by Arjan van Vught mailto:info@raspberrypi-dmx.nl
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

#ifndef AUTODRIVER_H_
#define AUTODRIVER_H_

#include <stdint.h>
#include <memory>
#include "l6470.h"
#include "bcm2835.hpp"

#define GPIO_BUSY_IN	RPI_V2_GPIO_P1_13
#define GPIO_RESET_OUT 	RPI_V2_GPIO_P1_15

class Motors: public L6470 {
public:
	Motors(uint8_t, uint8_t);

	~Motors(void);

	int busyCheck(void);
	void setUp(void);
	void setSpeed(float,float);
	void stop(void);
	long getPositionLeft();
	long getPositionRight();
	void resetPosition();
	int getBatteryVoltage();
	void setMicrostep(uint8_t);
	void setMaxSpeedForBoth(float);
	long getStatusLeft();
	long getStatusRight();

private:
	uint8_t SPIXfer(uint8_t);

	/*
	 * Additional methods
	 */
public:
	bool IsConnected(int);


private:
	uint8_t m_nPosition; //0-left 1-right
	uint8_t m_nSpiChipSelect;
	uint8_t m_nResetPin;
	uint8_t m_nBusyPin;
	bool l_bIsBusy;
	bool l_bIsConnected;
	bool r_bIsBusy;
	bool r_bIsConnected;
};

#endif /* AUTODRIVER_H_ */
