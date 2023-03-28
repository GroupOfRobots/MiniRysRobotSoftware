/**
 * @file autodriver.cpp
 *
 */
/*
 * Based on https://github.com/sparkfun/L6470-AutoDriver/tree/master/Libraries/Arduino
 */
/* Copyright (C) 2017-2018 by Arjan van Vught mailto:info@raspberrypi-dmx.nl
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
#include <stdio.h>
#include <assert.h>
#include <chrono>         // std::chrono::system_clock
#include <ctime>          // std::time_t, std::tm, std::localtime, std::mktime
#include <iostream>

#include "bcm2835.hpp"

//#if defined(__linux__)
//#else
// #include "bcm2835_gpio.h"
// #include "bcm2835_spi.h"
//#endif

#include "motorsclass.h"

#include "l6470constants.h"

#define BUSY_PIN_NOT_USED	0xFF


Motors::Motors(uint8_t nSpiChipSelect, uint8_t nResetPin) : l_bIsBusy(false), l_bIsConnected(false),r_bIsBusy(false), r_bIsConnected(false) {
	assert(nSpiChipSelect <= BCM2835_SPI_CS1);
	assert(nResetPin <= 31);

	m_nSpiChipSelect = nSpiChipSelect;
	m_nPosition = 0;
	m_nResetPin = nResetPin;
	m_nBusyPin = BUSY_PIN_NOT_USED;

	bcm2835_spi_begin();
	bcm2835_gpio_fsel(GPIO_RESET_OUT, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_set(GPIO_RESET_OUT);
	bcm2835_gpio_fsel(GPIO_BUSY_IN, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_clr(GPIO_RESET_OUT);
	bcm2835_delayMicroseconds(10000);
	bcm2835_gpio_set(GPIO_RESET_OUT);
	bcm2835_delayMicroseconds(10000);

	if (getParam(L6470_PARAM_CONFIG) == 0x2e88) {
		l_bIsConnected = true;
	}
	m_nPosition =1;
	if (getParam(L6470_PARAM_CONFIG) == 0x2e88) {
		r_bIsConnected = true;
	}
}

Motors::~Motors(void) {
	hardHiZ();
	m_nPosition=0;
	this->hardStop();
	m_nPosition=1;
	this->hardStop();
	l_bIsBusy = false;
	l_bIsConnected = false;
	r_bIsBusy = false;
	r_bIsConnected = false;
}

void Motors::setUp(){

	m_nPosition = 0;

	this->setParam(L6470_PARAM_CONFIG,0x2e8B); //16MHz
	this->configStepMode(0x06);   // 64microsteps per step
	this->setMaxSpeed(1000);        // 350 steps/s max
	this->setMinSpeed(0);        // 10 steps/s min
	this->setAcc(8000);              // accelerate at 350 steps/s/s
	this->setDec(8000);//1000
	this->setPWMFreq((0x00)<<13, (0x07)<<10); // 62.5kHz PWM freq
	this->setSlewRate(L6470_CONFIG_POW_SR_260V_us);   // Upping the edge speed increases torque.
	this->setOCThreshold(0x09);  // OC threshold 3000mA
	this->setOCShutdown(0x0000); // don't shutdown on OC
	//this->setVoltageComp(0x0000); // don't compensate for motor V
	//this->setSwitchMode(0x0010);    // Switch is not hard stop
	this->setParam(L6470_PARAM_ST_SLP,0x00);
	this->setParam(L6470_PARAM_FN_SLP_ACC,0x00);
	this->setParam(L6470_PARAM_FN_SLP_DEC,0x00);
	this->setParam(L6470_PARAM_ALARM_EN,0x00);
	this->setAccKVAL(0x96);           // We'll tinker with these later, if needed.
	this->setDecKVAL(0x96);
	this->setRunKVAL(0x96);
	this->setHoldKVAL(0x32);           // This controls the holding current; keep it low.
	//this->setFullSpeed(120);       // microstep above 120 steps/s, default value 602.7

	m_nPosition = 1;

	this->setParam(L6470_PARAM_CONFIG,0x2e8B); //16MHz
	this->configStepMode(0x06);   // 64microsteps per step
	this->setMaxSpeed(1000);        // 350 steps/s max
	this->setMinSpeed(0);        // 10 steps/s min
	this->setAcc(8000);             // accelerate at 350 steps/s/s
	this->setDec(8000);//1000
	this->setPWMFreq((0x00)<<13, (0x07)<<10); // 62.5kHz PWM freq
	this->setSlewRate(L6470_CONFIG_POW_SR_260V_us);   // Upping the edge speed increases torque.
	this->setOCThreshold(0x09);  // OC threshold 3000mA
	this->setOCShutdown(0x0000); // don't shutdown on OC
	//this->setVoltageComp(0x0000); // don't compensate for motor V
	//this->setSwitchMode(0x0010);    // Switch is not hard stop
	this->setParam(L6470_PARAM_ST_SLP,0x00);
	this->setParam(L6470_PARAM_FN_SLP_ACC,0x00);
	this->setParam(L6470_PARAM_FN_SLP_DEC,0x00);
	this->setParam(L6470_PARAM_ALARM_EN,0x00);
	this->setAccKVAL(0x96);           // We'll tinker with these later, if needed.
	this->setDecKVAL(0x96);
	this->setRunKVAL(0x96);
	this->setHoldKVAL(0x32);         // This controls the holding current; keep it low.
	//this->setFullSpeed(120);       // microstep above 120 steps/s, default value 602.7
}

void Motors::setMaxSpeedForBoth(float speed){
	m_nPosition = 0;
	this->setMaxSpeed(speed);
	m_nPosition = 1;
	this->setMaxSpeed(speed);
}

void Motors::setSpeed(float speedLeft, float speedRight){
	// // auto start = std::chrono::system_clock::now();
	// m_nPosition=0;
	// while (this->busyCheck())
	// 	;
	// m_nPosition=1;
	// while (this->busyCheck())
	// 	;
	// // auto end = std::chrono::system_clock::now();
	// // std::chrono::duration<double> elapsed_seconds = end-start;
 //    // std::cout << "busyCheck duration: " << elapsed_seconds.count() << "s\n";
	m_nPosition=0;
	if (speedLeft>=0)this->run(L6470_DIR_FWD,speedLeft);
	else this->run(L6470_DIR_REV,-1*speedLeft);

	m_nPosition=1;
	if (speedRight>=0)this->run(L6470_DIR_FWD,speedRight);
	else this->run(L6470_DIR_REV,-1*speedRight);
}

void Motors::setMicrostep(uint8_t step){


		m_nPosition = 0;
		this->configStepMode(step);

		m_nPosition = 1;
		this->configStepMode(step);
}

void Motors::stop(){

	m_nPosition=0;
	this->softStop();
	m_nPosition=1;
	this->softStop();
	m_nPosition=0;
	while (this->busyCheck())
		;
	m_nPosition=1;
	while (this->busyCheck())
		;
	m_nPosition=0;
	this->hardHiZ();
	m_nPosition=1;
	this->hardHiZ();
}

int Motors::busyCheck(void) {
	if (m_nPosition==0) {
		if (getParam(L6470_PARAM_STATUS) & L6470_STATUS_BUSY) {
			return 0;
		} else {
			return 1;
		}
	}else{
		if (getParam(L6470_PARAM_STATUS) & L6470_STATUS_BUSY) {
			return 0;
		} else {
			return 1;
		}

	}
}

uint8_t Motors::SPIXfer(uint8_t data) {
	uint8_t dataPacket[2];

	for (int i = 0; i < 2; i++) {
		dataPacket[i] = 0;
	}

	dataPacket[m_nPosition] = data;

	bcm2835_spi_chipSelect(m_nSpiChipSelect);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
	bcm2835_spi_transfern((char *) dataPacket, 2);

	return dataPacket[m_nPosition];
}

/*
 * Additional method
 */
bool Motors::IsConnected(int position){


	if (position) {
		return r_bIsConnected;
	}else{
		return l_bIsConnected;
	}
}

long Motors::getPositionLeft(){
		m_nPosition=0;
		return this->getPos();
}

long Motors::getPositionRight(){
		m_nPosition=1;
		return this->getPos();
}

void Motors::resetPosition(){
		m_nPosition=0;
		this->resetPos();
		m_nPosition=1;
		this->resetPos();
}

int Motors::getBatteryVoltage(){
	int voltage;
	m_nPosition = 0;
	voltage = this->getVoltageComp();
	m_nPosition = 1;
	voltage += this->getVoltageComp();

	return (voltage/16)*3.3*3.1;
}

long Motors::getStatusLeft(){
	m_nPosition = 0;
	return this->getStatus();
}

long Motors::getStatusRight(){
	m_nPosition = 1;
	return this->getStatus();
}

