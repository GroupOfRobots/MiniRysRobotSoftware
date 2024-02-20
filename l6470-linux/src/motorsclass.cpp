/**
 * @file motorsclass.cpp
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
#include <stdio.h>
#include <assert.h>
#include <iostream>


#include "motorsclass.h"

#define BUSY_PIN_NOT_USED	0xFF

Motors::Motors(uint8_t nSpiChipSelect, uint8_t nResetPin) : l_bIsBusy(false), l_bIsConnected(false), r_bIsBusy(false), r_bIsConnected(false) {
    assert(nSpiChipSelect <= 1);
    assert(nResetPin <= 31);

    m_nSpiChipSelect = nSpiChipSelect;
    m_nPosition = 0;
    m_nSpiChannel = 0;
    m_nResetPin = nResetPin;
    m_nBusyPin = BUSY_PIN_NOT_USED;
    m_nCount = 2;
    m_nSpiSpeed = 1000000;

    // Initialize WiringPi and SPI
    wiringPiSetup();
    wiringPiSPISetupMode(m_nSpiChipSelect, m_nSpiSpeed, SPI_MODE_3);

    // Reset procedure
    pinMode(GPIO_RESET_OUT, OUTPUT);
    digitalWrite(GPIO_RESET_OUT, HIGH);
    pinMode(GPIO_BUSY_IN, INPUT);
    digitalWrite(GPIO_RESET_OUT, LOW);
    delayMicroseconds(1000);
    digitalWrite(GPIO_RESET_OUT, HIGH);
    delayMicroseconds(1000);

    // Check if motors are connected
    if (getParam(L6470_PARAM_CONFIG) == 0x2e88) {
        l_bIsConnected = true;
    }
    m_nPosition = 1;
    if (getParam(L6470_PARAM_CONFIG) == 0x2e88) {
        r_bIsConnected = true;
    }
}

Motors::~Motors(void) {
	hardHiZ();
    for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
        this->softStop();
        this->resetDev();
    }
	l_bIsBusy = false;
	l_bIsConnected = false;
	r_bIsBusy = false;
	r_bIsConnected = false;
}

void Motors::setOscillatorMode(const TL6470ConfigOsc& oscillator, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setOscMode(oscillator);
        }
    }
    else{
        m_nPosition = index.value();
        this->setOscMode(oscillator);
    }
}

void Motors::configStepSelMode(uint8_t stepMode, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->configStepMode(stepMode);
        }
    }
    else{
        m_nPosition = index.value();
        this->configStepMode(stepMode);
    }
}

void Motors::setMaximumSpeed(float speedInStepsPerSecond, std::optional<u_int8_t> index) {
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setMaxSpeed(speedInStepsPerSecond);
        }
    }
    else{
        m_nPosition = index.value();
        this->setMaxSpeed(speedInStepsPerSecond);
    }
}

void Motors::setMinimumSpeed(float speedInStepsPerSecond, std::optional<u_int8_t> index) {
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setMinSpeed(speedInStepsPerSecond);
        }
    }
    else{
        m_nPosition = index.value();
        this->setMinSpeed(speedInStepsPerSecond);
    }
}

void Motors::setFullStepModeSpeed(float speedInStepsPerSecond, std::optional<u_int8_t> index) {
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setFullSpeed(speedInStepsPerSecond);
        }
    }
    else{
        m_nPosition = index.value();
        this->setFullSpeed(speedInStepsPerSecond);
    }
}

void Motors::setAcceleration(float accInStepsPerSecond2, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setAcc(accInStepsPerSecond2);
        }
    }
    else{
        m_nPosition = index.value();
        this->setAcc(accInStepsPerSecond2);
    }
}

void Motors::setDeceleration(float decInStepsPerSecond2, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setDec(decInStepsPerSecond2);
        }
    }
    else{
        m_nPosition = index.value();
        this->setDec(decInStepsPerSecond2);
    }
}

void Motors::setPWMFrequency(int divisor, int multiplier, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setPWMFreq(divisor, multiplier);
        }
    }
    else{
        m_nPosition = index.value();
        this->setPWMFreq(divisor, multiplier);
    }
}

void Motors::setPowSlewRate(int slewRate, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setSlewRate(slewRate);
        }
    }
    else{
        m_nPosition = index.value();
        this->setSlewRate(slewRate);
    }
}

void Motors::setOverCurrentThreshold(uint8_t threshold, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setOCThreshold(threshold);
        }
    }
    else{
        m_nPosition = index.value();
        this->setOCThreshold(threshold);
    }
}

void Motors::setOverCurrentShutdown(uint8_t shutdown, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setOCShutdown(shutdown);
        }
    }
    else{
        m_nPosition = index.value();
        this->setOCShutdown(shutdown);
    }
}

void Motors::setVoltageCompensation(int vsCompMode, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setVoltageComp(vsCompMode);
        }
    }
    else{
        m_nPosition = index.value();
        this->setVoltageComp(vsCompMode);
    }
}

void Motors::setSwitchModeConfig(int switchMode, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setSwitchMode(switchMode);
        }
    }
    else{
        m_nPosition = index.value();
        this->setSwitchMode(switchMode);
    }
}

void Motors::setBackEMF(std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setParam(L6470_PARAM_ST_SLP,0x00);
            this->setParam(L6470_PARAM_FN_SLP_ACC,0x00);
            this->setParam(L6470_PARAM_FN_SLP_DEC,0x00);
            this->setParam(L6470_PARAM_ALARM_EN,0x00);
        }
    }
    else{
        m_nPosition = index.value();
        this->setParam(L6470_PARAM_ST_SLP,0x00);
        this->setParam(L6470_PARAM_FN_SLP_ACC,0x00);
        this->setParam(L6470_PARAM_FN_SLP_DEC,0x00);
        this->setParam(L6470_PARAM_ALARM_EN,0x00);
    }
}

void Motors::setAccCurrentKVAL(uint8_t kValInput, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setAccKVAL(kValInput);
        }
    }
    else{
        m_nPosition = index.value();
        this->setAccKVAL(kValInput);
    }
}

void Motors::setDecCurrentKVAL(uint8_t kValInput, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setDecKVAL(kValInput);
        }
    }
    else{
        m_nPosition = index.value();
        this->setDecKVAL(kValInput);
    }
}

void Motors::setRunCurrentKVAL(uint8_t kValInput, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setRunKVAL(kValInput);
        }
    }
    else{
        m_nPosition = index.value();
        this->setRunKVAL(kValInput);
    }
}

void Motors::setHoldCurrentKVAL(uint8_t kValInput, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->setHoldKVAL(kValInput);
        }
    }
    else{
        m_nPosition = index.value();
        this->setHoldKVAL(kValInput);
    }
}

void Motors::setSpeeds(const std::vector<float>& speeds, std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            speeds[m_nPosition] >=0 ? this->run(L6470_DIR_FWD,speeds[m_nPosition]) : this->run(L6470_DIR_REV,-1*speeds[m_nPosition]);
        }
    }
    else{
        m_nPosition = index.value();
        speeds[m_nPosition] >=0 ? this->run(L6470_DIR_FWD,speeds[m_nPosition]) : this->run(L6470_DIR_REV,-1*speeds[m_nPosition]);
    }
}

unsigned long Motors::spdCalc(float stepsPerSec) {
    unsigned long temp = stepsPerSec / 67.103864;
    if (temp > 0x000FFFFF)
        return 0x000FFFFF;
    else
        return temp;
}

std::vector<unsigned long> Motors::getSpeeds(std::optional<u_int8_t> index){
    std::vector<unsigned long> speeds;
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            unsigned long temp = getParam(L6470_PARAM_SPEED);
            if (temp & 0x00200000) {
                temp |= 0xffc00000;
            }
            temp = spdCalc(temp);
            speeds.emplace_back(temp);
        }
    }
    else{
        m_nPosition = index.value();
        unsigned long temp = getParam(L6470_PARAM_SPEED);
        if (temp & 0x00200000) {
            temp |= 0xffc00000;
        }
        temp = spdCalc(temp);
        speeds.emplace_back(temp);
    }
    return speeds;
}

void Motors::stop(){
	m_nPosition=0;
	this->softStop();
	m_nPosition=1;
	this->softStop();
	m_nPosition=0;
	while (this->busyCheck());
	m_nPosition=1;
	while (this->busyCheck());
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
	}
    else{
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
    wiringPiSPIDataRW(m_nSpiChannel,static_cast<unsigned char*>(dataPacket), 2);

    return dataPacket[m_nPosition];
}

bool Motors::IsConnected(int position){
	if (position) {
		return r_bIsConnected;
	}else{
		return l_bIsConnected;
	}
}

std::vector<int32_t> Motors::getPosition(std::optional<u_int8_t> index){
    std::vector<int32_t> positions;
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            positions.emplace_back(this->getPos());
        }
    }
    else{
        m_nPosition = index.value();
        positions.emplace_back(this->getPos());
    }
    return positions;
}

std::vector<uint8_t> Motors::getMicroStepMode(std::optional<u_int8_t> index){
    std::vector<uint8_t> stepMode;
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            stepMode.emplace_back(this->getStepMode());
        }
    }
    else{
        m_nPosition = index.value();
        stepMode.emplace_back(this->getStepMode());
    }
    return stepMode;
}

void Motors::resetPosition(std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->resetPos();
        }
    }
    else{
        m_nPosition = index.value();
        this->resetPos();
    }
}

void Motors::resetDevice(std::optional<u_int8_t> index){
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            this->resetDev();
        }
    }
    else{
        m_nPosition = index.value();
        this->resetDev();
    }
}

std::vector<Motors::motorStatus> Motors::getMotorStatus(std::optional<u_int8_t> index){
    std::vector<Motors::motorStatus> statuses;
    if(index == std::nullopt){
        for(m_nPosition = 0; m_nPosition < m_nCount; m_nPosition++){
            const long rawStatus = this->getStatus();
            Motors::motorStatus status;
            status.stepLossA = ~rawStatus & L6470_STATUS_STEP_LOSS_A;
            status.stepLossB = ~rawStatus & L6470_STATUS_STEP_LOSS_B;
            status.overCurrent = rawStatus & L6470_STATUS_OCD;
            status.thermalShutdown = rawStatus & L6470_STATUS_TH_SD;
            status.thermalWarning = rawStatus & L6470_STATUS_TH_WRN;
            status.underVoltageLockout = rawStatus & L6470_STATUS_UVLO;
            status.motorStopped = ~rawStatus & 0x0060;
            status.motorAcceleration = (~rawStatus & 0x0040) & (rawStatus & 0x0020);
            status.motorDeceleration = (rawStatus & 0x0040) & (~rawStatus & 0x0020);
            status.motorConstSpeed = rawStatus & 0x0060;
            status.direction = rawStatus & L6470_STATUS_DIR;
            status.hiZ = rawStatus & L6470_STATUS_HIZ;
            status.busy = ~rawStatus & L6470_STATUS_BUSY;
            statuses.emplace_back(status);
        }
    }
    else{
        m_nPosition = index.value();
        const long rawStatus = this->getStatus();
        Motors::motorStatus status;
        status.stepLossA = ~rawStatus & L6470_STATUS_STEP_LOSS_A;
        status.stepLossB = ~rawStatus & L6470_STATUS_STEP_LOSS_B;
        status.overCurrent = rawStatus & L6470_STATUS_OCD;
        status.thermalShutdown = rawStatus & L6470_STATUS_TH_SD;
        status.thermalWarning = rawStatus & L6470_STATUS_TH_WRN;
        status.underVoltageLockout = rawStatus & L6470_STATUS_UVLO;
        status.motorStopped = ~rawStatus & 0x0060;
        status.motorAcceleration = (~rawStatus & 0x0040) & (rawStatus & 0x0020);
        status.motorDeceleration = (rawStatus & 0x0040) & (~rawStatus & 0x0020);
        status.motorConstSpeed = rawStatus & 0x0060;
        status.direction = rawStatus & L6470_STATUS_DIR;
        status.hiZ = rawStatus & L6470_STATUS_HIZ;
        status.busy = ~rawStatus & L6470_STATUS_BUSY;
        statuses.emplace_back(status);
    }
    return statuses;
}



