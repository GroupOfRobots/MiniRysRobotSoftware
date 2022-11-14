#include "minirys_ros2/helpers/PIDRegulator.hpp"

#include <algorithm>
#include <iostream>

PIDRegulator::PIDRegulator():
	looptime(0.0),
	kp(0),
	ki(0),
	kd(0),
	max(0),
	setpoint(0.0),
	prevError1(0.0),
    prevError2(0.0),
    output(0.0) {}

void PIDRegulator::setParams(std::chrono::duration<double> looptime, double kp, double ki, double kd, double max) {
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->max = max;
	this->looptime = looptime.count();
	this->zero();
}

double PIDRegulator::update(double setpoint, double value, double outValue) {

    double error = -setpoint + value;
    double Factor0 = this->kp * (1 + this->ki * this->looptime / 2 + this->kd / this->looptime);
    double Factor1 = this->kp * (this->ki * this->looptime / 2 - 2 * this->kd / this->looptime - 1);
    double Factor2 = this->kp * this->kd / this->looptime;

    this->output = Factor0 * error + Factor1 * this->prevError1 + Factor2 * this->prevError2 + outValue;
    this->prevError2 = this->prevError1;
    this->prevError1 = error;

	return std::min(std::max(output, -this->max), this->max);
}

void PIDRegulator::zero() {
	this->setpoint = 0.0;
	this->prevError1 = 0.0;
    this->prevError2 = 0.0;
	this->output = 0.0;
}
