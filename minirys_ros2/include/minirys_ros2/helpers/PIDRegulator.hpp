#pragma once

#include <chrono>

class PIDRegulator {
public:
	PIDRegulator();

	~PIDRegulator() = default;

	void setParams(std::chrono::duration<double> looptime, double kp, double ki, double kd, double max);

	double update(double setpoint, double value, double outValue);

	void zero();

private:
	double looptime;
	double kp;
	double ki;
	double kd;
	double max;

	double setpoint;
	double prevError1;
    double prevError2;
	double output;
};
