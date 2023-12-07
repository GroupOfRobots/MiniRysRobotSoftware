#pragma once

#include <chrono>

class PIDRegulator {
public:
	PIDRegulator(float t, float k, float ti, float td);
	float pid(float y, float y_zad);
	float pid_aw(float y, float y_zad,float Tv, float max);
	void clear();


private:
	float T;
	float K;
    float Ti;
    float Td;
    float e_past = 0.0f;
    float u_past = 0.0f;
	float calcUdUp(float e);

};