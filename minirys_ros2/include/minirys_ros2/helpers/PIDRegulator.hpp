#pragma once

#include <chrono>

class PIDRegulator {
public:
	PIDRegulator(float T, float K=0.0f, float Ti=10000000.0f, float Td=0.0f);

	float pid(float y, float y_zad);
	float pid_aw(float y, float y_zad,float Tv, float max);
	void clear();
	float pid(float y, float y_zad);


private:
	float K;
    float Ti;
    float Td;
    float T;
    float e_past = 0.0f;
    float u_past = 0.0f;
	float calcUdUp(float e);

};