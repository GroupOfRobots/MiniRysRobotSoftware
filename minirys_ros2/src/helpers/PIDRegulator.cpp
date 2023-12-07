#include "minirys_ros2/helpers/PIDRegulator.hpp"
#include <algorithm>

PIDRegulator(float T, float K=0.0f, float Ti=10000000.0f, float Td=0.0f): T(T), K(K), Ti(Ti), Td(Td) {};

float PIDRegulator::pid(float y, float y_zad){
	float e = y_zad - y;
	//float up = this->K*e;
	float ui = this->u_past + this->K/this->Ti*this->T*(this->e_past + e)/2;
	//float ud = this->K*this->Td*(e-this->e_past)/this->T;
	float u = calcUdUp(e) + ui;
	this->u_past = ui;
	this->e_past = e;
	return u;
}


float PIDRegulator::pid_aw(float y, float y_zad,float Tv, float max){
        float uw;
        if(this->u_past >max ){
            uw = 15.0f;
        }
        else if (this->u_past < -max)
        {
            uw = -15.0f;
        }
        else{
            uw = this->u_past;
        }
        
        float e = y_zad - y;
        //float up = this->K*e;
        float ui = this->u_past + this->K/this->Ti*this->T*(this->e_past + e)/2 +this->T /Tv * (uw - this->u_past);
        //float ud = this->K*this->Td*(e-this->e_past)/this->T;
        float u = clacUdUp(e) + ui;
        this->u_past = ui;
        this->e_past = e;
        return u;
    }

float PIDregulator::calcUd(float e){
	float up = this->K*e;
	float ud = this->K*this->Td*(e-this->e_past)/this->T;
	return up + ud;
}


void PIDRegulator::clear(){
        this->e_past = 0.0f;
        this->u_past = 0.0f;
    }