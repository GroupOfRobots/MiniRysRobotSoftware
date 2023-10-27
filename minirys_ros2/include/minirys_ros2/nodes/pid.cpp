//#include "pid.hpp"
class PID
{
    float K;
    float Ti;
    float Td;
    float T;
    float e_past = 0.0f;
    float u_past = 0.0f;

public:
    PID(float T, float K=0.0f, float Ti=10000000.0f, float Td=0.0f){
        this->T = T;
        this->K = K;
        this->Td = Td;
        this->Ti = Ti;
    }

    float pid(float y, float y_zad){
        float e = y_zad - y;
        float up = this->K*e;
        float ui = this->u_past + this->K/this->Ti*this->T*(this->e_past + e)/2;
        float ud = this->K*this->Td*(e-this->e_past)/this->T;
        //float u = this->K*e + this->u_past + this->K/this->Ti*this->T*(this->e_past + e)/2 + this->K*this->Td*(e-this->e_past)/this->T;
        float u = up + ui + ud;
        this->u_past = ui;
        this->e_past = e;
        return u;
    }
    float p(float y, float y_zad){
        float e = y_zad - y;
        return this->K * e;
    }

    float pi(float y, float y_zad){
        float e = y_zad - y;
        float up = this->K*e;
        float ui = this->u_past + this->K/this->Ti*this->T*(this->e_past + e)/2;
        float u = up + ui;
        this->u_past = ui;
        this->e_past = e;
        return u;

    }
    float pd(float y, float y_zad){
        float e = y_zad - y;
        float up = this->K*e;
        float ud = this->K*this->Td*(e-this->e_past)/this->T;
        float u = up + ud;
        this->e_past = e;
        return u;

    }
    void clear(){
        this->e_past = 0.0f;
        this->u_past = 0.0f;
    }
};
