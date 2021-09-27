//PID.h

#include "mbed.h"

class PID{
private:
    float _kp,_ki,_kd,_imax;
    float _prev_input;
    float _P,_I,_D,_prev_D;
    float _dt;
    float filter;
    Serial pc;


public:
    PID(const float &Filter=1,const float &Kp=1,const float &Ki=0,const float &Kd=0,const float &Imax=100):
    _kp(Kp),_ki(Ki),_kd(Kd),_imax(abs(Imax)),filter(Filter)
    ,pc(USBTX,USBRX,115200){
    }
    float P_control(float _error);
    float I_control(float _error,float _dt);
    float D_control(float _input,float _dt);
    float get_PID(float _error,float _input,float _dt);
    void reset();

    float kp() const{
        return _kp;
    }
    void kp(const float value){
        _kp=value;
    }
    float ki() const{
        return _ki;
    }
    void ki(const float value){
        _ki=value;
    }
    float kd() const{
        return _kd;
    }
    void kd(const float value){
        _kd=value;
    }
    float imax() const{
        return _imax;
    }
    void printP();
    void printI();
    void printD();

    void I_reset();
};