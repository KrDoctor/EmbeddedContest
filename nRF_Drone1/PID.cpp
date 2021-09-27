//PID.cpp

#include "mbed.h"
#include "PID.h"

float PID::P_control(float _error){
    _P = _error*_kp;
    return _P;
}
float PID::I_control(float _error, float _dt){
    // if(_ki==0) return 0;
    _I +=  (_error*_ki)*_dt;
    if(_I<-_imax) _I=-_imax;
    else if (_I>_imax) _I = _imax;
    return _I;
}
float PID::D_control(float _input, float _dt){
    // if(_kd==0) return 0;
    _D = (_input-_prev_input)/_dt;
    _D=filter * _D + (1 - filter) * _prev_D;
    _prev_input = _input;
    _prev_D = _D;
    return _kd*_D;
}
float PID::get_PID(float _error, float _input, float _dt){
    return P_control(_error)+I_control(_error, _dt)-D_control(_input, _dt);
}
void PID::reset(){
    _I=0;
    _prev_input=0;
    _prev_D=0;
}
void PID::printP(){
    pc.printf("$%.2f;\n",_P);
}
void PID::printI(){
    pc.printf("$%.2f;\n",_I);
}
void PID::printD(){
    pc.printf("$%.2f;\n",_D);
}

void PID::I_reset(){
    _I=0;
}