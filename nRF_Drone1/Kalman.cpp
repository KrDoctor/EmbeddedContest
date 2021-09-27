#include "Kalman.h"

Kalman::Kalman(const Matrix& x0,const Matrix& P0):
    _x(x0),_P(P0)
    {}

void Kalman::update_z(const Matrix& mat){
    _z=mat;
}



Matrix Kalman::filtering(){
    _x_predict = _A*_x;
    _P_predict=_A*_P*matrix.Transpose(_A)+_Q;

    _K=_P_predict*matrix.Transpose(_H)*matrix.Inv(_H*_P_predict*matrix.Transpose(_H)+_R);

    _x=_x_predict+_K*_z-_K*_H*_x_predict;

    _P=_P_predict-_K*_H*_P_predict;
    
    return _x;
}