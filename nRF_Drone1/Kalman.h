#include "Matrix.h"
#include "MatrixMath.h"

class Kalman{
public:
    Matrix _A,_H,_Q,_R;  //system model
    Matrix _P,_P_predict,_K;
    Matrix _x,_x_predict;
    Matrix _z;
    MatrixMath matrix;
    Kalman(const Matrix& x0,const Matrix& P0);

    const Matrix A(){
        return _A;
    }
    void A(const Matrix& mat){
        _A=mat;
    }
    const Matrix H(){
        return _H;
    }
    void H(const Matrix& mat){
        _H=mat;
    }
    const Matrix Q(){
        return _Q;
    }
    void Q(const Matrix& mat){
        _Q=mat;
    }
    const Matrix R(){
        return _H;
    }
    void R(const Matrix& mat){
        _R=mat;
    }

    void update_z(const Matrix& mat);

    Matrix filtering();
};