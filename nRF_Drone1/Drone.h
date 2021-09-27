//Drone.h 코드

#ifndef Drone_h
#define Drone_h

#include "mbed.h"
#include "PID.h"
#include <cstdint>
#include <string>
#include <cmath>
#include "Kalman.h"
#include "PMW3901.h"
#include "FastPWM.h"
#include "BNO080.h"
#include "Quaternion.h"
#include "Define.h"
#include "IMUfilter.h"
#include <algorithm>

#define MAX_PULSEWIDTH 2000.
#define MIN_PULSEWIDTH 1000.
#define PULSE_RANGE 20000.
#define RADtoDEG 180/3.141592
#define DEGtoRAD 3.141592/180
#define tau 1.0
#define PID_MAX 200.
#define gravity 9.80665

class PID;
class Drone{
private:
    FastPWM motor1;
    FastPWM motor2;
    FastPWM motor3;
    FastPWM motor4;
    Serial *pc;
    Timer *timer;
    Mutex m1;
    optical_PMW3901 *flow;
    I2C *LIDAR;

    bool esc_flag_MIN = false;
    bool esc_flag_MAX = false;
    bool timer_first_check = false;
    IMUfilter madgwick;

    double BNO080_Roll_Rate_RAD, BNO080_Roll_Rate_DEG, BNO080_Roll_Rate_DEG_LPF, BNO080_Pitch_Rate_RAD, BNO080_Pitch_Rate_DEG, BNO080_Pitch_Rate_DEG_LPF, BNO080_Yaw_Rate_RAD, BNO080_Yaw_Rate_DEG,BNO080_Yaw_Rate_DEG_LPF;
    double BNO080_Roll_RAD, BNO080_Pitch_RAD, BNO080_Yaw_RAD;
    double BNO080_Roll_RAD_Rate_LPF, BNO080_Pitch_RAD_Rate_LPF, BNO080_Yaw_RAD_Rate_LPF;
    double OffSet_Roll = 0, OffSet_Pitch = 0, OffSet_Yaw = 0, cos_pitch, sin_pitch, cos_roll, sin_roll, cos_theta, sin_theta, theta_rad, Yaw_Real;
    float q[4], quatRadianAccuracy;
    double Offset_GyroX = 0, Offset_GyroY = 0, Offset_GyroZ = 0;
    double Offset_AccX = 0, Offset_AccY = 0, Offset_AccZ = 0;
    bool Do_Cali_IMU = true;
    uint32_t Count_FOR_CALIBRATION = 0;
    bool IMU_first_check=false;
    double Madgwick_Roll, Madgwick_Pitch, Madgwick_Yaw;
    double LinAcc_X_Body, LinAcc_Y_Body, LinAcc_Z_Body;
    double LinAcc_X_Earth, LinAcc_Y_Earth, LinAcc_Z_Earth;
    double LinAcc_X_Measured, LinAcc_Y_Measured, LinAcc_Z_Measured;

    //lidar
    double Distance_LIDAR, Distance_LIDAR_NotCalibrated;
    double Distance_Optical;
    bool Do_Cali_LIDAR = true;
    double OffSet_LIDAR = 0;
    
    float Roll_PID(float roll,float rate,float velocity,float position);
    float Pitch_PID(float pitch,float rate,float velocity,float position);
    float Yaw_PID(float yaw,float rate);
    float Altitude_PID(float target,float altitude,float velocity);
    float Velocity_x_PID(float target_vel,float velocity);
    float Velocity_y_PID(float target_vel,float velocity);
    float Position_x_PID(float target_pos,float position);
    float Position_y_PID(float target_pos,float position);
    float PID_R=0,PID_P=0,PID_Y=0,PID_A=0;

    float gain;
    float prev_range=0;
    float _altitude=27;
    float prev_altitude=0,vel_altitude,acl_altitude;
    float prev_Vx,prev_Vy;
    float V_x,V_y;
    bool run=false;
    

    float init_altitude;

    Kalman *alti_Kalman;
    Matrix _x,_p;
    Matrix a_a,a_h,a_q,a_r,a_x,a_p,a_z;
    MatrixMath mm;
    Kalman *xpos_Kalman;
    Kalman *ypos_Kalman;
    Matrix x_z,y_z;
    
    template <class T>
    T map(T x, T in_min, T in_max, T out_min, T out_max);
    template <class A>
    A constrain(A x, A min, A max);

    float GyroX_value[9]={0},GyroY_value[9]={0},GyroZ_value[9]={0};
    float sorted_GyroX[3]={0},sorted_GyroY[3]={0},sorted_GyroZ[3]={0};
    
    
public:
    Drone(PinName motor1_pin,PinName motor2_pin,PinName motor3_pin,PinName motor4_pin,Serial *serial,Timer *timer_ptr,I2C *lidar_ptr,optical_PMW3901 *optical_ptr);
    //laser setting//
    float altitude() const{
        return Distance_LIDAR;
    }
    float altitude_vel() const{
        return vel_altitude;
    }
    void altitude_vel(const float value){
        vel_altitude=value;
    }
    
    void ESC_calibration();

    void calibrate_IMU();

    void dt();

    void get_data();

    void getAngle();
    void getYaw();

    float Roll();
    float Pitch();
    float Yaw();
    float Roll_R();
    float Pitch_R();
    float Yaw_R();

    float accelX();
    float accelY();
    float accelZ();

    float Rrate();
    float Prate();
    float Yrate();

    void Run();
    void Stop();

    void ESC_MAX();
    void ESC_MIN();
    void ESC_false();

    void ESC_control();

    void Calibrate_LIDAR();
    void get_altitude();

    void set_alti_kalman();
    void Altitude_Kalman(float altitude,float accel);
    void set_xpos_kalman();
    void set_ypos_kalman();
    void X_Position_Kalman(float x_pos,float x_vel);
    void Y_Position_Kalman(float y_pos,float y_vel);
    float targetR,targetP,targetY,target_altitude;
    float target_xvel=0,target_yvel=0;
    float Throttle;
    float sample_time;
    int16_t deltaX,deltaY;
    float x_velocity,y_velocity;
    float velocity_x,velocity_y;
    float accel_x,accel_y;
    float Altitude;
    float target_xpos=0,target_ypos=0;
    float x_position,y_position;
    float Kalman_xpos,Kalman_ypos;
    float prev_xpos,prev_ypos;
    float esc1,esc2,esc3,esc4;
    
    void get_velocity();
    void Velocity_LPF();
    void set_vel_Kalman();
    void Velocity_Kalman(int dir,float velocity,float accel);

    ///Communication///
    void start_Connection();  

    void Request_mission();

    void Check_Emergency();

    bool emergency_flag = false;
    Timer check_timer;

    //i2c communication
    void writeByte(uint8_t address, uint8_t regAddress, uint8_t data);
    float readByte(uint8_t address, uint8_t regAddress);

    PID pid_angle_R;
    PID pid_rate_R;
    PID pid_angle_P;
    PID pid_rate_P;
    PID pid_angle_Y;
    PID pid_rate_Y;
    PID pid_altitude;
    PID pid_altitude_vel;
    PID pid_x_velocity;
    PID pid_y_velocity;
    PID pid_x_position;
    PID pid_y_position;

    bool error_flag=false;
    
    void Median();
    float median_GyroX,median_GyroY,median_GyroZ;
};


#endif