//Drone.cpp 코드

#include "Drone.h"
#include <cmath>


class MPU6050;
class PID;
Drone::Drone(PinName motor1_pin,PinName motor2_pin,PinName motor3_pin,PinName motor4_pin,Serial *serial,Timer *timer_ptr,I2C *lidar_ptr,optical_PMW3901 *optical_ptr):
    motor1(motor1_pin),motor2(motor2_pin),motor3(motor3_pin),motor4(motor4_pin)
    ,pc(serial)
    ,timer(timer_ptr)
    ,LIDAR(lidar_ptr)
    ,flow(optical_ptr)
    ,pid_angle_R(1,4.,0.5),pid_angle_P(1,4.,0.5),pid_rate_R(0.01,0.8,0.5,0.05),pid_rate_P(0.01,0.8,0.5,0.05),pid_angle_Y(1,7.,0.7),pid_rate_Y(0.02,4,0.5)
    ,pid_altitude(1,0.5,0.1),pid_altitude_vel(0.01,0.5,1.0,0.5,500)  // 고도 속도 gain P:I = 1:20.
    ,pid_x_velocity(0.01,6.5,1.2,0.05,7.5),pid_y_velocity(0.01,6.5,0.7,0.05,7.5)
    ,pid_x_position(0.02,1.4,0,0.085,0.25),pid_y_position(0.02,1.4,0,0.07,0.25)
    ,Throttle(1250),targetR(0),targetP(0),target_altitude(120),target_xpos(2.5),target_ypos(2.5)
    ,_x(2,1),_p(mm.Eye(2))
    ,a_a(2,2),a_h(1,2),a_q(2,2),a_r(1,1),a_x(2,1),a_z(1,1)
    ,x_z(2,1),y_z(2,1)
    ,alti_Kalman(NULL)
    ,xpos_Kalman(NULL),ypos_Kalman(NULL)
    ,madgwick(0.005,5)
    {
        pid_angle_P.reset();
        pid_angle_R.reset();
        pid_rate_R.reset();
        pid_rate_P.reset();
        pid_angle_Y.reset();
        pid_rate_Y.reset();
        pid_altitude.reset();
        pid_altitude_vel.reset();
        pid_x_velocity.reset();
        pid_y_velocity.reset();
        pid_x_position.reset();
        pid_y_position.reset();
        madgwick.reset();
        _x(1,1)=28.;
    }

void Drone::set_alti_kalman(){
    a_h(1,1)=1;
    // a_h(2,3)=1;
    a_q(1,1)=0.1;
    a_q(2,2)=3;
    // a_q(3,3)=0.005;
    a_r(1,1)=200;
    // a_r(2,2)=1000;
    alti_Kalman=new Kalman(_x,_p);
    alti_Kalman->A(a_a);
    alti_Kalman->H(a_h);
    alti_Kalman->Q(a_q);
    alti_Kalman->R(a_r);
}
void Drone::Altitude_Kalman(float altitude,float accel){
    a_a(1,1)=1;
    a_a(1,2)=sample_time;
    // a_a(1,3)=sample_time*sample_time/2.;
    a_a(2,2)=1;
    // a_a(2,3)=sample_time;
    // a_a(3,3)=1;
    alti_Kalman->A(a_a);
    a_z(1,1)=altitude;
    // a_z(2,1)=accel*100;
    alti_Kalman->update_z(a_z);
    a_x=alti_Kalman->filtering();
    Altitude=a_x(1,1);
    vel_altitude=a_x(2,1);
    // acl_altitude=a_x(3,1);
    // pc->printf("$%.1f %.1f;\n",Altitude,vel_altitude);
}

void Drone::set_xpos_kalman(){
    Matrix h(2,2),q(2,2),r(2,2);
    Matrix x(2,1),p(2,2);
    p=mm.Eye(2);
    h=mm.Eye(2);
    q(1,1)=1;
    q(2,2)=1;
    r(1,1)=300;
    r(2,2)=1;
    x(1,1)=2.5;
    xpos_Kalman = new Kalman(x,p);
    xpos_Kalman->H(h);
    xpos_Kalman->Q(q);
    xpos_Kalman->R(r);
}

void Drone::set_ypos_kalman(){
    Matrix h(2,2),q(2,2),r(2,2);
    Matrix x(2,1),p(2,2);
    p=mm.Eye(2);
    h=mm.Eye(2);
    q(1,1)=1;
    q(2,2)=1;
    r(1,1)=300;
    r(2,2)=1;
    x(1,1)=2.5;
    ypos_Kalman = new Kalman(x,p);
    ypos_Kalman->H(h);
    ypos_Kalman->Q(q);
    ypos_Kalman->R(r);
}
void Drone::X_Position_Kalman(float x_pos, float x_vel){
    Matrix x(2,1),a(2,2);
    a=mm.Eye(2);
    a(1,2)=sample_time;
    xpos_Kalman->A(a);
    x_z(1,1)=x_pos;
    x_z(2,1)=x_vel;
    xpos_Kalman->update_z(x_z);
    x=xpos_Kalman->filtering();
    Kalman_xpos=x(1,1);
    prev_xpos=x_pos;
}
void Drone::Y_Position_Kalman(float y_pos, float y_vel){
    Matrix x(2,1),a(2,2);
    a=mm.Eye(2);
    a(1,2)=sample_time;
    ypos_Kalman->A(a);
    y_z(1,1)=y_pos;
    y_z(2,1)=y_vel;
    ypos_Kalman->update_z(y_z);
    x=ypos_Kalman->filtering();
    Kalman_ypos=x(1,1);
    prev_ypos=y_pos;
}

void Drone::ESC_calibration(){
    // while(!cali_flag){
    //     wait_us(100000);
    //     }
    pc->printf("Program begin...\n");
    pc->printf("This program will calibrate the ESC.\n");
    motor1.period_us(PULSE_RANGE);
    motor2.period_us(PULSE_RANGE);
    motor3.period_us(PULSE_RANGE);
    motor4.period_us(PULSE_RANGE);

    while (!esc_flag_MAX){
        if(emergency_flag){
            motor1.pulsewidth_us(MIN_PULSEWIDTH);
            motor2.pulsewidth_us(MIN_PULSEWIDTH);
            motor3.pulsewidth_us(MIN_PULSEWIDTH);
            motor4.pulsewidth_us(MIN_PULSEWIDTH);
        }
        wait_us(1000);
    }
    pc->printf("Now writing maximum output.\n");
    pc->printf("Turn on power source, then wait 2 seconds and press any key.\n");
    motor1.pulsewidth_us(MAX_PULSEWIDTH);
    motor2.pulsewidth_us(MAX_PULSEWIDTH);
    motor3.pulsewidth_us(MAX_PULSEWIDTH);
    motor4.pulsewidth_us(MAX_PULSEWIDTH);
    

    while (!esc_flag_MIN){
        if(emergency_flag){
            motor1.pulsewidth_us(MIN_PULSEWIDTH);
            motor2.pulsewidth_us(MIN_PULSEWIDTH);
            motor3.pulsewidth_us(MIN_PULSEWIDTH);
            motor4.pulsewidth_us(MIN_PULSEWIDTH);
        }
        wait_us(1000);
    }
    pc->printf("Sending minimum output\n");
    motor1.pulsewidth_us(MIN_PULSEWIDTH);
    motor2.pulsewidth_us(MIN_PULSEWIDTH);
    motor3.pulsewidth_us(MIN_PULSEWIDTH);
    motor4.pulsewidth_us(MIN_PULSEWIDTH);
}

void Drone::Calibrate_LIDAR(){
    if(Do_Cali_LIDAR == true)
    {
        while(1)
        {
            writeByte(LIDAR_ADD, LIDAR_TX_DATA_1, LIDAR_TX_DATA_2);
            Distance_LIDAR = readByte(LIDAR_ADD, LIDAR_TX_DATA_3);
            OffSet_LIDAR += Distance_LIDAR;
            Count_FOR_CALIBRATION++;
            if(Count_FOR_CALIBRATION %10==0){
                pc->printf(".");
            }
            if(Count_FOR_CALIBRATION == 100)
            {
                pc->printf("\n");
                Count_FOR_CALIBRATION = 0;
                break;
            } 
        }
        OffSet_LIDAR /= 100;
        Do_Cali_LIDAR = false;
        pc->printf("LIDAR calibrated\n");
    }
    else if(Do_Cali_LIDAR == false)
    {
        return;
    }
}

void Drone::get_altitude(){
    writeByte(LIDAR_ADD, LIDAR_TX_DATA_1, LIDAR_TX_DATA_2);
    Distance_LIDAR_NotCalibrated = readByte(LIDAR_ADD, LIDAR_TX_DATA_3);
    Distance_Optical=Distance_LIDAR_NotCalibrated+23.2*tan(BNO080_Roll * DEG_TO_RAD);
    // Distance_LIDAR_NotCalibrated /= 100.0;
    Distance_LIDAR = Distance_LIDAR_NotCalibrated * abs(cos(BNO080_Roll * DEG_TO_RAD)) * abs(cos(BNO080_Pitch * DEG_TO_RAD))+11.6*sin(BNO080_Roll * DEG_TO_RAD);

    // Distance_LIDAR = constrain<float>(Distance_LIDAR, 0, 25);
}

void Drone::get_velocity(){
    flow->readMotion(&deltaY, &deltaX);
    // pc->printf("$%.2f;\n",deltaX);
    // pc->printf("%f\n",sample_time);
    // V_x=4.2*DEGtoRAD*_altitude*0.01*deltaX/(30.0*0.01)+BNO080_Pitch_Rate_RAD*_altitude*0.01;
    // V_y=4.2*DEGtoRAD*_altitude*0.01*deltaY/(30.0*0.01)-BNO080_Roll_Rate_RAD*_altitude*0.01;
    V_x=-4.2*DEGtoRAD*Distance_Optical*0.01*deltaX/(30.0*0.0093)+BNO080_Pitch_Rate_RAD*Distance_Optical*0.01;
    V_y=-4.2*DEGtoRAD*Distance_Optical*0.01*deltaY/(30.0*0.0093)-BNO080_Roll_Rate_RAD*Distance_Optical*0.01;

    // velocity_x=V_x;
    // velocity_y=V_y;
    // if(abs(V_x)>abs(peak_Vx)) peak_Vx=V_x;
    // if(abs(V_y)>abs(peak_Vy)) peak_Vy=V_y;
    // pc->printf("$%6.2f %6.2f;\n", V_x, V_y);
    
}

void Drone::Velocity_LPF(){
    velocity_x=0.035*V_x+0.965*prev_Vx;
    velocity_y=0.035*V_y+0.965*prev_Vy;
    prev_Vx=velocity_x;
    prev_Vy=velocity_y; 
}

void Drone::dt(){
    if(!timer_first_check) timer_first_check=true;
    else{
        sample_time = timer->read();
        timer->reset();
        // pc->printf("%f    \n",sample_time);
    }
}

void Drone::calibrate_IMU(){
    if(Do_Cali_IMU == true)
    {
        while(1)
        {
            if(BNO080_dataAvailable() == 1)
            {

                q[0] = BNO080_getQuatI();
                q[1] = BNO080_getQuatJ();
                q[2] = BNO080_getQuatK();
                q[3] = BNO080_getQuatReal();
                quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
                Quaternion_Update(&q[0]);
                BNO080_Yaw *= -1;
                Offset_GyroX+=BNO080_getGyroX();
                Offset_GyroY+=BNO080_getGyroY();
                Offset_GyroZ+=BNO080_getGyroZ();
                Offset_AccX+=BNO080_getLinAccelX();
                Offset_AccY+=BNO080_getLinAccelY();
                Offset_AccZ+=BNO080_getLinAccelZ();
                // OffSet_Yaw+=BNO080_Yaw;
                if(Count_FOR_CALIBRATION %100==0){
                    pc->printf(".");
                }
                Count_FOR_CALIBRATION++;
            }
            
            if(Count_FOR_CALIBRATION == 1000)
            {
                pc->printf("\n");
                Count_FOR_CALIBRATION = 0;
                break;
            } 
        }
        Offset_GyroX/=1000;
        Offset_GyroY/=1000;
        Offset_GyroZ/=1000;
        Offset_AccX/=1000;
        Offset_AccY/=1000;
        Offset_AccZ/=1000;
        OffSet_Yaw/=1000;
        Do_Cali_IMU = false;
        pc->printf("IMU calibrated\n");
    }
    else if(Do_Cali_IMU == false)
    {
        return;
    }
    
}

void Drone::get_data(){
    // pc->printf("%d\n",BNO080_dataAvailable());
    if (BNO080_dataAvailable() == 1)
    {
        q[0] = BNO080_getQuatI();
        q[1] = BNO080_getQuatJ();
        q[2] = BNO080_getQuatK();
        q[3] = BNO080_getQuatReal();

        
        BNO080_Roll_Rate_RAD = BNO080_getGyroX()-Offset_GyroX;
        BNO080_Roll_Rate_DEG = BNO080_Roll_Rate_RAD * RADtoDEG;
        // BNO080_Roll_Rate_DEG_LPF = 0.9 * BNO080_Roll_Rate_DEG_LPF + 0.1 * BNO080_Roll_Rate_DEG;
        // BNO080_Roll_RAD_Rate_LPF = BNO080_Roll_Rate_DEG_LPF * DEGtoRAD;
        
        // BNO080_Roll_Rate_DEG *= -1;
        
        BNO080_Pitch_Rate_RAD = BNO080_getGyroY()-Offset_GyroY;
        BNO080_Pitch_Rate_DEG = BNO080_Pitch_Rate_RAD * RADtoDEG;
        // BNO080_Pitch_Rate_DEG_LPF = 0.9 * BNO080_Pitch_Rate_DEG_LPF + 0.1 * BNO080_Pitch_Rate_DEG;
        // BNO080_Pitch_RAD_Rate_LPF = BNO080_Pitch_Rate_DEG_LPF * DEGtoRAD;
        // BNO080_Pitch_Rate_DEG *= -1;

        BNO080_Yaw_Rate_RAD = BNO080_getGyroZ()-Offset_GyroZ;
        BNO080_Yaw_Rate_DEG = BNO080_Yaw_Rate_RAD * RADtoDEG;
        BNO080_Yaw_Rate_DEG_LPF = 0.9 * BNO080_Yaw_Rate_DEG_LPF + 0.1 * BNO080_Yaw_Rate_DEG;
        BNO080_Yaw_RAD_Rate_LPF = BNO080_Yaw_Rate_DEG_LPF * DEGtoRAD;
        // BNO080_Yaw_Rate_DEG *= -1;
        // BNO080_Yaw_Rate *= 10;

        quatRadianAccuracy = BNO080_getQuatRadianAccuracy();
        Quaternion_Update(&q[0]);
        // // BNO080_Roll *= -1;
        BNO080_Pitch *= -1;
        BNO080_Yaw *= -1;
        // pc->printf("$%.2f %.2f %.2f %.2f %.2f;\n",q[0],q[1],q[2],q[3],BNO080_Roll);
        // BNO080_Roll -= OffSet_Roll;
        // BNO080_Pitch -= OffSet_Pitch;

        BNO080_Roll_RAD = BNO080_Roll * DEGtoRAD;
        BNO080_Pitch_RAD = BNO080_Pitch * DEGtoRAD;

        BNO080_Yaw -= OffSet_Yaw;

        if (BNO080_Yaw <= 0) BNO080_Yaw = BNO080_Yaw + 360.f;
        BNO080_Yaw_RAD = BNO080_Yaw * DEGtoRAD;

        if (BNO080_Yaw >= 180.0) BNO080_Yaw = BNO080_Yaw - 360.f;
        else if (BNO080_Yaw <= -180.0) BNO080_Yaw = BNO080_Yaw + 360.f;

        if (BNO080_Pitch >= 180.0) BNO080_Pitch = BNO080_Pitch - 360.f;
        else if (BNO080_Pitch <= -180.0) BNO080_Pitch = BNO080_Pitch + 360.f;

        cos_pitch = cos((int)BNO080_Pitch / 180.0 * M_PI);

        cos_roll = cos((int)BNO080_Roll / 180.0 * M_PI);
        
        sin_roll = sin((int)BNO080_Roll / 180.0 * M_PI);
        cos_theta = -sin_pitch;
        sin_theta = -(cos_pitch * sin_roll);

        theta_rad = atan2f(sin_theta, cos_theta);

        Yaw_Real = theta_rad * 180 / M_PI;
        
        LinAcc_X_Body = BNO080_getLinAccelX();
        LinAcc_Y_Body = BNO080_getLinAccelY();
        LinAcc_Z_Body = BNO080_getLinAccelZ();
        // pc->printf("$%.2f %.2f;\n",BNO080_Roll,BNO080_Roll_Rate_DEG);
        getAngle();
        if(!IMU_first_check){
            targetY=Madgwick_Yaw;
            IMU_first_check=true;
        }
        // pc->printf("$%.2f;\n",BNO080_Pitch_Rate_DEG);
    }
    
}

void Drone::getAngle(){
    madgwick.updateFilter(BNO080_Roll_Rate_RAD, BNO080_Pitch_Rate_RAD, BNO080_Yaw_Rate_RAD, LinAcc_X_Body-Offset_AccX, LinAcc_Y_Body-Offset_AccY, LinAcc_Z_Body);
    madgwick.computeEuler();
    Madgwick_Yaw = madgwick.getYaw()*RADtoDEG;
}

void Drone::Median(){
    for(int i=0;i<8;i++){
        GyroX_value[i]=GyroX_value[i+1];
        GyroY_value[i]=GyroY_value[i+1];
        GyroZ_value[i]=GyroZ_value[i+1];
    }
    GyroX_value[8]=BNO080_Roll_Rate_DEG;
    GyroY_value[8]=BNO080_Pitch_Rate_DEG;
    GyroZ_value[8]=BNO080_Yaw_Rate_DEG;
    for(int i=0;i<3;i++){
        sorted_GyroX[i]=GyroX_value[i*4];
        sorted_GyroY[i]=GyroY_value[i*4];
        sorted_GyroZ[i]=GyroZ_value[i*4];
    } 
    sort(sorted_GyroX,sorted_GyroX+3);
    sort(sorted_GyroY,sorted_GyroY+3);
    sort(sorted_GyroZ,sorted_GyroZ+3);
    median_GyroX=sorted_GyroX[1];
    median_GyroY=sorted_GyroY[1];
    median_GyroZ=sorted_GyroZ[1];

    // for(int i=0;i<3;i++){
    //     pc->printf("%.2f  ",GyroX_value[i]);
    // }
    // pc->printf("/  ");
    // for(int i=0;i<3;i++){
    //     pc->printf("%.2f  ",sorted_GyroX[i]);
    // }
    // pc->printf("\n");
}

float Drone::Roll(){
    return BNO080_Roll;
}

float Drone::Pitch(){
    return BNO080_Pitch;
}
float Drone::Yaw(){
    // return BNO080_Yaw;
    return Madgwick_Yaw;
}
float Drone::Roll_R(){
    return BNO080_Roll_RAD;
    // return Madgwick_Roll*DEGtoRAD;
}
float Drone::Pitch_R(){
    return BNO080_Pitch_RAD;
    // return Madgwick_Pitch*DEGtoRAD;
}
float Drone::Yaw_R(){
    return Madgwick_Yaw*DEGtoRAD;
}

float Drone::accelX(){
    LinAcc_X_Earth= LinAcc_X_Body*cos(Pitch_R())+LinAcc_Z_Body*sin(Pitch_R());
    return LinAcc_X_Earth;
}
float Drone::accelY(){
    LinAcc_Y_Earth=LinAcc_X_Body*sin(Roll_R())*sin(Pitch_R())+LinAcc_X_Body*cos(Roll_R())-LinAcc_Z_Body*sin(Roll_R())*cos(Pitch_R());
    return LinAcc_Y_Earth;
}
float Drone::accelZ(){
    LinAcc_Z_Earth=-LinAcc_X_Body*cos(Roll_R())*sin(Pitch_R())+LinAcc_Y_Body*sin(Roll_R())+LinAcc_Z_Body*cos(Roll_R())*cos(Pitch_R());
    return LinAcc_Z_Earth;
}
float Drone::Rrate(){
    return BNO080_Roll_Rate_DEG;
}
float Drone::Prate(){
    return BNO080_Pitch_Rate_DEG;
}
float Drone::Yrate(){
    return BNO080_Yaw_Rate_DEG;
}

void Drone::Run(){
    run=true;
}
void Drone::Stop(){
    run=false;
    
}
void Drone::ESC_false(){
    esc_flag_MIN = false;
    esc_flag_MAX = false;
}

void Drone::ESC_MAX(){
    esc_flag_MAX = true;
}
void Drone::ESC_MIN(){
    esc_flag_MIN = true;
}


float Drone::Roll_PID(float roll,float rate,float velocity,float position){
    if (run){
        // if(!error_flag){
            targetR=-Velocity_y_PID(Position_y_PID(target_ypos, position), velocity);
        // }
        // else{
            // targetR=-Velocity_y_PID(target_yvel, velocity);
        // }
        float error = targetR-roll;
        float target_rate = pid_angle_R.P_control(error);
        PID_R= pid_rate_R.get_PID(target_rate-rate,rate,sample_time)+pid_angle_R.I_control(error, sample_time);
        if(PID_R>PID_MAX) PID_R=PID_MAX;
        else if(PID_R<-PID_MAX) PID_R=-PID_MAX;
        return PID_R;
    }
    else{
        PID_R=0;
        pid_angle_R.reset();
        pid_rate_R.reset();
        pid_y_velocity.reset();
        pid_y_position.reset();
        return 0;
    }
}

float Drone::Pitch_PID(float pitch,float rate,float velocity,float position){
    if (run){
        // if(!error_flag){
            targetP=Velocity_x_PID(Position_x_PID(target_xpos, position), velocity);
        // }
        // else{
            // targetP=Velocity_x_PID(target_xvel, velocity);
        // }
        float error = targetP-pitch;
        // pc->printf("$%.2f;\n",error);
        float target_rate = pid_angle_P.P_control(error);
        PID_P= pid_rate_P.get_PID(target_rate-rate,rate,sample_time)+pid_angle_P.I_control(error, sample_time);
        if(PID_P>PID_MAX) PID_P=PID_MAX;
        else if(PID_P<-PID_MAX) PID_P=-PID_MAX;
        // pid_rate_P.printP();
        // pid_rate_P.printI();
        // pid_rate_P.printD();
        return PID_P;
    }
    else{
        PID_P=0;
        pid_angle_P.reset();
        pid_rate_P.reset();
        pid_x_velocity.reset();
        pid_x_position.reset();
        return 0;
    }
}

// float Drone::Yaw_PID(float yaw,float rate){
//     if (run){
//         float error = 0.0-rate;
//         // float target_rate = pid_angle_Y.P_control(error); 
//         PID_Y= pid_rate_Y.get_PID(error, rate,sample_time);
//         // pid_rate_Y.printD();
//         if(PID_Y>PID_MAX) PID_Y=PID_MAX;
//         else if(PID_Y<-PID_MAX) PID_Y=-PID_MAX;
//         // pc->printf("%f  %f  \n",targetY,yaw);
//         return PID_Y;
//     }
//     else{
//         PID_Y=0;
//         pid_angle_Y.reset();
//         pid_rate_Y.reset();
//         return 0;
//     }
// }

float Drone::Yaw_PID(float yaw,float rate){
    if (run){
        float error = targetY-yaw;
        float target_rate = pid_angle_Y.P_control(error);
        // float target_rate = pid_angle_Y.P_control(error); 
        PID_Y= pid_rate_Y.get_PID(target_rate - rate, rate,sample_time)+pid_angle_Y.I_control(error, sample_time);
        // pid_rate_Y.printD();
        if(PID_Y>PID_MAX) PID_Y=PID_MAX;
        else if(PID_Y<-PID_MAX) PID_Y=-PID_MAX;
        // pc->printf("%f  %f  \n",targetY,yaw);
        return PID_Y;
    }
    else{
        PID_Y=0;
        pid_angle_Y.reset();
        pid_rate_Y.reset();
        IMU_first_check=false;
        return 0;
    }
}

float Drone::Altitude_PID(float target,float altitude,float velocity){
    if(run){
        float error = target-altitude;
        // pc->printf("$%.2f;\n",error);
        float target_vel=pid_altitude.P_control(error);
        PID_A=pid_altitude_vel.get_PID(target_vel-velocity, velocity, sample_time)+pid_altitude.I_control(error, sample_time);
        // pid_altitude_vel.printD();
        // if(PID_A>400) PID_A=400;
        // else if(PID_A<-400) PID_A=-400;
        
        return PID_A;
    }
    else{
        pid_altitude.reset();
        pid_altitude_vel.reset();
        PID_A=0;
        return 0;
    }
}

float Drone::Velocity_x_PID(float target_vel,float velocity){
    float PID_Vx=0;
    if(run){
        float error=target_vel-velocity;
        PID_Vx=pid_x_velocity.get_PID(error, velocity, sample_time);
        if(PID_Vx>25) PID_Vx=25;
        else if(PID_Vx<-25) PID_Vx=-25;
        // pid_x_velocity.printD();
        // pc->printf("x:%f   ",PID_Vx);
        return PID_Vx;
    }
    else{
        pid_x_velocity.reset();
        PID_Vx=0;
        return 0;
    }
}
float Drone::Velocity_y_PID(float target_vel,float velocity){
    float PID_Vy=0;
    if(run){
        float error=target_vel-velocity;
        PID_Vy=pid_y_velocity.get_PID(error, velocity, sample_time);
        if(PID_Vy>25) PID_Vy=25;
        else if(PID_Vy<-25) PID_Vy=-25;
        // pid_y_velocity.printD();
        // pc->printf("y:%f   \n",PID_Vy);
        return PID_Vy;
    }
    else{
        pid_y_velocity.reset();
        PID_Vy=0;
        return 0;
    }
}

float Drone::Position_x_PID(float target_pos,float position){
    float PID_Px=0;
    if(run){
        float error=target_pos-position;
        PID_Px=pid_x_position.get_PID(error, position, sample_time);
        if(PID_Px>0.5) PID_Px=0.5;
        else if(PID_Px<-0.5) PID_Px=-0.5;
        // pid_x_position.printD();
        // pc->printf("y:%f   \n",PID_Vy);
        return PID_Px;
    }
    else{
        pid_x_position.reset();
        PID_Px=0;
        return 0;
    }
}

float Drone::Position_y_PID(float target_pos,float position){
    float PID_Py=0;
    if(run){
        float error=target_pos-position;
        PID_Py=pid_y_position.get_PID(error, position, sample_time);
        if(PID_Py>0.5) PID_Py=0.5;
        else if(PID_Py<-0.5) PID_Py=-0.5;
        // pid_y_position.printD();
        // pc->printf("y:%f   \n",PID_Vy);
        return PID_Py;
    }
    else{
        pid_y_position.reset();
        PID_Py=0;
        return 0;
    }
}

void Drone::ESC_control(){
    // pc->printf("%f   %f    \n",targetR,roll);
    // PID_R=0;
    // PID_P=0;
    // PID_Y=0;
    // PID_A=0;
    PID_R=Roll_PID(BNO080_Roll,median_GyroX,velocity_y,Kalman_ypos);
    PID_P=Pitch_PID(BNO080_Pitch,median_GyroY,velocity_x,Kalman_xpos);
    PID_Y=Yaw_PID(Madgwick_Yaw,median_GyroZ);
    PID_A=Altitude_PID(target_altitude, Altitude,vel_altitude);
    // pc->printf("$%.2f;\n",PID_A);
    
    if(emergency_flag){
        motor1.pulsewidth_us(MIN_PULSEWIDTH);
        motor2.pulsewidth_us(MIN_PULSEWIDTH);
        motor3.pulsewidth_us(MIN_PULSEWIDTH);
        motor4.pulsewidth_us(MIN_PULSEWIDTH);
    }
    else{
        if(run){
            // if(Altitude<30){
            //     pid_angle_P.I_reset();
            //     pid_angle_R.I_reset();
            //     pid_angle_Y.I_reset();
            //     pid_rate_P.I_reset();
            //     pid_rate_R.I_reset();
            //     pid_rate_Y.I_reset();
            // }
            
            // pc->printf("%.2f    ",altitude);
            esc1=Throttle+PID_R+PID_P-PID_Y+PID_A;
            esc2=Throttle-PID_R+PID_P+PID_Y+PID_A;
            esc3=Throttle+PID_R-PID_P+PID_Y+PID_A;
            esc4=Throttle-PID_R-PID_P-PID_Y+PID_A;
            if(esc1<1100) esc1=1100;
            if(esc2<1100) esc2=1100;
            if(esc3<1100) esc3=1100;
            if(esc4<1100) esc4=1100;

            if(esc1>1850) esc1=1850;
            if(esc2>1850) esc2=1850;
            if(esc3>1850) esc3=1850;
            if(esc4>1850) esc4=1850;

            motor1.pulsewidth_us(esc1);
            motor2.pulsewidth_us(esc2);
            motor3.pulsewidth_us(esc3);
            motor4.pulsewidth_us(esc4);
            
            // pc->printf("$%.1f %.1f %.1f %.1f %.2f;\n",esc1,esc2,esc3,esc4,BNO080_Roll);
            // pc->printf(" %.2f  %.2f  %.2f\n",target_altitude,target_xvel,target_yvel);
        }
        else {
            esc1=Throttle;
            esc2=Throttle;
            esc3=Throttle;
            esc4=Throttle;
            motor1.pulsewidth_us(MIN_PULSEWIDTH);
            motor2.pulsewidth_us(MIN_PULSEWIDTH);
            motor3.pulsewidth_us(MIN_PULSEWIDTH);
            motor4.pulsewidth_us(MIN_PULSEWIDTH);
            target_altitude=120;

            target_xpos = Kalman_xpos;
            target_ypos = Kalman_ypos;

            IMU_first_check=false;
            madgwick.reset();
            // pc->printf("%.2f   %.2f   %.2f   %.2f   %.2f \n",targetP,pid_angle_P.kp(),pid_angle_P.ki(),pid_rate_P.kp(),pid_rate_P.ki());
        }
        // pc->printf("  %.1f %.1f %.1f %.1f  \n",esc1,esc2,esc3,esc4);
        // pc->printf("%.1f %.1f %.1f %.1f\n",PID_R,PID_P,PID_Y,PID_A);
    }
}

void Drone::Check_Emergency(){
    // if(check_timer.read_us()>3000000){
    //     emergency_flag = true;
    //     Stop();
    // }
    if(emergency_flag == true){
        pc->printf("Emergency Now!!!!\n");
    }
}

void Drone::writeByte(uint8_t address, uint8_t regAddress, uint8_t data)
{
    char data_write[2];
    data_write[0] = regAddress;
    data_write[1] = data;
    LIDAR->write(address, data_write, 2, 0);
}

float Drone::readByte(uint8_t address, uint8_t regAddress)
{
    char data[2];
    char data_write[1];
    data_write[0] = regAddress;
    LIDAR->write(address, data_write, 1, 1);
    LIDAR->read(address, data, 2, 0);
    float Return_Val = (data[0] << 8) + data[1];
    return Return_Val;
}

template <class T>
T Drone::map(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
template <class A>
A Drone::constrain(A x, A min, A max)
{
    if(x >= max) x = max;
    else if(x < min) x = min;
    return x;
}