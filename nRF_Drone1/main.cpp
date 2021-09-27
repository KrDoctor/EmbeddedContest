//Drone main

#include "mbed.h" 
#include <cmath>
#include <cstdint>
#include <string>
#include "Drone.h"
#include "BNO080.h"
#include "math.h"
///////////////////////////
bool run_okay =false;
bool altitude_okay = false;
bool target_okay = false;
bool stop_okay = false;
int target_okay_count = 0;
int altitude_okay_count = 0;
int stop_okay_count = 0;
///////////////////////////
////////////////////////통신
Thread Xbee_Protocol_Thread;
char inputChar_Xbee;
bool Flag_Start_Get_Xbee=false;
bool Xbee_complete = false;
char Buf_Xbee[50]={0};
int Xbee_Count = 0;
RawSerial Xbee(PC_12, PD_2, 115200);
osThreadId Xbee_Protocol_threadID;
void Check_Xbee_RX();
void Xbee_Protocol();
void Protocol_Solution();
string Xbee_string;
char *c_string_copy;
char *token;
string rec_value;
void XbeePRINT();

double Xbee_x_target;
double Xbee_y_target;
double Xbee_altitude_target;
Ticker target_ticker;
Ticker altitude_target_ticker;
bool altitude_target_finish_flag = false;
bool x_target_finish_flag = false;
bool y_target_finish_flag = false;
void target_ticker_function();
void Altitude_ticker_function();
/////////////////////////////////////////////////////////


Serial pc(USBTX,USBRX,115200);
optical_PMW3901 flow(PB_10);
I2C LIDAR(I2C_SDA,I2C_SCL);
Timer timer;


Drone drone(D2,D3,D9,D10,&pc,&timer,&LIDAR,&flow);

volatile bool int_sensor1 = false, int_sensor2=false;
bool on1=false,on2=false;

static Kalman *kalman=NULL;

Thread Stabilization;
Thread Sensors_thread;
Thread Altitude_Ctrl;
Thread thread_IMU;
Thread thread_Opticalflow;
Thread thread_Laser;
Thread thread_Control;

void Stable();
void Altitude();
void Control();
void IMU();
void PMW();
void PRINT();


bool escape_flag = false;

/////////////////////////////////////////////// 리모컨 용
InterruptIn PPM(PC_5);

Timer PPM_timer;
int i = 0, PPM_init = 0;
uint32_t PPM_all[19], PWM_all[19], CH[7]={0};

// 수신기 값 700~1500
// CH0 = 오른쪽 스틱 위아래     Pitch
// CH1 = 왼쪽 스틱 좌우         Yaw
// CH2 = 왼쪽 스틱 위아래       Throttle
// CH3 = 오른쪽 스틱 좌우       Roll 값 좌우 반전됨
// CH4 = 오른쪽 위 D/R          Stop/Run   700(Stop) 1500(Run)
// CH5 = 왼쪽 위 GEAR           Emergency  700 1500(O)
// CH6 = 오른쪽 앞 FMD          Calibration 700 1100 1500

void PPM_Rise();
void PPM_Fall();
void Check_CH();
template <class T>
    T map(T x, T in_min, T in_max, T out_min, T out_max);
Timer remote_timer;
Timer remote_timer_x;
Timer remote_timer_y;
bool calibration_flag = false;
InterruptIn red_btn(BUTTON1);

void calibration_mode(){
    calibration_flag = true;
}

float target_altitude=70;
uint32_t PPM_signal[7]={0};//신호 잘 받는지 확인용
int stop_count = 0;
////////////////////////////////////////////////리모컨 용


////////////////////////////////////////////////UWB용
RawSerial UWB(PC_10,PC_11,115200);
unsigned long low=0,high=0,uwb_Hex=0;
double anchor_distance[8]={0};
double x_k1,y_k1,z_k1,x_k,y_k,z_k,v_xk_1,v_yk_1,v_zk_1,v_xk,v_yk,v_zk;//x_k1은 x_k+1을 의미하고 v_yk_1은 V_y좌표의 k-1번째를 뜻함
double xi[6]={0,5.127,5.127,0,0,5.127},yi[6]={0,0,5.127,5.127,2.5635,2.5635},zi[6]={1.97,2.105,1.965,1.965,1.985,2.115};//anchor2,3,4,5 4개의 고정 좌표
// double xi[6]={0,6,6,0,0,6},yi[6]={0,0,9,9,4.5,4.5},zi[6]={1.97,1.97,1.965,1.965,2.1,2.115};
bool anchor_exist[6]={false};
int cal_count = 0;
double cal_xi[4]={0}, cal_yi[4]={0}, cal_zi[4]={0};

////////////////////////////gradient decent의 초기값
double first_x=3;
double first_y=4.5;
double first_z=1;
double first_vx=0;
double first_vy=0;
double first_vz=0;
/////////////////////////////
double alpha = 0.1, beta=0.9; //학습률 0.01 0.1  0.05  0.01/41
float lpf_alpha = 0.7;
double drone_past_x = 0, drone_past_y = 0;
int running_time = 50;
float error_diff = 0.0003;//0.001;
float error_diff_case3 = 0.0001;
float alpha_case3 = 0.1;
int uwb_case = 0;
int uwb_check_count = 0;
int uwb_anchor_number_setting = 0;
int uwb_check_case = 0;
int nan_checking = 0;
int check_nan = 0;


double distance_anchor[6]={0};
double cal_distance_anchor[4]={0};
double distance_2D_anchor[4]={0};
double Bk_i_2D(double x_k,double y_k,int i);
double gradient_fx_2D(double *distance_anchor,double x_k,double y_k);
double gradient_fy_2D(double *distance_anchor,double x_k,double y_k);
void distance_2D_Update(double *distance_anchor,double *distance_2D_anchor, float drone_Altitude);
void Trilateration_2D(double *distance_2D_anchor);
float Error_calculation(double *distance_2D_anchor,double x,double y);

void cal_Trilateration_2D(double *distance_2D_anchor);
void cal_distance_2D_Update(double *cal_distance_anchor,double *distance_2D_anchor, float drone_Altitude);
double cal_Bk_i_2D(double x_k,double y_k,int i);
double cal_gradient_fx_2D(double *distance_2D_anchor,double x_k,double y_k);
double cal_gradient_fy_2D(double *distance_2D_anchor,double x_k,double y_k);
double drone_x=0,drone_y=0,drone_z=0;
char inputChar_UWB;
bool UWB_complete=false;
bool Flag_Start_Get_UWB = false;
char Buf_UWB[50] = {0};
int UWB_Count = 0;
void UWB_get_Serial_data();
Thread UWB_Protocol_Thread;
osThreadId UWB_Protocol_threadID;
void UWB_Protocol();
Mutex m1;
float uwb_altitude;
void UWB_calculation();
osThreadId UWB_calculation_threadID;
Thread UWB_calculation_Thread;

int CCount = 0;
bool main_flag = false;
float relative_error = 0;
bool uwb_main_flag = false;
bool print_gain_flag = false;
int emergency_int = 0;
int case_sibal = 0;
unsigned int uwb_thread_check = 0;
unsigned int uwb_interrupt_check = 0;


////////////////////////////////////////////////UWB용

////////////////////////////////////////////////서보모터
PwmOut rc(PA_11);
void turn(PwmOut &rcServo,float deg);
void rc_turn();
Ticker rc_Ticker;
float rc_deg = 55;
////////////////////////////////////////////////서보모터


osThreadId IMU_Protocol_threadID;

int main() {

    rc.period_ms(20);

    thread_IMU.set_priority(osPriorityRealtime);
    thread_Control.set_priority(osPriorityHigh);
    thread_Laser.set_priority(osPriorityAboveNormal);
    thread_Opticalflow.set_priority(osPriorityNormal);
    UWB_Protocol_Thread.set_priority(osPriorityBelowNormal);

    red_btn.fall(&calibration_mode);
    /////////////////////////////통신 용
    Xbee.attach(&Check_Xbee_RX, Serial::RxIrq);


    
    Xbee_Protocol_Thread.start(&Xbee_Protocol);
    /////////////////////////////통신 용
    
    while(calibration_flag == false){
        wait_us(1000);
    }
    m1.lock();
    Xbee.printf("Cali is Ready\n");
    m1.unlock();
    /////////////////////////////리모컨 용
    PPM_timer.start();
    remote_timer.start();
    remote_timer_x.start();
    remote_timer_y.start();
    PPM.enable_irq();
    PPM.rise(&PPM_Rise);
    PPM.fall(&PPM_Fall);
    ///////////////////////////////리모컨 용

    
    // while(true){
    //     for(int i=0;i<7;i++){
    //         printf("%d  ",CH[i]);
    //     }
    //     printf("\n");
    // }

    drone.ESC_calibration();

    // rc_Ticker.attach_us(&rc_turn,1000);
    turn(rc,0);//반대 서보
    // turn(rc,20);

    // IMU 세팅
    BNO080_Initialization();
    BNO080_enableRotationVector(2500); // 2.5ms 
    BNO080_enableGyro(1000);
    BNO080_enableLinearAccelerometer(2500); //2.5ms
    BNO080_calibrateAll();
    // LIDAR 세팅
    LIDAR.frequency(400000);
  
    drone.calibrate_IMU();
    // drone.Calibrate_LIDAR();
    drone.set_alti_kalman();
    drone.set_xpos_kalman();
    drone.set_ypos_kalman();

    if (!flow.begin()) 
    {
        pc.printf("Initialization of the flow sensor failed");
        while(1) { }
    }

    thread_Laser.start(&Altitude);
    thread_IMU.start(&IMU);
    thread_Opticalflow.start(&PMW);
    thread_Control.start(&Control);

    ///////////////////////////////UWB용
    UWB.attach(&UWB_get_Serial_data,Serial::RxIrq);
    UWB_Protocol_Thread.start(&UWB_Protocol);
    //////////////////////////////UWB용
    m1.lock();
    Xbee.printf("Drone1 Ready\n");
    m1.unlock();

    

    // Communication_drone.start(callback(&drone,&Drone::Command_drone_thread)); // 드론 하나만 쓸거면 주석처리 할것
    
    while(1) {
        drone.Check_Emergency();
        // pc.printf("%4.4d %4.4d %4.4d %4.4d %4.4d %4.4d\n",remote_THRO,remote_ELEV,remote_AILE,remote_GEAR,remote_AUX1,remote_AUX2);
        // for(int i =0;i<7;i++){
        //     pc.printf("%d   ",PPM_signal[i]);
        // }
        // pc.printf("\n");
        ///////////////////////////////////////////원터치 기능
        if(run_okay==true){
            if(abs(drone.Altitude-drone.target_altitude)<3){// 처음에는 기본적으로 drone.target_altitude가 120으로 되어 있음.
                altitude_okay_count++;
            }
        }
        if(altitude_okay_count>15){
            run_okay=false;
            altitude_okay=true;
            target_ticker.attach(&target_ticker_function,0.14);//목표지점으로 가기 시작
            altitude_okay_count = 0;
        }
        if(altitude_okay==true){
            if(abs(drone.Kalman_xpos-Xbee_x_target)<0.2 && abs(drone.Kalman_ypos-Xbee_y_target)<0.2){
                target_okay_count++;
            }
        }
        if(target_okay_count>15){
            altitude_okay=false;
            target_okay = true;
            Xbee_altitude_target = 28;
            altitude_target_finish_flag = false;
            target_ticker.attach(&Altitude_ticker_function,0.14);
            target_okay_count = 0;
        }
        if(target_okay == true){
            if(abs(drone.Altitude-Xbee_altitude_target)<3){
                stop_okay_count++;
            }
        }
        if(stop_okay_count>15){
            target_okay = false;
            drone.Stop();
            stop_okay_count = 0;
        }

        //////////////////////////////////////////원터치 기능
        if(main_flag==true){
            m1.lock();
            XbeePRINT();
            m1.unlock();
        }
        float a =3.457;
        float b =4.3127;

        if(uwb_main_flag==true){
            m1.lock();
            // Xbee.printf("X %2.4f, Y %2.4f  %d  %f\n",drone.Kalman_xpos,drone.Kalman_ypos,CCount,relative_error);
            // Xbee.printf("$%.3f  %.3f;\n",drone.Kalman_xpos,drone.Kalman_ypos);
            Xbee.printf("<drone1/%.3f/%.3f/%.3f/%.3f>\n",drone.Kalman_xpos,drone.target_xpos,drone.Kalman_ypos,drone.target_ypos);
            // pc.printf("x = %.3f     y = %.3f\n",drone.target_xpos,drone.target_ypos);
            // Xbee.printf("$%d;\n",case_sibal);
            m1.unlock();
        }
        if(print_gain_flag==true){
            m1.lock();
            Xbee.printf("x/ %.3f / %.3f / %.3f\ny/ %.3f / %.3f / %.3f\nVx/ %.3f / %.3f / %.3f\nVy/ %.3f / %.3f / %.3f\n"
            ,drone.pid_x_position.kp(),drone.pid_x_position.ki(),drone.pid_x_position.kd()
            ,drone.pid_y_position.kp(),drone.pid_y_position.ki(),drone.pid_y_position.kd()
            ,drone.pid_x_velocity.kp(),drone.pid_x_velocity.ki(),drone.pid_x_velocity.kd()
            ,drone.pid_y_velocity.kp(),drone.pid_y_velocity.ki(),drone.pid_y_velocity.kd());
            m1.unlock();
            print_gain_flag = false;
        }
        if(emergency_int !=0){Xbee.printf("%d\n",emergency_int);}
        
        ThisThread::sleep_until(Kernel::get_ms_count()+100);
    }
}

Timer test_timer;
int cnt=0;

float vel[5000],acl[5000],t[5000];
void Control(){
    
    timer.start();
    // test_timer.start();
    float Control_sample = 0.001;
    while(true){
        uint64_t current_Tick = Kernel::get_ms_count();
        drone.dt();
        drone.Median();
        drone.Altitude_Kalman(drone.altitude(),drone.accelZ());
        drone.Velocity_LPF();
        drone.X_Position_Kalman(drone.x_position,drone.velocity_x);
        drone.Y_Position_Kalman(drone.y_position,drone.velocity_y);
        drone.ESC_control();
        // Xbee.printf("$%.2f;\r\n",drone.Altitude);
        // if(cnt<5000){
        //     vel[cnt]=drone.x_velocity;
        //     acl[cnt]=drone.accelX();
        //     t[cnt]=test_timer.read();
        //     pc.printf("%d\n",cnt);
        //     cnt++;
        // }
        // else if(cnt==5000){
        //     for(int i=0;i<5000;i++){
        //         pc.printf("%f\n",vel[i]);
        //     }
        //     pc.printf("\n\n\n");
        //     for(int i=0;i<5000;i++){
        //         pc.printf("%f\n",acl[i]);
        //     }
        //     pc.printf("\n\n\n");
        //     for(int i=0;i<5000;i++){
        //         pc.printf("%f\n",t[i]);
        //     }
        //     pc.printf("\n\n\n");
        //     cnt++;
        // }
        PRINT();
        uint64_t working_Tick = Kernel::get_ms_count();
        // pc.printf("Working time: %llu\n", working_Tick - current_Tick);
        ThisThread::sleep_until(Kernel::get_ms_count()+(Control_sample*1000-(working_Tick-current_Tick)));
        uint64_t last_Tick = Kernel::get_ms_count();
        // pc.printf("Total time: %llu\n", last_Tick - current_Tick);
   }

}
// Timer IMU_timer;
// int IMU_fuck = 0;

void IMU(){
    float IMU_sample=0.001;
    test_timer.start();
    // IMU_timer.start();
    while(true){
        uint64_t current_Tick = Kernel::get_ms_count();
        drone.get_data();
        // pc.printf("$%.1f %.1f;\n",drone.targetP,drone.Pitch());
        // while(IMU_timer.read_us()<1000){

        // }
        uint64_t working_Tick = Kernel::get_ms_count();
        // pc.printf("%f   ",test_timer.read());
        // pc.printf("Working time: %llu\n", working_Tick - current_Tick);
        ThisThread::sleep_until(Kernel::get_ms_count()+((IMU_sample*1000)-(working_Tick-current_Tick)));
        // ThisThread::sleep_until(Kernel::get_ms_count()-1);
        // ThisThread::sleep_for(1);
        // ThisThread::sleep_until(Kernel::get_ms_count());
        // ThisThread::sleep_for(IMU_sample*1000-(working_Tick-current_Tick));
        uint64_t last_Tick = Kernel::get_ms_count();
        // pc.printf("%f\n",test_timer.read());
        test_timer.reset();
        // IMU_timer.reset();
        // pc.printf("Total time: %llu\n", last_Tick - current_Tick);
    }
}
// Timer Alti_timer;
Timer test_timer2;
// int Alti_fuck = 0;
void Altitude(){
    float laser_sample = 0.002;
    // Alti_timer.start();
    test_timer2.start();
    while(true){
        uint64_t current_Tick = Kernel::get_ms_count();
        drone.get_altitude();

        // while(Alti_timer.read_us()<2000){

        // }

        uint64_t working_Tick = Kernel::get_ms_count();
        // pc.printf("Working time: %llu\n", working_Tick - current_Tick);
        ThisThread::sleep_until(Kernel::get_ms_count()+(laser_sample*1000-(working_Tick-current_Tick)));
        // ThisThread::sleep_until(Kernel::get_ms_count()-1);
        uint64_t last_Tick = Kernel::get_ms_count();
        // pc.printf("%f\n",test_timer2.read());
        // Alti_timer.reset();
        test_timer2.reset();
        // pc.printf("Total time: %llu\n", last_Tick - current_Tick);
    }
}

// Timer PWM_timer;
Timer test_timer3;
// int PWM_fuck = 0;
void PMW(){
    float flow_sample=0.01;
    // PWM_timer.start();
    test_timer3.start();
    while(true){
        uint64_t current_Tick = Kernel::get_ms_count();
        drone.get_velocity();
        // while(PWM_timer.read_us()<10000){

        // }
        uint64_t working_Tick = Kernel::get_ms_count();
        // pc.printf("Working time: %llu\n", working_Tick - current_Tick);
        ThisThread::sleep_until(Kernel::get_ms_count()+(flow_sample*1000-(working_Tick-current_Tick)));
        uint64_t last_Tick = Kernel::get_ms_count();
        // pc.printf("%f\n",test_timer3.read());
        // PWM_timer.reset();
        test_timer3.reset();
        // pc.printf("Total time: %llu\n", last_Tick - current_Tick);
    }
}
void XbeePRINT(){
    // Xbee.printf("X %2.4f, Y %2.4f, Z %2.4f,  %d\n",drone.Kalman_xpos,drone.Kalman_ypos,uwb_altitude,CCount);
    // Xbee.printf("$%.5f  %.5f  %.5f  %.5f;\n",drone_x*100,drone_y*100,drone.target_xpos*100,drone.target_ypos*100);
    // Xbee.printf("$%.3f %.3f %.2f;\n",drone.Kalman_xpos,drone.Kalman_ypos,drone.Altitude);
    // Xbee.printf("$%.3f %.3f;\n",drone.Kalman_xpos,drone.target_xpos);
    // Xbee.printf("$%.3f %.3f;\n",drone.Kalman_ypos,drone.target_ypos);
    // Xbee.printf("$%.3f %.3f %.3f %.3f %d %d;\n",drone.Kalman_xpos,drone.target_xpos,drone.Kalman_ypos,drone.target_ypos,uwb_case,case_sibal);
    // Xbee.printf("$%.3f %.3f %.3f %.3f;\n",drone.Kalman_xpos,drone.target_xpos,drone.Kalman_ypos,drone.target_ypos);
    Xbee.printf("<drone1/%.3f/%.3f/%.3f/%.3f>\n",drone.Kalman_xpos,drone.target_xpos,drone.Kalman_ypos,drone.target_ypos);
    // Xbee.printf("$%d;\n",case_sibal);
    // Xbee.printf("%.2f   %.2f   %.2f\n",drone.target_altitude,drone.target_xpos,drone.target_ypos);
    // Xbee.printf("$%.3f %.3f;\n",drone.Altitude,drone.target_altitude);
    // Xbee.printf("$%.1f %.1f %.1f %.1f;\n",drone.esc1,drone.esc2,drone.esc3,drone.esc4);
    // Xbee.printf("$%.3f %.3f %.3f %.3f;\n",drone.velocity_x,drone.target_xvel,drone.velocity_y,drone.target_yvel);
    // Xbee.printf("$%.3f %.3f %.3f %.3f;\n",drone.Roll(),drone.targetR,drone.Pitch(),drone.targetP);
    // Xbee.printf("$%.3f %.3f;\n",drone.Yaw(),drone.targetY);
}

void PRINT(){
    // pc.printf("$%.2f;\n",drone.accelX());
    // pc.printf("$%.2f;\n",drone.Rrate());
    // pc.printf("$%.1f %.1f %.2f;\n",drone.targetP,drone.Pitch(),drone.Prate());
    // pc.printf("$%.1f %.1f %.2f %.2f;\n",drone.targetP,drone.Pitch(),drone.Prate(),drone.median_GyroY);
    // pc.printf("$%.1f %.1f;\n",drone.targetR,drone.Roll());
    // pc.printf("$%.1f %.1f;\n",drone.targetP,drone.Pitch());
    // pc.printf("$%.1f %.1f;\n",drone.targetY,drone.Yaw());
    // pc.printf("$%.2f %.2f;\n",drone.Yaw(),drone.Yrate());
    // pc.printf("$%.2f %.2f;\n",drone.Roll(),drone.Pitch());
    // pc.printf("$%.2f %.2f %.2f %.2f;\n",drone.Roll(),drone.targetR,drone.Pitch(),drone.targetP);
    // pc.printf("$%.2f %.2f %.2f;\n",drone.Roll(),drone.Pitch(),drone.Yaw());
    // pc.printf("$%.2f %.2f %.2f;\n",drone.Rrate(),drone.Prate(),drone.Yrate());
    // pc.printf("$%.2f %.2f;\n",drone.Roll(),drone.Rrate());
    // pc.printf("$%.2f;\n",drone.Roll());
    // pc.printf("$%.2f;\n",drone.altitude());
    // pc.printf("$%.2f;\n",drone.Altitude);
    // pc.printf("\n");
    // pc.printf("$%.3f %.3f;\n",drone.velocity_x,drone.velocity_y);
    // pc.printf("$%.1f %.1f;\n",drone.targetR,drone.targetP);
    // pc.printf("%.2f  %.2f  %.2f\n",drone.target_altitude,drone.target_xvel,drone.target_yvel);
    // pc.printf("%.2f         %d\n",drone.target_altitude,CH[2]);
    // pc.printf("$%.2f %.2f;\n",drone.targetR,drone.targetP);
    // pc.printf("$%.2f %.2f;\n",drone.deltaX,drone.deltaY);
    // pc.printf("$%f %f;\n",drone.x_position,drone.y_position);
    // pc.printf("$%.2f %.2f %d %d;\n",drone.Kalman_xpos,drone.Kalman_ypos,uwb_case,case_sibal);
    // pc.printf("$%.2f %.2f;\n",drone.Kalman_xpos,drone.Kalman_ypos);
    // pc.printf("$%.2f %.2f %d %d;\n",drone_x,drone_y,uwb_case,CCount);
    // pc.printf("$%.2f;\n",drone.Kalman_xpos);
    // for(int i=0;i<7;i++){
    //     pc.printf("%4d    ",PPM_signal[i]);
    // }
    // pc.printf("\n");
    // pc.printf("$%.2f %.2f;\n",drone.Prate(),drone.median_GyroY);
    // pc.printf("$%.3f %.3f;\n",drone.target_xpos,drone.target_ypos);
}





//////////////////////////////////////리모컨용

template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PPM_Rise(){
    PPM_timer.reset();
}
void PPM_Fall(){
    PPM_all[i] = PPM_timer.read_us();
    i++;
    if(i==17){
        for(int k=18;k>-1;k--){
            if(PPM_all[k]>10000){
                PPM_init = k;
            }
        }
        for(int k=0;k<7;k++){
            CH[k]=PPM_all[PPM_init+k+1];
        }
        i = 0;
        Check_CH();
    }
}
void Check_CH(){
    for(int k=0;k<7;k++){
            if(CH[k]<700)CH[k]=700;
            if(CH[k]>1500)CH[k]=1500;
        }
    
    for(int i =0;i<7;i++){
            PPM_signal[i]=CH[i];
        }
    //////////Throttle
    
    if(CH[2]>=700 && CH[2]<=964 && remote_timer.read_ms()>100){
        remote_timer.reset();
        drone.target_altitude -= 1;
        if(drone.target_altitude<23) drone.target_altitude=23;
    }
    else if(CH[2]>1228 && CH[2]<=1500 && remote_timer.read_ms()>100){
        remote_timer.reset();
        drone.target_altitude += 1;
        if(drone.target_altitude>180) drone.target_altitude=180;
    }

    // drone.Throttle=map<float>(CH[2], 700, 1500, 1100, 1550);
    
    // if(CH[2]>=700 && CH[2]<=1500){
    //     drone.Throttle=map<float>(CH[2], 700, 1500, 1100, 1600);
    // }
    // else if(CH[2]>964 && CH[2]<=1228){
    //     drone.target_altitude = target_altitude;
    // }

    ///////////Roll 현재는 x속도 조절용
    

    // if(CH[3]>=1050&&CH[3]<1150){
    //     drone.targetR = 0;
    // }else{
    //     drone.targetR = map<float>(CH[3], 700, 1500, 10, -10);
    //     // drone.target_xvel = CH[3];
    // }

//////////////////////////////////////////////////////////////////x속도
    // if(CH[3]<=1050){
    //     drone.target_xvel = map<float>(CH[3], 700, 1500, 1, -1);
    // }else if(CH[3]<=1150){
    //     drone.target_xvel = 0;
    // }else if(CH[3]<=1500){
    //     drone.target_xvel = map<float>(CH[3], 700, 1500, 1, -1);
    // }
//////////////////////////////////////////////////////////////////x속도
//////////////////////////////////////////////////////////////////x좌표

    if(CH[3]>=700 && CH[3]<=964 && remote_timer_x.read_ms()>100){
        remote_timer_x.reset();
        drone.target_xpos += 0.05;
    }
    else if(CH[3]>1228 && CH[3]<=1500 && remote_timer_x.read_ms()>100){
        remote_timer_x.reset();
        drone.target_xpos -= 0.05;
    }
//////////////////////////////////////////////////////////////////x좌표

    ///////////Pitch 현재는 y속도 조절용
    // if(CH[0]>=1050&&CH[0]<1150){
    //     drone.targetP = 0;
    // }else{
    //     drone.targetP = map<float>(CH[0], 700, 1500, -10, 10);
    //     // drone.target_yvel = CH[0];
    // }
//////////////////////////////////////////////////////////////////y속도
    // if(CH[0]<=1050){
    //     drone.target_yvel = map<float>(CH[0], 700, 1500, -1, 1);
    // }else if(CH[0]<=1150){
    //     drone.target_yvel = 0;
    // }else if(CH[0]<=1500){
    //     drone.target_yvel = map<float>(CH[0], 700, 1500, -1, 1);
    // }
//////////////////////////////////////////////////////////////////y속도


    if(CH[0]>=700 && CH[0]<=964 && remote_timer_y.read_ms()>100){
        remote_timer_y.reset();
        drone.target_ypos -= 0.05;
    
    }
    else if(CH[0]>1228 && CH[0]<=1500 && remote_timer_y.read_ms()>100){
        remote_timer_y.reset();
        drone.target_ypos += 0.05;
    }


    //////////////////////////////////////////////////////////////////x좌표

    // if(!drone.error_flag){
    //     //////////////////////////////////////////////////////////////////x좌표
    //     if(CH[3]>=700 && CH[3]<=964 && remote_timer_x.read_ms()>100){
    //         remote_timer_x.reset();
    //         drone.target_xpos += 0.05;
    //     }
    //     else if(CH[3]>1228 && CH[3]<=1500 && remote_timer_x.read_ms()>100){
    //         remote_timer_x.reset();
    //         drone.target_xpos -= 0.05;
    //     }
    //     //////////////////////////////////////////////////////////////////x좌표
    // }
    // else{
    //     //////////////////////////////////////////////////////////////////x속도
    //     if(CH[3]<=1050){
    //         drone.target_xvel = map<float>(CH[3], 700, 1500, 1, -1);
    //     }else if(CH[3]<=1150){
    //         drone.target_xvel = 0;
    //     }else if(CH[3]<=1500){
    //         drone.target_xvel = map<float>(CH[3], 700, 1500, 1, -1);
    //     }
    //     //////////////////////////////////////////////////////////////////x속도
    // }


    ///////////Pitch 현재는 y속도 조절용
    // if(CH[0]>=1050&&CH[0]<1150){
    //     drone.targetP = 0;
    // }else{
    //     drone.targetP = map<float>(CH[0], 700, 1500, -10, 10);
    //     // drone.target_yvel = CH[0];
    // }

    // if(!drone.error_flag){
    //     //////////////////////////////////////////////////////////////////y좌표
    //     if(CH[0]>=700 && CH[0]<=964 && remote_timer_y.read_ms()>100){
    //         remote_timer_y.reset();
    //         drone.target_ypos -= 0.05;
    //     }
    //     else if(CH[0]>1228 && CH[0]<=1500 && remote_timer_y.read_ms()>100){
    //         remote_timer_y.reset();
    //         drone.target_ypos += 0.05;
    //     }
    //     //////////////////////////////////////////////////////////////////y좌표
    // }
    // else{
    //     //////////////////////////////////////////////////////////////////y속도
    //     if(CH[0]<=1050){
    //         drone.target_yvel = map<float>(CH[0], 700, 1500, -1, 1);
    //     }else if(CH[0]<=1150){
    //         drone.target_yvel = 0;
    //     }else if(CH[0]<=1500){
    //         drone.target_yvel = map<float>(CH[0], 700, 1500, -1, 1);
    //     }
    //     //////////////////////////////////////////////////////////////////y속도
    // }

    //////////Stop/Run
    //현재 MIX버튼인데 값이 롤처럼 반전됨
    if(CH[6]>900){
        drone.Run();
        main_flag = true;
        stop_count = 0;
    }else if(CH[6]<900){
        stop_count++;
        if(stop_count>=7)drone.Stop();
        main_flag = false;
    }
    // if(CH[6]>900){
    //     drone.Run();
    //     main_flag = true;
       
    // }else if(CH[6]<900){
    //     drone.Stop();
    //     main_flag = false;
    // }

    ////////Emergency
    // if(CH[4]>1100){
    //     drone.emergency_flag = true;
    //     emergency_int = 1;
    // }

    ///////Calibration
    if(CH[5]<964){
        drone.ESC_false();
    }else if(CH[5]<1228){
        drone.ESC_MAX();
    }else if(CH[5]<1600){
        drone.ESC_MIN();
    }
}
/////////////////////////////////////////////////리모컨용



//////////////////통신

void Xbee_Protocol(){

    Xbee_Protocol_threadID = osThreadGetId();

    while(true){
        osEvent sig = osSignalWait(1, osWaitForever);

        Xbee_string=Buf_Xbee;
        Xbee_string = Xbee_string.substr(1,Xbee_Count-2);
        c_string_copy = &Xbee_string[0];
        // pc.printf("%s \n",c_string_copy);

        Protocol_Solution();

        Xbee_Count = 0;
        memset(Buf_Xbee, 0, sizeof(Buf_Xbee));
        Xbee_complete = false;
    }
}

void Check_Xbee_RX()    //
{
    if(Xbee.readable())
    {
        inputChar_Xbee = Xbee.getc();
        // if(inputChar_Xbee == 'e') {drone.emergency_flag = true; emergency_int = 2;}
        if(inputChar_Xbee == '<' && !Xbee_complete) Flag_Start_Get_Xbee = true;
        if(Flag_Start_Get_Xbee)
        {
            Buf_Xbee[Xbee_Count++] = inputChar_Xbee;

            if(inputChar_Xbee == '>')
            {
                Flag_Start_Get_Xbee = false;
                Xbee_complete = true;
                
                osSignalSet(Xbee_Protocol_threadID, 1);
            }

        }
    }
}

void Protocol_Solution(){
    token = strtok(c_string_copy,"/");
    rec_value = token;

    // pc.printf("%s \n",rec_value.c_str());
    if(rec_value == "e"){// 긴급 상황 즉시 착륙요망
        drone.Stop();
        drone.emergency_flag = true;
        emergency_int = 3;
    }else if(rec_value=="0"){
        drone.Stop();
    }else if(rec_value=="1"){
        drone.Run();
    }else if(rec_value =="esc_max"){// ESC 캘리브레이션 Max 하는 코드
        drone.ESC_MAX();
        drone.check_timer.start();
    }else if(rec_value =="esc_min"){// ESC 캘리브레이션 min 하는 코드
        drone.ESC_MIN();
    }else if(rec_value =="R"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="Ap"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_R.kp(stof(rec_value));
            Xbee.printf(" R/Ap : %f\n",drone.pid_angle_R.kp());
        }else if(rec_value =="Ai"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_R.ki(stof(rec_value));
            Xbee.printf(" R/Ai : %f\n",drone.pid_angle_R.ki());
        }else if(rec_value =="Ad"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_R.kd(stof(rec_value));
            Xbee.printf(" R/Ad : %f\n",drone.pid_angle_R.kd());
        }else if(rec_value =="Rp"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_R.kp(stof(rec_value));
            Xbee.printf(" R/Rp : %f\n",drone.pid_rate_R.kp());
        }else if(rec_value =="Ri"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_R.ki(stof(rec_value));
            Xbee.printf(" R/Ri : %f\n",drone.pid_rate_R.ki());
        }else if(rec_value =="Rd"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_R.kd(stof(rec_value));
            Xbee.printf(" R/Rd : %f\n",drone.pid_rate_R.kd());
        }
    }else if(rec_value =="P"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="Ap"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_P.kp(stof(rec_value));
            Xbee.printf(" P/Ap : %f\n",drone.pid_angle_P.kp());
        }else if(rec_value =="Ai"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_P.ki(stof(rec_value));
            Xbee.printf(" P/Ai : %f\n",drone.pid_angle_P.ki());
        }else if(rec_value =="Ad"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_P.kd(stof(rec_value));
            Xbee.printf(" P/Ad : %f\n",drone.pid_angle_P.kd());
        }else if(rec_value =="Rp"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_P.kp(stof(rec_value));
            Xbee.printf(" P/Rp : %f\n",drone.pid_rate_P.kp());
        }else if(rec_value =="Ri"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_P.ki(stof(rec_value));
            Xbee.printf(" P/Ri : %f\n",drone.pid_rate_P.ki());
        }else if(rec_value =="Rd"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_P.kd(stof(rec_value));
            Xbee.printf(" P/Rd : %f\n",drone.pid_rate_P.kd());
        }
    }else if(rec_value =="Y"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="Rp"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_Y.kp(stof(rec_value));
            Xbee.printf(" Y/Rp : %f\n",drone.pid_rate_Y.kp());
        }else if(rec_value =="Ri"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_Y.ki(stof(rec_value));
            Xbee.printf(" Y/Ri : %f\n",drone.pid_rate_Y.ki());
        }else if(rec_value =="Rd"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_rate_Y.kd(stof(rec_value));
            Xbee.printf(" Y/Rd : %f\n",drone.pid_rate_Y.kd());
        }else if(rec_value =="Ap"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_Y.kp(stof(rec_value));
            Xbee.printf(" Y/Ap : %f\n",drone.pid_angle_Y.kp());
        }else if(rec_value =="Ai"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_Y.ki(stof(rec_value));
            Xbee.printf(" Y/Ai : %f\n",drone.pid_angle_Y.ki());
        }else if(rec_value =="Ad"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_angle_Y.kd(stof(rec_value));
            Xbee.printf(" Y/Ad : %f\n",drone.pid_angle_Y.kd());
        }
    }else if(rec_value=="A"){
        token=strtok(NULL,"/");
        rec_value=token;
        if(rec_value=="p"){
            token=strtok(NULL,"/");
            rec_value=token;
            drone.pid_altitude.kp(stof(rec_value));
            Xbee.printf(" A/p : %f\n",drone.pid_altitude.kp());
        }
        else if(rec_value=="i"){
            token=strtok(NULL,"/");
            rec_value=token;
            drone.pid_altitude.ki(stof(rec_value));
            Xbee.printf(" A/i : %f\n",drone.pid_altitude.ki());
        }
        else if(rec_value=="Vp"){
            token=strtok(NULL,"/");
            rec_value=token;
            drone.pid_altitude_vel.kp(stof(rec_value));
            Xbee.printf(" A/Vp : %f\n",drone.pid_altitude_vel.kp());
        }
        else if(rec_value=="Vi"){
            token=strtok(NULL,"/");
            rec_value=token;
            drone.pid_altitude_vel.ki(stof(rec_value));
            Xbee.printf(" A/Vi : %f\n",drone.pid_altitude_vel.ki());
        }
        else if(rec_value=="Vd"){
            token=strtok(NULL,"/");
            rec_value=token;
            drone.pid_altitude_vel.kd(stof(rec_value));
            Xbee.printf(" A/Vd : %f\n",drone.pid_altitude_vel.kd());
        }
    }else if(rec_value =="Vx"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="p"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_velocity.kp(stof(rec_value));
            Xbee.printf(" Vx/p : %f\n",drone.pid_x_velocity.kp());
        }else if(rec_value =="i"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_velocity.ki(stof(rec_value));
            Xbee.printf(" Vx/i : %f\n",drone.pid_x_velocity.ki());
        }else if(rec_value =="d"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_velocity.kd(stof(rec_value));
            Xbee.printf(" Vx/d : %f\n",drone.pid_x_velocity.kd());
        }
    }else if(rec_value =="Vy"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="p"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_velocity.kp(stof(rec_value));
            Xbee.printf(" Vy/p : %f\n",drone.pid_y_velocity.kp());
        }else if(rec_value =="i"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_velocity.ki(stof(rec_value));
            Xbee.printf(" Vy/i : %f\n",drone.pid_y_velocity.ki());
        }else if(rec_value =="d"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_velocity.kd(stof(rec_value));
            Xbee.printf(" Vy/d : %f\n",drone.pid_y_velocity.kd());
        }
    }else if(rec_value =="uwb"){
        m1.lock();
        UWB.printf("%c",3);
        m1.unlock();
    }else if(rec_value =="show"){
        uwb_main_flag = true;
    }else if(rec_value =="done"){
        uwb_main_flag = false;
    }else if(rec_value =="running_time"){
        token = strtok(NULL,"/");
        rec_value = token;
        running_time = stoi(rec_value);
        m1.lock();
        Xbee.printf("running_time %d\n",running_time);
        m1.unlock();
    }else if(rec_value =="alpha"){
        token = strtok(NULL,"/");
        rec_value = token;
        alpha = stof(rec_value);
        m1.lock();
        Xbee.printf("alpha %f\n",alpha);
        m1.unlock();
    }else if(rec_value =="beta"){
        token = strtok(NULL,"/");
        rec_value = token;
        beta = stof(rec_value);
    }else if(rec_value =="error_diff"){
        token = strtok(NULL,"/");
        rec_value = token;
        error_diff = stof(rec_value);
        m1.lock();
        Xbee.printf("error_diff %f\n",error_diff);
        m1.unlock();
    }else if(rec_value =="error_diff_case3"){
        token = strtok(NULL,"/");
        rec_value = token;
        error_diff_case3 = stof(rec_value);
        m1.lock();
        Xbee.printf("error_diff_case3 %f\n",error_diff_case3);
        m1.unlock();
        
    }else if(rec_value =="alpha_case3"){
        token = strtok(NULL,"/");
        rec_value = token;
        alpha_case3 = stof(rec_value);
        m1.lock();
        Xbee.printf("alpha_case3 %f\n",alpha_case3);
        m1.unlock();
    }
    
    else if(rec_value =="lpf_alpha"){
        token = strtok(NULL,"/");
        rec_value = token;
        lpf_alpha = stof(rec_value);
    }else if(rec_value =="gain"){
        print_gain_flag = true;
    }else if(rec_value =="x"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="p"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_position.kp(stof(rec_value));
            Xbee.printf(" x/p : %f\n",drone.pid_x_position.kp());
        }else if(rec_value =="i"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_position.ki(stof(rec_value));
            Xbee.printf(" x/i : %f\n",drone.pid_x_position.ki());
        }else if(rec_value =="d"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_x_position.kd(stof(rec_value));
            Xbee.printf(" x/d : %f\n",drone.pid_x_position.kd());
        }
    }else if(rec_value =="y"){
        token = strtok(NULL,"/");
        rec_value = token;
        if(rec_value =="p"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_position.kp(stof(rec_value));
            Xbee.printf(" y/p : %f\n",drone.pid_y_position.kp());
        }else if(rec_value =="i"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_position.ki(stof(rec_value));
            Xbee.printf(" y/i : %f\n",drone.pid_y_position.ki());
        }else if(rec_value =="d"){
            token = strtok(NULL,"/");
            rec_value = token;
            drone.pid_y_position.kd(stof(rec_value));
            Xbee.printf(" y/d : %f\n",drone.pid_y_position.kd());
        }
    }else if(rec_value =="cali_on"){
        calibration_flag = true;
    }else if(rec_value =="grap"){
        turn(rc,90);//반대서보
        // turn(rc,150);
        // rc_deg = 180;
        m1.lock();
        Xbee.printf("Grap!\n");
        m1.unlock();
    }else if(rec_value =="position"){
        // rc_deg = 180;
        m1.lock();
        Xbee.printf("Update position!\n");
        m1.unlock();
    }
    else if(rec_value =="release"){
        turn(rc,0);//반대서보
        // turn(rc,20);
        // rc_deg = 0;
        m1.lock();
        Xbee.printf("Release!\n");
        m1.unlock();
    }else if(rec_value =="Grap"){
        
        token = strtok(NULL,"/");
        rec_value = token;
        // rc_deg = stof(rec_value);
        turn(rc,stof(rec_value));

        
    }else if(rec_value =="target"){
        token = strtok(NULL,"/");
        rec_value = token;
        Xbee_x_target = stof(rec_value);
        token = strtok(NULL,"/");
        rec_value = token;
        Xbee_y_target=stof(rec_value);

        x_target_finish_flag = false;
        y_target_finish_flag = false;
        // target_ticker.attach(&target_ticker_function,0.14);// 원터치 안하면 키세요
        // pc.printf("업뎃함\n");
        
    }else if(rec_value =="down"){
        Xbee_altitude_target = 23;
        altitude_target_finish_flag = false;
        target_ticker.attach(&Altitude_ticker_function,0.14);
    }else if(rec_value =="altitude"){
        token = strtok(NULL,"/");
        rec_value = token;
        Xbee_altitude_target = stof(rec_value);
        altitude_target_finish_flag = false;
        target_ticker.attach(&Altitude_ticker_function,0.14);
    }else if(rec_value =="run"){
        run_okay=true;
        drone.Run();
    }else if(rec_value =="stop"){
        drone.Stop();
    }
    else{
        Xbee.printf("There is nothing\n");
    }

        rec_value = " ";

}
///////////////////////////////////통신




////////////////////////////////////UWB
void UWB_get_Serial_data()
{   uwb_interrupt_check++;
    if(UWB.readable())
    {
        inputChar_UWB = UWB.getc();
        
        if(inputChar_UWB == '<' && !UWB_complete) Flag_Start_Get_UWB = true;
        if(Flag_Start_Get_UWB)
        {
            Buf_UWB[UWB_Count++] = inputChar_UWB;
            if(inputChar_UWB == '>')
            {
                Flag_Start_Get_UWB = false;
                UWB_complete = true;
                
                osSignalSet(UWB_Protocol_threadID, 1);
            }
        }
    }
}

void UWB_Protocol(){

    UWB_Protocol_threadID = osThreadGetId();
    float UWB_sample = 0.1;
    while(true){

        uint64_t current_Tick = Kernel::get_ms_count();
        uwb_thread_check++;
        // m1.lock();
        // UWB.printf("%c",3);
        // m1.unlock();

        osEvent sig = osSignalWait(1, osWaitForever);
        // m1.lock();
        for(int i=1;i<17;i++){
                    uwb_Hex = Buf_UWB[i];
                    // pc.printf("%x",Buf_UWB[i]);
                    
                    if((i-1)%2==0){
                        low = uwb_Hex;
                    }else{
                        high = uwb_Hex<<8;
                        high = high|low;
                        anchor_distance[(i-2)/2] = (double)high*0.00469176 - 154.6;
                        // Xbee.printf("%4.2f   ",anchor_distance[(i-2)/2]);
                    }
                }

                // Xbee.printf("\n");
                // m1.unlock();
        ///////////////////////////////////////////////////////anchor 4개일때
        // for(int i = 0;i<4;i++){
        //     if(anchor_distance[i+2]>-150){
        //         uwb_check_count++;
        //     }
        // }
        // if(uwb_check_count==3){
        //     if(anchor_distance[2]<-150){
        //         uwb_check_case = 2;
        //         case_sibal = uwb_check_case;
        //     }else if(anchor_distance[3]<-150){
        //         uwb_check_case = 3;
        //         case_sibal = uwb_check_case;
        //     }else if(anchor_distance[4]<-150){
        //         uwb_check_case = 4;
        //         case_sibal = uwb_check_case;
        //     }else if(anchor_distance[5]<-150){
        //         uwb_check_case = 5;
        //         case_sibal = uwb_check_case;
        //     }
        // }
        // if(uwb_check_count==4){//모든 값을 다 받으면
        //     case_sibal=0;
        //     for(int i = 0;i<4;i++){
        //         distance_anchor[i]=anchor_distance[i+2];////////distance_anchor 배열 갯수를 꼭 4로 줄이기!
        //         anchor_distance[i+2]= -200;
        //     }
        //     uwb_altitude = drone.Altitude/100.;
        //     distance_2D_Update(distance_anchor,distance_2D_anchor,(drone.Altitude/100.)+0.11);
        //     Trilateration_2D(distance_2D_anchor);
        //     uwb_case = 104;
        // }
        // else{
        //     drone.error_flag=true;
        //     uwb_case = 101;
        // }
        ///////////////////////////////////////////////////////anchor 4개일때

        ///////////////////////////////////////////////////////anchor 6개일때
        for(int i = 0;i<6;i++){
            if(anchor_distance[i+2]>0){
                uwb_check_count++;
                anchor_exist[i]=true;
                // pc.printf("anchor_distance[%d] = %f\n",i+2,anchor_distance[i+2]);
            }else{
                anchor_exist[i]=false;
            }
            // pc.printf("%d   ",anchor_exist[i]);
            distance_anchor[i] = anchor_distance[i+2];////////distance_anchor 배열 갯수를 꼭 6으로 늘리기!
        }
        // pc.printf("\n");
        
        if(anchor_distance[2]<-150){
            uwb_check_case = 2;
            case_sibal = uwb_check_case;
        }else if(anchor_distance[3]<-150){
            uwb_check_case = 3;
            case_sibal = uwb_check_case;
        }else if(anchor_distance[4]<-150){
            uwb_check_case = 4;
            case_sibal = uwb_check_case;
        }else if(anchor_distance[5]<-150){
            uwb_check_case = 5;
            case_sibal = uwb_check_case;
        }else if(anchor_distance[6]<-150){
            uwb_check_case = 6;
            case_sibal = uwb_check_case;
        }else if(anchor_distance[7]<-150){
            uwb_check_case = 7;
            case_sibal = uwb_check_case;
        }else {
            uwb_check_case = 0;
            case_sibal = uwb_check_case;
        }
        

        if(uwb_check_count>=4){
            for(int i = 0;i<6;i++){
                if(cal_count<4 && anchor_exist[i]==true){
                    cal_distance_anchor[cal_count]=distance_anchor[i];
                    cal_xi[cal_count]=xi[i],    cal_yi[cal_count]=yi[i],    cal_zi[cal_count]=zi[i];
                    // pc.printf("cal_anchor_distance[%d] = %f\n",cal_count,cal_distance_anchor[cal_count]);
                    cal_count++;               
                }
            }
            uwb_altitude = drone.Altitude/100.;
            // pc.printf("drone.Altitude : %f\n",drone.Altitude);
            cal_distance_2D_Update(cal_distance_anchor,distance_2D_anchor,(drone.Altitude/100.)+0.11);
            cal_Trilateration_2D(distance_2D_anchor);
            uwb_case = 104;
            // pc.printf("----------------------------------------------------------\n");
        }
        cal_count=0;
        ///////////////////////////////////////////////////////anchor 6개일때

        uwb_check_count = 0;
        uwb_anchor_number_setting = 0;
        UWB_Count = 0;
        uwb_check_case = 0;
        memset(Buf_UWB, 0, sizeof(Buf_UWB));
        UWB_complete = false;
        


        uint64_t working_Tick = Kernel::get_ms_count();
        ThisThread::sleep_until(Kernel::get_ms_count()+(UWB_sample*1000-(working_Tick-current_Tick)));
        uint64_t last_Tick = Kernel::get_ms_count();
    }
}




void Trilateration_2D(double *distance_2D_anchor){
    x_k = first_x;  y_k=first_y;//초기값 설정
    v_xk_1 =first_vx; v_yk_1=first_vy;//속도 초기값 설정
    int count = 0;
    

////////////////////////////////////////////////////////////////////////////////another Batch gradient desecnet
    x_k1=0; y_k1=0;
    bool while_flag = true;
    if(uwb_check_count==4){
        while(while_flag && count<running_time){
            
            x_k1 = x_k - alpha*gradient_fx_2D(distance_2D_anchor,x_k,y_k);
            y_k1 = y_k - alpha*gradient_fy_2D(distance_2D_anchor,x_k,y_k);

            // if(abs(x_k1-x_k)<error_diff && abs(y_k1-y_k)<error_diff){
            //     while_flag = false;
            // }
            if(abs((x_k1-x_k)/x_k1)<error_diff && abs((y_k1-y_k)/y_k1)<error_diff){
                while_flag = false;
            }
            relative_error = (abs(x_k1-x_k))/x_k1;
            x_k = x_k1; y_k=y_k1;
            // pc.printf("%d  %2.5f   %2.5f\n",count+1,x_k1,y_k1);
            count++;
        }
    }

    // pc.printf("/////////////\n");
    if(nan_checking!=0){//nan이 일어남
        drone_x = drone_past_x;
        drone_y = drone_past_y;
        check_nan++;
    }else{
        drone_x = x_k1;
        drone_y = y_k1;
    }
    // pc.printf("%2.5f    %2.5f\n",drone_x,drone_y);
    nan_checking = 0;
    CCount = count;
    drone.error_flag=false;
    drone_past_x = drone_x;
    drone_past_y = drone_y;
    drone.x_position = drone_x;
    drone.y_position = drone_y;
////////////////////////////////////////////////////////////////////////////////
}


double Bk_i_2D(double x_k,double y_k,int i){
    double value = sqrt(pow(x_k-xi[i],2)+pow(y_k-yi[i],2));
    return value;
}

double gradient_fx_2D(double *distance_2D_anchor,double x_k,double y_k){
    double value=0;
    if(uwb_check_count==4){
        for(int i=0;i<4;i++){
            value += 2*(Bk_i_2D(x_k,y_k,i)-distance_2D_anchor[i])*(x_k-xi[i])/Bk_i_2D(x_k,y_k,i);
        }
    }
    return value;
}


double gradient_fy_2D(double *distance_2D_anchor,double x_k,double y_k){
    double value=0;
    if(uwb_check_count==4){
        for(int i=0;i<4;i++){
            value += 2*(Bk_i_2D(x_k,y_k,i)-distance_2D_anchor[i])*(y_k-yi[i])/Bk_i_2D(x_k,y_k,i);
        }
    }
    
    return value;
}


void distance_2D_Update(double *distance_anchor,double *distance_2D_anchor, float drone_Altitude){
    if(uwb_check_count==4){
        for(int i = 0 ; i<4 ; i++){
            distance_2D_anchor[i]=sqrt( pow(distance_anchor[i],2) - pow(abs(zi[i]-drone_Altitude),2) );
            if(distance_2D_anchor[i]<=0){
                nan_checking++;
                check_nan++;
                // pc.printf("a %f\n",distance_2D_anchor[i]);
            }
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void cal_Trilateration_2D(double *distance_2D_anchor){
    x_k = first_x;  y_k=first_y;//초기값 설정
    v_xk_1 =first_vx; v_yk_1=first_vy;//속도 초기값 설정
    int count = 0;

    x_k1=0; y_k1=0;
    bool while_flag = true;

        while(while_flag && count<running_time){
            
            x_k1 = x_k - alpha*cal_gradient_fx_2D(distance_2D_anchor,x_k,y_k);
            y_k1 = y_k - alpha*cal_gradient_fy_2D(distance_2D_anchor,x_k,y_k);
            // pc.printf("x_k1 : %f    x_k : %f\n",x_k1,x_k);
            // if(abs(x_k1-x_k)<error_diff && abs(y_k1-y_k)<error_diff){
            //     while_flag = false;
            // }
            if(abs((x_k1-x_k)/x_k1)<error_diff && abs((y_k1-y_k)/y_k1)<error_diff){
                while_flag = false;
            }
            relative_error = (abs(x_k1-x_k))/x_k1;
            x_k = x_k1; y_k=y_k1;
            // pc.printf("%d  %2.5f   %2.5f\n",count+1,x_k1,y_k1);
            count++;
        }


    // pc.printf("/////////////\n");
    if(nan_checking!=0){//nan이 일어남
        drone_x = drone_past_x;
        drone_y = drone_past_y;
        check_nan++;
    }else{
        drone_x = x_k1;
        drone_y = y_k1;
    }
    // pc.printf("%2.5f    %2.5f\n",drone_x,drone_y);
    nan_checking = 0;
    CCount = count;
    drone.error_flag=false;
    drone_past_x = drone_x;
    drone_past_y = drone_y;
    drone.x_position = drone_x;
    drone.y_position = drone_y;

}

void cal_distance_2D_Update(double *cal_distance_anchor,double *distance_2D_anchor, float drone_Altitude){
        for(int i = 0 ; i<4 ; i++){
            distance_2D_anchor[i]=sqrt( pow(cal_distance_anchor[i],2) - pow(abs(cal_zi[i]-drone_Altitude),2) );
            // pc.printf("cal_distance_anchor[%d] : %f  cal_zi[%d] : %f    drone_Altitude : %f     abs(cal_zi[i]-drone_Altitude) : %f\n",i,cal_distance_anchor[i],i,cal_zi[i],drone_Altitude,abs(cal_zi[i]-drone_Altitude));
            if(distance_2D_anchor[i]<=0){
                nan_checking++;
                check_nan++;
                // pc.printf("a %f\n",distance_2D_anchor[i]);
            }
        }
        // pc.printf("////////////////////////////////\n");
}

double cal_Bk_i_2D(double x_k,double y_k,int i){
    double value = sqrt(pow(x_k-cal_xi[i],2)+pow(y_k-cal_yi[i],2));
    return value;
}

double cal_gradient_fx_2D(double *distance_2D_anchor,double x_k,double y_k){
    double value=0;
    
        for(int i=0;i<4;i++){
            value += 2*(cal_Bk_i_2D(x_k,y_k,i)-distance_2D_anchor[i])*(x_k-cal_xi[i])/cal_Bk_i_2D(x_k,y_k,i);
        }
    
    
    return value;
}


double cal_gradient_fy_2D(double *distance_2D_anchor,double x_k,double y_k){
    double value=0;

        for(int i=0;i<4;i++){
            value += 2*(cal_Bk_i_2D(x_k,y_k,i)-distance_2D_anchor[i])*(y_k-cal_yi[i])/cal_Bk_i_2D(x_k,y_k,i);
        }
    
    
    return value;
}



float Error_calculation(double *distance_2D_anchor,double x,double y){
    float error = 0;
    for(int i=0;i<4;i++){
        error += pow(sqrt(pow(x-xi[i],2)+pow(y-yi[i],2))-distance_2D_anchor[i],2);
    }
    return error;
}

///////////////////////////////////////UWB


////////////////////////////////////////서보모터
void turn(PwmOut &rcServo,float deg){
    deg = map<float>(deg,0.,180.,180.,0.);//반대 서보
    uint16_t pulseW = map<float>(deg,0.,180.,2500.,3800.);
    rcServo.pulsewidth_us(pulseW);
}

void rc_turn(){
    rc_deg = map<float>(rc_deg,0.,180.,180.,0.);//반대 서보
    uint16_t pulseW = map<float>(rc_deg,0.,180.,3000.,4300.);
    rc.pulsewidth_us(pulseW);
}
////////////////////////////////////////서보모터

void target_ticker_function(){
    
    if(abs(Xbee_x_target-drone.target_xpos)<0.05&&!x_target_finish_flag){
        drone.target_xpos = Xbee_x_target;
        x_target_finish_flag = true;
    }
    else if(Xbee_x_target>drone.target_xpos){
        drone.target_xpos += 0.05;
    }else if(Xbee_x_target<drone.target_xpos){
        drone.target_xpos -= 0.05;
    }

    if(abs(Xbee_y_target-drone.target_ypos)<0.05&&!y_target_finish_flag){
        drone.target_ypos = Xbee_y_target;
        y_target_finish_flag = true;
    }
    else if(Xbee_y_target>drone.target_ypos){
        drone.target_ypos += 0.05;
    }else if(Xbee_y_target<drone.target_ypos){
        drone.target_ypos -= 0.05;
    }

    if(x_target_finish_flag&&y_target_finish_flag){
        target_ticker.detach();
    }
}

void Altitude_ticker_function(){
    
    if(abs(Xbee_altitude_target-drone.target_altitude)<0.05&&!altitude_target_finish_flag){
        drone.target_altitude = Xbee_altitude_target;
        altitude_target_finish_flag = true;
    }
    else if(Xbee_altitude_target>drone.target_altitude){
        drone.target_altitude += 1;
        if(drone.target_altitude>180) drone.target_altitude=180;
    }else if(Xbee_altitude_target<drone.target_altitude){
        drone.target_altitude -= 1;
    }

    if(altitude_target_finish_flag){
        altitude_target_ticker.detach();
    }

}