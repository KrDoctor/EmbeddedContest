///PIN OUT///
#define PIN_BLDC_PWM_1 D4
#define PIN_BLDC_PWM_2 D5
#define PIN_BLDC_PWM_3 D6
#define PIN_BLDC_PWM_4 D8
#define PIN_BLDC_PWM_5 PB_7
#define PIN_BLDC_PWM_6 D10

#define PIN_LANDING PA_11

#define PIN_UWB_TX PA_0
#define PIN_UWB_RX PA_1

#define PIN_PPM D3

#define PIN_OPTICAL D2

#define PIN_ZIGBEE_TX PC_12
#define PIN_ZIGBEE_RX PD_2
/// 상수 //
#define M_PI 3.142592
#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.295779513

#define BLDC_MAX_SIGNAL 2000
#define BLDC_MIN_SIGNAL 1000

#define LIDAR_TX_DATA_1 0x00
#define LIDAR_TX_DATA_2 0x04
#define LIDAR_TX_DATA_3 0x8f
#define LIDAR_ADD 0x62 << 1

//TIMER 변수
#define Dt 0.001
#define Dt_LIDAR 0.002
#define Dt_UART 0.01
#define Dt_OPTICAL 0.01
#define Dt_IMU 0.001