//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------

class MahonyAHRS{
    
public:

    MahonyAHRS(float Freq);
    
    //Function declarations
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void getEuler();
    float getRoll();
    float getPitch();
    float getYaw();
    
    
private:
    
    // Variable declaration
    float sampleFreq;
    float roll;
    float pitch;
    float yaw;
};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================