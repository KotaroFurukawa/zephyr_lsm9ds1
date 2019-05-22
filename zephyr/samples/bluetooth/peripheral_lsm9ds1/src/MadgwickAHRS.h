//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

#define PI 3.14159265358979323846
#define M_PI PI
//#define PI M_PI

//float q[4] = {0.1f, 0.0f, 0.0f, 0.0f};
#define deg2rad(a) ((a)/180.0f * M_PI) /* deg を rad に換算するマクロ関数 */

typedef struct quaternion_t{
    float x;
    float y;
    float z;
    float w;
}Quaternion;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

Quaternion getQuaternion(void);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
