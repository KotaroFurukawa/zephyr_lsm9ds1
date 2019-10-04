#ifndef quaternionFilter_H
#define quaternionFilter_H

#include <stdint.h>
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration

#define M_PI (3.14159265358979323846264338327950288f)
#define PI M_PI


//extern float q0, q1, q2, q3;    // quaternion elements representing the estimated orientation
extern float q[4];  // vector to hold quaternion
extern float deltat;
extern float pitch, yaw, roll;
extern float eInt[3];

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError (PI * (40.0f / 180.0f))   // gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasDrift (PI * (0.0f  / 180.0f))   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

#define beta (sqrt(3.0f / 4.0f) * GyroMeasError)   // compute beta
#define zeta (sqrt(3.0f / 4.0f) * GyroMeasDrift)   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void yawPitchRoll(void);

#endif

