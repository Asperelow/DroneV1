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

typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;

	float sampleFreq;
	float beta;
}quanternion;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSInit(quanternion *q, float sampleFreqMS);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, quanternion *q);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, quanternion *q);
float invSqrt(float x);


#endif
//=====================================================================================================
// End of file
//=====================================================================================================
