//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//---------------------------------------------------------------------------------------------------
// Definitions

#define MadgwickBetaDef   0.25   // 2 * proportional gain
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern double madgwick_beta;   // algorithm gain
extern double q_a[4];  // quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double deltat);
void MadgwickGYROupdate(double gx, double gy, double gz, double deltat);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
