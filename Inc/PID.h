/*
 * PID.h
 *
 *  Created on: Dec 30, 2023
 *      Author: skyle
 */

#ifndef INC_PID_H_
#define INC_PID_H_


typedef struct
{
	// Controller Gains
	float K_p;
	float K_i;
	float K_d;

	// Derivative low-pass filter time constant
	float tau;

	// Output limits
	float limMin;
	float limMax;

	// Sample time (in seconds)
	float T;

	// Controller memory
	float integrator;
	float differentiator;
	float prevError;
	float prevMeasurement;

	// Controller output
	float out;
} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif /* INC_PID_H_ */
