/*
 * PID.c
 *
 *  Created on: Jan 2, 2024
 *      Author: skyle
 */

#include "PID.h"

void PIDController_Init(PIDController *pid){
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement){

	// Error signal
	float error = setpoint - measurement;


	// Proportional

	float proportional = pid->K_p * error;


	// Integral
	float ki = pid->K_i;
	if(ki == 0.0f){
		pid->integrator = 0;
	} else{
		pid->integrator = pid->integrator + 0.5f * pid->K_i * pid->T * (error + pid->prevError);

		// Anti-windup via dynamic integrator clamping
		float limMinInt, limMaxInt;
		if (pid->limMax > proportional){
			limMaxInt = pid->limMax - proportional;
		} else{
			limMaxInt = 0.0f;
		}

		if(pid->limMin < proportional){
			limMinInt = pid->limMin - proportional;
		}else{
			limMinInt = 0.0f;
		}

		// Clamp the integrator
		if (pid->integrator > limMaxInt){
			pid->integrator = limMaxInt;
		} else if(pid->integrator < limMinInt){
			pid->integrator = limMinInt;
		}
	}

	if(pid->K_d != 0){
		// Derivative (band limited differentiator)
		pid->differentiator = (2.0f * pid->K_d * (measurement - pid->prevMeasurement)
							  + (2.0f * pid->tau - pid->T) * pid->differentiator)
							  / (2.0f * pid->tau + pid->T);
	} else{
		pid->differentiator = 0;
	}

	pid->out = proportional + pid->integrator + pid->differentiator;

	if(pid->out > pid->limMax){
		pid->out = pid-> limMax;
	} else if (pid->out < pid->limMin){
		pid->out = pid->limMin;
	}

	pid ->prevError = error;
	pid->prevMeasurement = measurement;

	return pid->out;
}
