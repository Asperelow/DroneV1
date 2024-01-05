/*
 * MotorControl.h
 *
 *  Created on: Jan 2, 2024
 *      Author: skyle
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

typedef struct{
	int TimARR;
	float sensorMax;

	float Roll;
	float Pitch;
	float Yaw;


	/* ----------- Drone Anatomy -------------------
	 *    M1    M2          +X Pitch
	 *    \   /
	 *      X         +Y        -Y   Roll
	 *    /   \
	 *   M3    M4          -X
	 * ---------------------------------------------
	 */

	int M1DutyCycle;
	int M2DutyCycle;
	int M3DutyCycle;
	int M4DutyCycle;
} DroneMotorCommand;

void Motor_Command_Roll(DroneMotorCommand *MotorCommands, float *DroneRollPosition);
void Update_Duty_Cycles(DroneMotorCommand *MotorCommands);

#endif /* INC_MOTORCONTROL_H_ */
