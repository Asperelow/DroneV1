/*
 * MotorControl.c
 *
 *  Created on: Jan 2, 2024
 *      Author: skyler
 */

#include "MotorControl.h"


/*
 * This function takes in a float value between 0 and 100 and converts it to a duty cycle
 * MotorControlInput: % Of motors duty cycle
 */
void Motor_Command_Roll(DroneMotorCommand *MotorCommands, float *DroneRollPosition){

	if(MotorCommands->Roll < MotorCommands->throttleMin){
		MotorCommands->Roll = MotorCommands->throttleMin;
	}
	if(MotorCommands->Roll > MotorCommands->throttleMax){
		MotorCommands->Roll = MotorCommands->throttleMax;
	}
}

void Update_Duty_Cycles(DroneMotorCommand *MotorCommands){
	// Reset motor commands

	MotorCommands->M1DutyCycle = MotorCommands->throttleMin;
	MotorCommands->M2DutyCycle = MotorCommands->throttleMin;
	MotorCommands->M3DutyCycle = MotorCommands->throttleMin;
	MotorCommands->M4DutyCycle = MotorCommands->throttleMin;

	// ################ Add z Commands ########################
	if (MotorCommands->z > 0){
		MotorCommands->M1DutyCycle += MotorCommands->z;
		MotorCommands->M2DutyCycle += MotorCommands->z;
		MotorCommands->M3DutyCycle += MotorCommands->z;
		MotorCommands->M4DutyCycle += MotorCommands->z;

	}

	// ################ Add Roll Commands ########################
	if (MotorCommands->Roll > 0){
		MotorCommands->M1DutyCycle += MotorCommands->Roll;
		MotorCommands->M3DutyCycle += MotorCommands->Roll;
	}
	else{
		MotorCommands->M2DutyCycle += MotorCommands->Roll;
		MotorCommands->M4DutyCycle += MotorCommands->Roll;
	}

	// ################ Add Pitch Commands ########################
	if (MotorCommands->Pitch > 0){
		MotorCommands->M1DutyCycle += MotorCommands->Roll;
		MotorCommands->M2DutyCycle += MotorCommands->Roll;
	}
	else{
		MotorCommands->M3DutyCycle += MotorCommands->Roll;
		MotorCommands->M4DutyCycle += MotorCommands->Roll;
	}


	// ############### Bound output of each motor #################
	if(MotorCommands->M1DutyCycle > MotorCommands->throttleMax){
		MotorCommands->M1DutyCycle = MotorCommands->throttleMax;
	} else if (MotorCommands->M1DutyCycle < MotorCommands->throttleMin){
		MotorCommands->M1DutyCycle = MotorCommands->throttleMin;
	}

	if(MotorCommands->M2DutyCycle > MotorCommands->throttleMax){
		MotorCommands->M2DutyCycle = MotorCommands->throttleMax;
	} else if (MotorCommands->M2DutyCycle < MotorCommands->throttleMin){
		MotorCommands->M2DutyCycle = MotorCommands->throttleMin;
	}

	if(MotorCommands->M3DutyCycle > MotorCommands->throttleMax){
		MotorCommands->M3DutyCycle = MotorCommands->throttleMax;
	} else if (MotorCommands->M3DutyCycle < MotorCommands->throttleMin){
		MotorCommands->M3DutyCycle = MotorCommands->throttleMin;
	}

	if(MotorCommands->M4DutyCycle > MotorCommands->throttleMax){
		MotorCommands->M4DutyCycle = MotorCommands->throttleMax;
	} else if (MotorCommands->M4DutyCycle < MotorCommands->throttleMin){
		MotorCommands->M4DutyCycle = MotorCommands->throttleMin;
	}
}
