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

	MotorCommands->Roll = (int)((*DroneRollPosition / MotorCommands->sensorMax) * (float)MotorCommands->TimARR/2);

	if(MotorCommands->Roll < 0){
		MotorCommands->Roll = 0;
	}
	if(MotorCommands->Roll > 900){
		MotorCommands->Roll = 900;
	}
}

void Update_Duty_Cycles(DroneMotorCommand *MotorCommands){
	// Reset motor commands
	int motorMin = 595;
	int motorMax = 650;

	MotorCommands->M1DutyCycle = motorMin;
	MotorCommands->M2DutyCycle = motorMin;	//TBD
	MotorCommands->M3DutyCycle = motorMin;	//TBD
	MotorCommands->M4DutyCycle = motorMin;	//TBD

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

	if(MotorCommands->M1DutyCycle > motorMax){
		MotorCommands->M1DutyCycle = motorMax;
	} else if (MotorCommands->M1DutyCycle < motorMin){
		MotorCommands->M1DutyCycle = motorMin;
	}

	if(MotorCommands->M2DutyCycle > motorMax){
		MotorCommands->M2DutyCycle = motorMax;
	} else if (MotorCommands->M2DutyCycle < motorMin){
		MotorCommands->M2DutyCycle = motorMin;
	}

	if(MotorCommands->M3DutyCycle > motorMax){
		MotorCommands->M3DutyCycle = motorMax;
	} else if (MotorCommands->M3DutyCycle < motorMin){
		MotorCommands->M3DutyCycle = motorMin;
	}

	if(MotorCommands->M4DutyCycle > motorMax){
		MotorCommands->M4DutyCycle = motorMax;
	} else if (MotorCommands->M4DutyCycle < motorMin){
		MotorCommands->M4DutyCycle = motorMin;
	}
}
