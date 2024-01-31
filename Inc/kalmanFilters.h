/*
 * kalmanFilters.h
 *
 *  Created on: Jun 11, 2023
 *      Author: skyle
 */

#ifndef INC_KALMANFILTERS_H
#define INC_KALMANFILTERS_H

#include<math.h>

#define g ((float)9.81)

typedef struct{
	float phi_rad;
	float theta_rad;

	float P[4];
	float Q[2];
	float R[3];
} KalmanRollPitch;

void kalman_roll_pitch_init(KalmanRollPitch *kal, float Pinit, float *Q, float *R);
void kalman_roll_pitch_predict(KalmanRollPitch *kal, float *gyro_rps, float T);
void kalman_roll_pitch_update(KalmanRollPitch *kal, float *accel_mps2);
void matrix_multiply(float *A, float *B, float *C, int rows_A, int cols_A, int cols_B);
void matrix_transpose(float *input, float *output, int rows, int cols);
void matrix_inverse_3by3(float *A, float *result);

#endif /* INC_KALMANFILTERS_H */
