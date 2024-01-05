/*
 * kalmanFilters.c
 *
 *  Created on: Jun 11, 2023
 *      Author: skyle
 */

#include "kalmanFilters.h"


void kalman_roll_pitch_init(KalmanRollPitch *kal, float Pinit, float *Q, float *R)
{
	kal->phi_rad = 0.0f;
	kal->theta_rad = 0.0f;

	kal->P[0] = Pinit; 	kal->P[1] = 0;
	kal->P[2] = 0; 		kal->P[3] = Pinit;

	kal->Q[0] = Q[0];	kal->Q[1] = Q[1];
	kal->R[0] = R[0];	kal->R[1] = R[1];	kal->R[2] = R[2];
}

void kalman_roll_pitch_predict(KalmanRollPitch *kal, float *gyro_rps, float T)
{
	// Extract measurements
	float p = gyro_rps[0];
	float q = gyro_rps[1];
	float r = gyro_rps[2];

	// Common used trig terms
	float sinPhi = sin(kal->phi_rad);	float cosPhi = cos(kal->phi_rad);
	float tanTheta = tan(kal->theta_rad);

	// x += x- + T * f(x,u)
	kal->phi_rad = 		kal->phi_rad 	+ T * (p + tanTheta * (q * sinPhi + r * cosPhi));
	kal->theta_rad = 	kal->theta_rad 	+ T * (q * cosPhi - r * sinPhi);

	// Common trig terms with new estimates
	sinPhi = sin(kal->phi_rad);	cosPhi = cos(kal->phi_rad);
	float sinTheta = sin(kal->theta_rad);	float cosTheta = cos(kal->theta_rad);
	tanTheta = sinTheta / cosTheta;

	// Compute Jacobian of f(x,u)
	float A[4] = {	tanTheta * (q * cosPhi - r * sinPhi), (r * cosPhi + q * sinPhi) * (tanTheta * tanTheta + 1.0f),
					-(r * cosPhi + q * sinPhi)			, 0.0f};

	float Ptmp[4] = { 	T * (kal->Q[0] + 2.0f * A[0] * kal->P[0] + A[1] * kal->P[1] + A[1] * kal->P[2]),
						T * (A[0] * kal->P[1] + A[2] * kal->P[0] + A[1] * kal->P[3] + A[3] * kal->P[1]),
						T * (A[0] * kal->P[2] + A[2] * kal->P[0] + A[1] * kal->P[3] + A[3] * kal->P[2]),
						T * (kal->Q[1] + A[2] * kal->P[1] + A[2] * kal->P[2] + 2.0f * A[3] * kal->P[3])};

	kal->P[0] = kal->P[0] + Ptmp[0];	kal->P[1] = kal->P[1] + Ptmp[1];
	kal->P[2] = kal->P[2] + Ptmp[2];	kal->P[3] = kal->P[3] + Ptmp[3];
}


void kalman_roll_pitch_update(KalmanRollPitch *kal, float *accel_mps2)
{
	// Extract measurements
	float ax = accel_mps2[0];
	float ay = accel_mps2[1];
	float az = accel_mps2[2];

	// Common used trig terms
	float sinPhi = sin(kal->phi_rad);		float cosPhi = cos(kal->phi_rad);
	float sinTheta = sin(kal->theta_rad);	float cosTheta = cos(kal->theta_rad);

	// Output function h(x,u)
	float h[3] = {	g * sinTheta,
					-g * cosTheta * sinPhi,
					-g * cosTheta * cosPhi};

	// Jacobian of h(x,u)
	float C[6] = {	0.0f,	g * cosTheta,
					-g * cosPhi * cosTheta,	g * sinPhi * sinTheta,
					g * sinPhi *  cosTheta, g * cosPhi * sinTheta};

	// Kalman gain K = P * C' / (C * P * C' + R)
	float Pmat[2][2] = {kal->P[0], kal->P[1], kal->P[2], kal->P[3]};
	float Cmat[3][2] = {C[0], C[1], C[2], C[3], C[4], C[5]};
	float CmatT[2][3];
	matrix_transpose(Cmat, CmatT, 3, 2);
	float Rmat[3][3] = {kal->R[0], 0.0f, 0.0f, 0.0f, kal->R[1], 0.0f, 0.0f, 0.0f, kal->R[2]};

	float CP[3][2];
	matrix_multiply(&Cmat, &Pmat, &CP, 3, 2, 2);
	float CPCT[3][3];
	matrix_multiply(&CP, &CmatT, &CPCT, 3, 2, 3);
	CPCT[0][0] = CPCT[0][0] + kal->R[0]; CPCT[1][1] = CPCT[1][1] + kal->R[1];	CPCT[2][2] = CPCT[2][2] + kal->R[2];


	float PCT[2][3];
	matrix_multiply(&Pmat, &CmatT, &PCT, 2, 2, 3);

	float CPCTi[3][3];
	float K[2][3];
	matrix_inverse_3by3(&CPCT, &CPCTi);
	matrix_multiply(&PCT, &CPCTi, &K, 2, 3, 3);


	// Kalman gain K = P * C' / (C * P * C' + R)

	kal->phi_rad = 		kal->phi_rad + K[0][0]*(ax - h[0]) + K[0][1] * (ay - h[1]) + K[0][2] * (az - h[2]);
	kal->theta_rad = 	kal->theta_rad + K[1][0]*(ax - h[0]) + K[1][1] * (ay - h[1]) + K[1][2] * (az - h[2]);
}



void matrix_multiply(float *A, float *B, float *C, int rows_A, int cols_A, int cols_B)
{
    for (int i = 0; i < rows_A; i++) {
        for (int j = 0; j < cols_B; j++) {
            C[i * cols_B + j] = 0.0;
            for (int k = 0; k < cols_A; k++) {
                C[i * cols_B + j] += A[i * cols_A + k] * B[k * cols_B + j];
            }
        }
    }
}

void matrix_transpose(float *input, float *output, int rows, int cols)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            output[j * rows + i] = input[i * cols + j];
        }
    }
}


void matrix_inverse_3by3(float *A, float *result)
{
    // Calculate the determinant of the matrix
    float determinant = A[0] * (A[4] * A[8] - A[5] * A[7]) -
                        A[1] * (A[3] * A[8] - A[5] * A[6]) +
                        A[2] * (A[3] * A[7] - A[4] * A[6]);

    // Check if the matrix is non-invertible (singular)
    if (fabs(determinant) < 1e-6) {
        return;
    }

    // Calculate the co-factor matrix
    result[0] = (A[4] * A[8] - A[5] * A[7]) / determinant;
    result[1] = -(A[1] * A[8] - A[2] * A[7]) / determinant;
    result[2] = (A[1] * A[5] - A[2] * A[4]) / determinant;
    result[3] = -(A[3] * A[8] - A[5] * A[6]) / determinant;
    result[4] = (A[0] * A[8] - A[2] * A[6]) / determinant;
    result[5] = -(A[0] * A[5] - A[2] * A[3]) / determinant;
    result[6] = (A[3] * A[7] - A[4] * A[6]) / determinant;
    result[7] = -(A[0] * A[7] - A[1] * A[6]) / determinant;
    result[8] = (A[0] * A[4] - A[1] * A[3]) / determinant;
}

















