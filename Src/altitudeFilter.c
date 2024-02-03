/*
 * altitudeFilter.c
 *
 *  Created on: Feb 1, 2024
 *      Author: skyle
 */

#include "altitudeFilter.h"
#include <math.h>

// Internal state
static float alpha = 0.6f; // Complementary filter coefficient

void altitude_filter_update(float* currentAltitude, float* pressure, float* az) {
    // Estimate change in altitude from accelerometer
    // Simplified to focus on vertical acceleration component (az) only
    float verticalAcceleration = *az - 10.05f; // Assuming az includes gravity, subtract it
    float altitudeChange = verticalAcceleration * 0.1f * 0.1f;// * 0.001f * 0.001f;

    // Update the current altitude estimate using a complementary filter
    // Note: This simple approach combines barometer and accelerometer data directly
    // without double integrating acceleration.
    *currentAltitude = alpha * (*currentAltitude + altitudeChange) + (1.0f - alpha) * *pressure;
}

float altitude_filter_get_altitude(float* currentAltitude) {
    return *currentAltitude;
}
