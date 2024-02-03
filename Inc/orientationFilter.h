/*
 * orientation_filter.h
 *
 *  Created on: Feb 1, 2024
 *      Author: skyle
 */

#ifndef INC_ORIENTATIONFILTER_H
#define INC_ORIENTATIONFILTER_H



#ifdef __cplusplus
extern "C" {
#endif

// Struct to hold the orientation in Euler angles (roll and pitch)
typedef struct {
    float roll;  // Roll angle in radians
    float pitch; // Pitch angle in radians
} OrientationEuler;

// Initialize the orientation filter
void orientation_filter_init(float alpha);

// Update the orientation filter with new sensor data
void orientation_filter_update(float ax, float ay, float az, float gx, float gy, float deltaTime);

// Get the current orientation
OrientationEuler orientation_filter_get_orientation();

#ifdef __cplusplus
}
#endif

#endif // ORIENTATIONFILTER_H
