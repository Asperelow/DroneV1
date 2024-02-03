/*
 * orientation_filter.c
 *
 *  Created on: Feb 1, 2024
 *      Author: skyle
 */

#include "orientationFilter.h"
#include <math.h>

// Internal state
static OrientationEuler current_orientation = {0.0f, 0.0f};
static float filter_alpha = 0.5f; // Complementary filter constant, 0 < alpha < 1

void orientation_filter_init(float alpha) {
    filter_alpha = alpha;
    current_orientation.roll = 0.0f;
    current_orientation.pitch = 0.0f;
}

void orientation_filter_update(float ax, float ay, float az, float gx, float gy, float deltaTime) {
    // Convert accelerometer readings into roll and pitch angles
    float roll_acc = atan2(ay, az);
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az));

    // Integrate gyroscope data to get angles in radians
    current_orientation.pitch += gx * deltaTime;
    current_orientation.roll += gy * deltaTime;

    // Apply complementary filter
    current_orientation.pitch = current_orientation.pitch * filter_alpha + roll_acc * (1.0f - filter_alpha);
    current_orientation.roll = current_orientation.roll * filter_alpha + pitch_acc * (1.0f - filter_alpha);
}

OrientationEuler orientation_filter_get_orientation() {
    return current_orientation;
} /* INC_ORIENTATIONFILTER_H_ */
