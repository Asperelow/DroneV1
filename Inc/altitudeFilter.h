/*
 * altitudeFilter.h
 *
 *  Created on: Feb 1, 2024
 *      Author: skyle
 */

#ifndef INC_ALTITUDEFILTER_H_
#define INC_ALTITUDEFILTER_H_

//#define BARO_UPDATE_RATE_MS 	500;

// Updates the altitude estimate based on new sensor data
void altitude_filter_update(float* currentAltitude, float* pressure, float* az);

// Retrieves the current altitude estimate
float altitude_filter_get_altitude(float* currentAltitude);

#ifdef __cplusplus
}
#endif

#endif /* INC_ALTITUDEFILTER_H_ */
