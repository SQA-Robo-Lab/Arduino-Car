#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#ifdef __cplusplus
  extern "C" {
#endif

#include "Arduino.h"

/**
 * @brief Initializes the distance sensors. MUST be called once before the other methods are effective.
 * 
 */
void initializeDistanceSensors();

/**
 * @brief Measure the front distance.
 * 
 * @return the measured distance in cm
 */
int getFrontDistance();

/**
 * @brief Measure the rear distance.
 * 
 * @return the measured distance in cm
 */
int getRearDistance();

#ifdef __cplusplus
  }
#endif
#endif /* DISTANCE_SENSOR_H */