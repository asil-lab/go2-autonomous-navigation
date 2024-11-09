/*
 * This file is used to define the light sensor functionality.
 * Author: Alexander James Becoy
 * Date: 2024-11-04
 */

#ifndef AUXILIARY_SENSING__LIGHT_H
#define AUXILIARY_SENSING__LIGHT_H

#include <hp_BH1750.h>

// Function prototypes
bool isLightSensorAvailable();
float getLightValue();

// Create an instance of the light sensor
hp_BH1750 light_sensor;

// Function to check if the light sensor is available
bool isLightSensorAvailable() {
    return light_sensor.begin(BH1750_TO_GROUND);
}

// Function to get the light value
float getLightValue() {
    light_sensor.start();
    return light_sensor.getLux(); // 16-bit light intensity value in light (SI unit)
}

#endif //AUXILIARY_SENSING__LIGHT_H