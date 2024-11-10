/*
 * This file is used to define the light sensor functionality.
 * Author: Alexander James Becoy
 * Date: 2024-11-04
 */

#ifndef AUXILIARY_SENSING__LIGHT_H
#define AUXILIARY_SENSING__LIGHT_H

#include <hp_BH1750.h>

// Function prototypes
void startLightSensor();
bool isLightSensorAvailable();
float getLightValue();

// Global variables
bool light_sensor_available = false;

// Create an instance of the light sensor
hp_BH1750 light_sensor;

// Function to check if the light sensor is available
void startLightSensor() {
    light_sensor_available = light_sensor.begin(BH1750_TO_GROUND);
}

// Function to check if the light sensor is available
bool isLightSensorAvailable() {
    return light_sensor_available;
}

// Function to get the light value
float getLightValue() {
    // Return -1 if the light sensor is not available
    if (!isLightSensorAvailable()) {
        return -1;
    }

    light_sensor.start();
    return light_sensor.getLux(); // 16-bit light intensity value in light (SI unit)
}

#endif //AUXILIARY_SENSING__LIGHT_H