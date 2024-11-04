/*
 * This file is used to define the lux sensor functionality.
 * Author: Alexander James Becoy
 * Date: 2024-11-04
 */

#ifndef AUXILIARY_SENSING__LUX_H
#define AUXILIARY_SENSING__LUX_H

#include <hp_BH1750.h>

// Function prototypes
bool isLuxSensorAvailable();
float getLuxValue();

// Create an instance of the lux sensor
hp_BH1750 lux_sensor;

// Function to check if the lux sensor is available
bool isLuxSensorAvailable() {
    return lux_sensor.begin(BH1750_TO_GROUND);
}

// Function to get the lux value
float getLuxValue() {
    lux_sensor.start();
    return lux_sensor.getLux();
}

#endif //AUXILIARY_SENSING__LUX_H