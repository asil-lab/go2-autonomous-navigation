/*
 * This file is used to define the temperature and humidity sensor functionality.
 * Author: Alexander James Becoy
 * Date: 2024-11-04
 */

#ifndef AUXILIARY_SENSING__TEMP_HUMIDITY_H
#define AUXILIARY_SENSING__TEMP_HUMIDITY_H

#include <Adafruit_SHT31.h>

#define SHT31_TO_GROUND 0x44

// Function prototypes
bool isTempHumiditySensorAvailable();
float getTemperatureValue();
float getHumidityValue();

// Create an instance of the temperature and humidity sensor
Adafruit_SHT31 temp_humidity_sensor = Adafruit_SHT31();

// Function to check if the temperature and humidity sensor is available
bool isTempHumiditySensorAvailable() {
    return temp_humidity_sensor.begin(SHT31_TO_GROUND);
}

// Function to get the temperature value
float getTemperatureValue() {
    return temp_humidity_sensor.readTemperature();
}

// Function to get the humidity value
float getHumidityValue() {
    return temp_humidity_sensor.readHumidity();
}

#endif //AUXILIARY_SENSING__TEMP_HUMIDITY_H