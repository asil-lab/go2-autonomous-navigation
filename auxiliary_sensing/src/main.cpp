#include <Arduino.h>
#include "miso.h"
#include "sensors/lux.h"
#include "sensors/temp_humidity.h"

#define SERIAL_BAUD_RATE 115200
#define DELAY_TIME 100

struct MOSI mosi;

void setup() {
  // Initialize the serial communication.
    Serial.begin(SERIAL_BAUD_RATE);

    // Initialize the sensors.
    if (!isLuxSensorAvailable()) {
        Serial.println("Lux sensor not available.");
        while (true) {};
    }

    if (!isTempHumiditySensorAvailable()) {
        Serial.println("Temperature and humidity sensor not available.");
        while (true) {};
    }
}

void loop() {
    // Read the sensor data.
    mosi.lux = getLuxValue();
    mosi.temperature = getTemperatureValue();
    mosi.humidity = getHumidityValue();

    // Output the sensor data.
    Serial.println(mosi_to_string(&mosi));

    // Delay for 1 second.
    delay(DELAY_TIME);
}
