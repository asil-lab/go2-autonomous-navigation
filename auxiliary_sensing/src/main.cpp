#include <Arduino.h>
#include "miso.h"
#include "sensors/light.h"
#include "sensors/temp_humidity.h"

#define SERIAL_BAUD_RATE 115200
#define DELAY_TIME 1000

struct MOSI mosi;

void setup() {
  // Initialize the serial communication.
    Serial.begin(SERIAL_BAUD_RATE);

    // Initialize the sensors.
    startLightSensor();
    startTempHumiditySensor();
}

void loop() {
    // Read the sensor data.
    mosi.light = getLightValue();
    mosi.temperature = getTemperatureValue();
    mosi.humidity = getHumidityValue();

    // Output the sensor data.
    Serial.println(mosi_to_string(&mosi));

    // Delay for 1 second.
    delay(DELAY_TIME);
}
