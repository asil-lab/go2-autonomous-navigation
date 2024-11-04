#include <Arduino.h>
#include <hp_BH1750.h>
#include <Adafruit_SHT31.h>
#include <Wire.h>

struct MISO {
  u_int32_t lux;
  u_int32_t temperature;
  u_int32_t humidity;
};

Adafruit_SHT31 sht31 = Adafruit_SHT31();
hp_BH1750 BH1750;

void setup() {
  Serial.begin(115200);

  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};
  }
  Serial.println("BH1750 sensor found!");

  if (! sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  Serial.println("SHT31 sensor found!");
}

void loop() {
  BH1750.start();
  float lux = BH1750.getLux();
  Serial.print("Lux: "); Serial.println(lux);

  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {
    Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
  } else { 
    Serial.println("Failed to read temperature");
  }

  if (! isnan(h)) {
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }

  delay(1000);
}
