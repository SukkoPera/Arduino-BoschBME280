#include "BoschBME280.h"

BoschBME280 bme;

void setup () {
  Serial.begin (9600);
	while (!Serial)
    ;

  BoschBME280::Settings settings;
  settings.osr_h = BoschBME280::OVERSAMPLING_1X;
  settings.osr_p = BoschBME280::OVERSAMPLING_16X;
  settings.osr_t = BoschBME280::OVERSAMPLING_2X;
  settings.filter = BoschBME280::FILTER_COEFF_16;
  if (bme.begin(BoschBME280::I2C_ADDR_PRIM, BoschBME280::INTF_I2C, settings) != 0) {
    Serial.println(F("No BME280 sensor found"));
    while (1)
      ;
  }

  uint8_t settings_sel = BoschBME280::SEL_OSR_PRESS | BoschBME280::SEL_OSR_TEMP | BoschBME280::SEL_OSR_HUM | BoschBME280::SEL_FILTER;
  if (bme.set_sensor_settings(settings_sel) != 0) {
    Serial.println(F("Unable to set sensor settings"));
    while (1)
      ;
  }

  Serial.println(F("T\tH\tP"));
}

void loop () {
  BoschBME280::data data = {0, 0, 0};

  if (bme.set_sensor_mode(BoschBME280::MODE_FORCED) == 0) {
    /* Wait for the measurement to complete and print data @25Hz */
    delay(40);

    if (bme.get_sensor_data(BoschBME280::DATA_ALL, data) == 0) {
      Serial.print(data.temperature);
      Serial.print('\t');
      Serial.print(data.humidity);
      Serial.print('\t');
      Serial.println(data.pressure);
    } else {
      Serial.println(F("Sensor read failed"));
    }
  } else {
    Serial.println(F("Unable to set sensor mode"));
  }

  delay (1000);
}

