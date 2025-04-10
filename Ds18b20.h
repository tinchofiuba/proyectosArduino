#ifndef SENSOR_DS18B20_H
#define SENSOR_DS18B20_H

#include <Arduino.h>
#include <DallasTemperature.h>
#include "Sensor.h"

// Clase específica para el sensor de temperatura
class Ds18b20 : public Sensor {
  private:
    DallasTemperature* sensor;

  public:
  Ds18b20(String sensorName, DallasTemperature* DSsensor)
      : Sensor(sensorName), sensor(DSsensor) {}

    void inicializar() override {
      sensor->begin();
    }

    float leer(uint8_t index = 0) override {
      if (index >= sensor->getDeviceCount()) {
        Serial.println("Índice fuera de rango. No se puede leer el sensor.");
        return NAN; // Devuelve un valor no numérico si el índice es inválido
      }
      sensor->requestTemperatures();
      return sensor->getTempCByIndex(index); // Lee la temperatura del sensor especificado
    }

    void setPrecision(uint8_t precision) {
        if (precision >= 9 && precision <= 12) {
            sensor->setResolution(precision); // Configura la resolución del sensor
          } else {
            Serial.println("Resolución inválida. Debe estar entre 9 y 12 bits.");
          }
    }


};

#endif