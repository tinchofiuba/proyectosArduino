#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <vector>
class Sensor {
  protected:
    String name;                // Nombre del sensor
    std::vector<int> pins;      // Lista de pines asociados al sensor

  public:
    Sensor(String sensorName, int pin) : name(sensorName), pins({pin}) {}
    Sensor(String sensorName, std::initializer_list<int> pinList) : name(sensorName), pins(pinList) {}

    virtual void inicializar() {
      if (!pins.empty()) {
        pinMode(pins[0], INPUT); // por default pin=pin[0]
      }
    }

    virtual float leer() {
      if (!pins.empty()) {
        return analogRead(pins[0]); // por default pin=pin[0]
      }
      return NAN; // Devuelve "Not a Number" si no hay pines configurados
    }

    String getName() const {
      return name;
    }

    void setName(String newName) {
      name = newName;
    }

    std::vector<int> getPins() const {
      return pins;
    }
};

#endif