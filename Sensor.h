#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

class Sensor {
    protected:
    String name;
    public:
    Sensor(String sensorName) : name(sensorName) {};
    virtual void inicializar() = 0;
    virtual float leer() = 0;
};
#endif