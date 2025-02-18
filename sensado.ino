#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#define ONE_WIRE_BUS 2
#define DHTPIN 3
#define TDS_PIN A0

#define DHTTYPE DHT22

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHTTYPE);



class calibrar {
  public:
    byte tiempoMedicion=15; //en segundos, tiempo de medición, a menso que se modifique
    array  
};

class Sensor {
  public:
    virtual void calibrar() = 0;
    virtual float leer() = 0;
    virtual void enviar() = 0;
};

class SensorTemperaturaAgua : public Sensor {
  public:
    void calibrar(float referencia, float valor, byte numDatos) {
      
    }

    float leer() {
      return leerTemperaturaAgua();
    }

    void enviar() {
      //enviar
    }
};

float leerTemperaturaAgua() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}
