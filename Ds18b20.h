#ifndef SENSOR_DS18B20_H
#define SENSOR_DS18B20_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <map>
#include "Sensor.h"

class Ds18b20 : public Sensor {
  private:
    OneWire* oneWire; 
    DallasTemperature* sensor; 
    std::map<String, uint8_t> sensorMap; // Mapa para asociar nombres con índices

  public:
.
    Ds18b20(String sensorName, uint8_t pin)
      : Sensor(sensorName) {
      oneWire = new OneWire(pin);
      sensor = new DallasTemperature(oneWire); 
    }

    // Destructor para liberar memoria
    ~Ds18b20() {
      delete sensor;
      delete oneWire;
    }

    // Inicializar los sensores y asignar nombres personalizados
    void inicializar(std::map<String, uint8_t> nombresPersonalizados = {}) {
      sensor->begin();
      Serial.println("Sensor DS18B20 inicializado.");
      uint8_t numSensores = sensor->getDeviceCount();
      Serial.print("Número de sensores encontrados: ");
      Serial.println(numSensores);

      // Asignar nombres personalizados o generar nombres automáticamente e el caso de que no se suministre un {}
      for (uint8_t i = 0; i < numSensores; i++) {
        String sensorName;
        if (nombresPersonalizados.size() > 0) {
          // Si hay nombres personalizados, asignarlos
          for (auto& pair : nombresPersonalizados) {
            if (pair.second == i) {
              sensorName = pair.first;
              break;
            }
          }
        } else {
          // Si no hay nombres personalizados, generar nombres automáticamente
          sensorName = name + "_" + String(i); // Ejemplo: "Sensor_0", "Sensor_1"
        }

        sensorMap[sensorName] = i; // Asociar el nombre con el índice
        Serial.print("Sensor registrado: ");
        Serial.println(sensorName);
      }
    }

    // Método para leer la temperatura usando el nombre del sensor
    float leer(String sensorName) {
      if (sensorMap.find(sensorName) == sensorMap.end()) {
        Serial.println("Nombre del sensor no encontrado.");
        return NAN; // Devuelve un valor no numérico si el nombre no existe
      }

      uint8_t index = sensorMap[sensorName]; // Obtener el índice del sensor
      sensor->requestTemperatures();
      return sensor->getTempCByIndex(index); // Leer la temperatura del sensor
    }

    void setPrecision(uint8_t precision) {
      if (precision >= 9 && precision <= 12) {
        sensor->setResolution(precision);
        Serial.print("Resolución configurada a ");
        Serial.print(precision);
        Serial.println(" bits.");
      } else {
        Serial.println("Resolución inválida. Debe estar entre 9 y 12 bits.");
      }
    }
};


/*
ejemplo de uso con Arduino:

#include "Ds18b20.h"

#define ONE_WIRE_BUS 4

Ds18b20 sensorTemp("Sensor", ONE_WIRE_BUS);

void setup() {
  Serial.begin(115200); // O cualquier otra velocidad

  // Crear un mapa con nombres personalizados
  std::map<String, uint8_t> nombresPersonalizados = {
    {"SensorTempAire", 0},
    {"SensorTempAgua", 1}
  };

  sensorTemp.inicializar(nombresPersonalizados);

  sensorTemp.setPrecision(9); // No es necesario, pero puede ser útil en caso de requerir más precisión
}

void loop() {
  float tempAire = sensorTemp.leer("SensorTempAire");
  Serial.print("Temperatura de SensorTempAire: ");
  Serial.print(tempAire);
  Serial.println(" °C");
  
  float tempAgua = sensorTemp.leer("SensorTempAgua");
  Serial.print("Temperatura de SensorTempAgua: ");
  Serial.print(tempAgua);
  Serial.println(" °C");

  delay(2000); // Esperar 2 segundos antes de la siguiente lectura
}

*/
#endif