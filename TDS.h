#ifndef TDS_H
#define TDS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Sensor.h"
#include <FS.h>      
#include <SPIFFS.h>  

class TDS : public Sensor {
  private:
    float m; 
    float b; 
    const char* sensorNameCStr;

  public:
    TDS(String sensorName, int pin) : Sensor(sensorName, pin), m(2), b(0.0) {
        sensorNameCStr = name.c_str(); // Convertir name a const char* al construir el objeto
        }
    void inicializar() override {
        pinMode(pins[0], INPUT); 
        Serial.println("Inicializando sensor TDS...");

        File configFile = SPIFFS.open("/configSensores.json", "r");
        if (configFile) {
            StaticJsonDocument<1024> jsonDoc;
            DeserializationError error = deserializeJson(jsonDoc, configFile);
            configFile.close();

            if (!error && jsonDoc.containsKey(sensorNameCStr)) {
            m = jsonDoc[sensorNameCStr]["calibracion"]["m"] | 2.0; 
            b = jsonDoc[sensorNameCStr]["calibracion"]["b"] | 0.0; 
            Serial.println("Calibración cargada:");
            Serial.print("m: ");
            Serial.println(m);
            Serial.print("b: ");
            Serial.println(b);
            } else {
            Serial.println("No se encontró calibración previa. Usando valores por defecto.");
            }
        } else {
            Serial.println("No se pudo abrir configSensores.json. Usando valores por defecto.");
        }
    }

    float leer() override {
      if (!pins.empty()) {
        int rawValue = analogRead(pins[0]);
        return m * rawValue + b; 
      }
      return NAN; // en caso de no haber colocado pines devuelvo NaN
    }

    void calibrar(float nuevoM, float nuevoB) {
      m = nuevoM;
      b = nuevoB;

      StaticJsonDocument<1024> jsonDoc;
      File configFile = SPIFFS.open("/configSensores.json", "r");
      if (configFile) {
        deserializeJson(jsonDoc, configFile);
        configFile.close();
      }

      jsonDoc[sensorNameCStr]["calibracion"]["m"] = m;
      jsonDoc[sensorNameCStr]["calibracion"]["b"] = b;

      configFile = SPIFFS.open("/configSensores.json", "w");
      if (configFile) {
        serializeJson(jsonDoc, configFile);
        configFile.close();
        Serial.println("Calibración guardada en configSensores.json");
      } else {
        Serial.println("Error al guardar la calibración.");
      }
    }
};

#endif