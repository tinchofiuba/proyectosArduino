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

float leerTemperaturaAgua() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float leerHumedad() {
  return dht.readHumidity();
}

float leerTemperaturaAmbiente() {
  return dht.readTemperature();
}

float leerTDS() {
  int sensorValue = analogRead(TDS_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  // Convertir el voltaje a TDS (esto depende de tu sensor y su calibración)
  float tdsValue = (voltage * 1000) / 2; // Ejemplo de conversión
  return tdsValue;
}