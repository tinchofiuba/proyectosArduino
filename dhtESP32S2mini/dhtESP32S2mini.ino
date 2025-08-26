#include "cred.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//pines de sensores analógicos
#define TDS_PIN 1
#define PH_PIN 2

//activadores analógicos
#define wifi_ON_Off 3
#define control_On_Off 4

//pines digitales ON / OFF de nivel
#define HHALL_PIN 11 //pines en batea de recolección
#define LLALL_PIN 12 //pines en batea de recolección
//
#define HHALL_PIN 13 //pines en batea de bombeo
#define LALL_PIN 14 //pines en batea de bombeo
#define HALL_PIN 15 //pines en batea de bombeo
#define LLALL_PIN 16 //pines en batea de bombeo

//pines de sensores y/o actuadores digitales
#define DHTPIN 17
#define BOMB_PIN 18
#define VALVULA_PIN 19
#define ONE_WIRE_BUS 20
#define TRIG_PIN 21
#define ECHO_PIN 22

#define DHTTYPE DHT22
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int numIter = 8; 
const char* serverName = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

DHT dht(DHTPIN, DHTTYPE);

float tAguaArray[numIter];
float humedadArray[numIter];
float tempAmbArray[numIter];
float distanciaArray[numIter];
float tdsArray[numIter];
float phArray[numIter];

long leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  delay(120);
  return duration;
}

float leerTDS() {
  int sensorValue = analogRead(TDS_PIN);
  return sensorValue;
}

float leerPH() {
  int sensorValue = analogRead(PH_PIN);
  return sensorValue;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Configura resolución a 12 bits (0-4095)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TDS_PIN, INPUT);
  pinMode(HHALL_PIN, INPUT);
  pinMode(LLALL_PIN, INPUT);
  pinMode(BOMB_PIN, OUTPUT);
  pinMode(VALVULA_PIN, OUTPUT);
  dht.begin(); 
  sensors.begin(); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");
}

void loop() {

  for (int i = 0; i < numIter; i++) {
    sensors.requestTemperatures();
    tAguaArray[i] = sensors.getTempCByIndex(0);
    humedadArray[i] = dht.readHumidity();
    tempAmbArray[i] = dht.readTemperature();
    distanciaArray[i] = leerDistancia();
    tdsArray[i] = leerTDS();
    phArray[i] = leerPH();
  }

  byte bombaEstado = digitalRead(BOMB_PIN);
  byte HHALLEstado = digitalRead(HHALL_PIN);
  byte LLALLEstado = digitalRead(LLALL_PIN);
  byte valvulaEstado = digitalRead(VALVULA_PIN);
  
  Serial.println(tAguaArray[0]);
  Serial.println(humedadArray[0]);
  Serial.println(tempAmbArray[0]);
  Serial.println(distanciaArray[0]);
  Serial.println(tdsArray[0]);
  Serial.println(phArray[0]);
  Serial.println("fin");


  // Creo un objeto JSON
  StaticJsonDocument<1000> jsonDoc; 
  JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
  JsonArray humedadJson = jsonDoc.createNestedArray("humedad");
  JsonArray tempAmbJson = jsonDoc.createNestedArray("tempAmb");
  JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
  JsonArray tdsJson = jsonDoc.createNestedArray("tds");
  JsonArray phJson = jsonDoc.createNestedArray("phs");

  // Relleno los arrays JSON con los datos
  for (int i = 0; i < numIter; i++) {
    tAguaJson.add(tAguaArray[i]);
    humedadJson.add(humedadArray[i]);
    tempAmbJson.add(tempAmbArray[i]);
    distanciaJson.add(distanciaArray[i]);
    tdsJson.add(tdsArray[i]);
    phJson.add(phArray[i]);
  }

  jsonDoc["bomba"] = bombaEstado;
  jsonDoc["HHALL"] = HHALLEstado;
  jsonDoc["LLALL"] = LLALLEstado;
  jsonDoc["valvula"] = valvulaEstado;

  // Convierto el objeto JSON a una cadena
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonString);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error en la solicitud POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi desconectado! reconectando...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(3000);
      Serial.println("Reconectando a WiFi...");
    }
    Serial.println("WiFi reconectado");
  }
  delay(4000);
}
