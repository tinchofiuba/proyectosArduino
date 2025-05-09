#include "cred.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//pines de sensores
#define TDS_PIN 4
#define DHTPIN 6
#define ONE_WIRE_BUS 7
#define PH_PIN 1
#define TRIG_PIN 8
#define ECHO_PIN 9

//pines ON/OFF
#define HHALL_PIN 3
#define LLALL_PIN 2
#define BOMB_PIN 5
#define VALVULA_PIN 0
#define VALVULA1_PIN 10


#define DHTTYPE DHT22
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int numIter = 5; 
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
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(30);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
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
  pinMode(PH_PIN, INPUT);
  pinMode(HHALL_PIN, INPUT);
  pinMode(LLALL_PIN, INPUT);
  pinMode(BOMB_PIN, OUTPUT);
  pinMode(VALVULA1_PIN, OUTPUT);
  pinMode(VALVULA2_PIN, OUTPUT);
  dht.begin(); // Iniciar DHT
  sensors.begin(); // Iniciar sensor de temperatura del agua

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");
}

void loop() {

  for (int i = 0; i < numIter; i++) {
    sensors.requestTemperatures();
    delay(20);
    tAguaArray[i] = sensors.getTempCByIndex(0);
    delay(20);
    humedadArray[i] = dht.readHumidity();
    delay(20);
    tempAmbArray[i] = dht.readTemperature();
    delay(20);
    distanciaArray[i] = leerDistancia();
    delay(20);
    tdsArray[i] = leerTDS();
    delay(20);
    phArray[i] = leerPH();
    delay(20);
    bombaArray[i] = digitalRead(BOMB_PIN);
    HHALLArray[i] = digitalRead(HHALL_PIN);
    LLALLArray[i] = digitalRead(LLALL_PIN);
    valvula1Array[i] = digitalRead(VALVULA1_PIN);
    valvula2Array[i] = digitalRead(VALVULA2_PIN);
    delay(20);
  }
  Serial.println(tAguaArray[0]);
  Serial.println(humedadArray[0]);
  Serial.println(tempAmbArray[0]);
  Serial.println(distanciaArray[0]);
  Serial.println(tdsArray[0]);
  Serial.println(phArray[0]);
  Serial.println("fin");

  // Crear un objeto JSON
  StaticJsonDocument<20000> jsonDoc; 
  JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
  JsonArray humedadJson = jsonDoc.createNestedArray("humedad");
  JsonArray tempAmbJson = jsonDoc.createNestedArray("tempAmb");
  JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
  JsonArray tdsJson = jsonDoc.createNestedArray("tds");
  JsonArray phJson = jsonDoc.createNestedArray("phs");
  JsonArray bombaJson = jsonDoc.createNestedArray("bomba");
  JsonArray HHALLJson = jsonDoc.createNestedArray("HHALL");
  JsonArray LLALLJson = jsonDoc.createNestedArray("LLALL");
  JsonArray valvulaJson = jsonDoc.createNestedArray("valvula");


  // Rellenar los arrays JSON con los datos
  for (int i = 0; i < numIter; i++) {
    tAguaJson.add(tAguaArray[i]);
    humedadJson.add(humedadArray[i]);
    tempAmbJson.add(tempAmbArray[i]);
    distanciaJson.add(distanciaArray[i]);
    tdsJson.add(tdsArray[i]);
    phJson.add(phArray[i]);
    bombaJson.add(bombaArray[i]);
    HHALLJson.add(HHALLArray[i]);
    LLALLJson.add(LLALLArray[i]);
    valvulaJson.add(valvulaArray[i]);
  }

  // Convertir el objeto JSON a una cadena
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
    Serial.println("WiFi desconectado");
  }
}
