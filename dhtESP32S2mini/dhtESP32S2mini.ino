#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

#define TDS_PIN 2
#define DHTPIN 3
#define DHTTYPE DHT22
#define ONE_WIRE_BUS 4
#define PH_PIN 5
#define TRIG_PIN 6
#define ECHO_PIN 7
#define HHLLA_PIN 8
#define LLLLA_PIN 9
#define BOMB_PIN 10
#define VALVULA_PIN 11



OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const char* ssid = "Telecentro-fd55";
const char* password = "VTWMK4AUKMRW";
const char* serverName = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

DHT dht(DHTPIN, DHTTYPE);

float tAguaArray[10];
float humedadArray[10];
float tempAmbArray[10];
float distanciaArray[10];
float tdsArray[10];

long leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration;
}

float leerTDS() {
  int sensorValue = analogRead(TDS_PIN);
  return sensorValue;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Configura resolución a 12 bits (0-4095)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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

  for (int i = 0; i < 10; i++) {
    sensors.requestTemperatures();
    tAguaArray[i] = sensors.getTempCByIndex(0);
    humedadArray[i] = dht.readHumidity();
    tempAmbArray[i] = dht.readTemperature();
    distanciaArray[i] = leerDistancia();
    tdsArray[i] = leerTDS();
    delay(10);
  }
  // Crear un objeto JSON
  StaticJsonDocument<20000> jsonDoc; 
  JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
  JsonArray humedadJson = jsonDoc.createNestedArray("humedad");
  JsonArray tempAmbJson = jsonDoc.createNestedArray("tempAmb");
  JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
  JsonArray tdsJson = jsonDoc.createNestedArray("tds");

  // Rellenar los arrays JSON con los datos
  for (int i = 0; i < 10; i++) {
    tAguaJson.add(tAguaArray[i]);
    humedadJson.add(humedadArray[i]);
    tempAmbJson.add(tempAmbArray[i]);
    distanciaJson.add(distanciaArray[i]);
    tdsJson.add(tdsArray[i]);
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
