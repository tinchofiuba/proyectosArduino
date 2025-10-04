#include "cred.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//pines analógicos
#define pin_ph 1 //pin del ph
#define pin_EC 2 //pin del conductimetro

//pines ON /OFF
#define LL 3 // tacho ppal
#define LL 4 // tacho ppal
#define HL 5 // tacho ppal
#define HH 6 // tacho ppal
#define LL_2 7 // tacho secundario
#define HH_2 10 // tacho secundario
#define wifi 11 // habilitación de envío de datos

//pines periféricos I2C
#define pin_SDA 8
#define pin_SCL 9

//pines de servo / motores por pasos
#define pin_servo_ph 12 //servomotor para el phmetro

//pines digitales sensores
#define TRIG_PIN 13
#define ECHO_PIN 14
#define DS18B20 15
#define DHTTYPE DHT22

//pines de bombas
#define PIN_BOMBA 16
#define PIN_BOMBA_A 17
#define PIN_BOMBA_B 18
#define PIN_BOMBA_micro 21
#define PIN_BOMBA_FE 33
#define PIN_BOMBA_AGUA_RESERVA 34


OneWire oneWire(DS18B20);
DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHTTYPE);

const int numIter = 10; 
const char* serverName = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

float tAguaArray[numIter];
float humedadArray[numIter];
float tempAmbArray[numIter];
float distanciaArray[numIter];
float conductividadArray[numIter]; // C9
float phArray[numIter];        // C8

long leerDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  delay(30);
  return duration;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Configura resolución a 12 bits (0-4095)
  
  // Configurar pines de ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configurar pines de control
  pinMode(PIN_TRASMITIR, INPUT_PULLUP);
  pinMode(PIN_BOMBA, OUTPUT);
  
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

  // Leer sensores que requieren múltiples mediciones
  for (int i = 0; i < numIter; i++) {
    sensors.requestTemperatures();
    tAguaArray[i] = sensors.getTempCByIndex(0);
    
    // Leer humedad y validar NaN
    float humedad = dht.readHumidity();
    humedadArray[i] = (isnan(humedad)) ? 0.0 : humedad;
    
    // Leer temperatura ambiente y validar NaN
    float tempAmb = dht.readTemperature();
    tempAmbArray[i] = (isnan(tempAmb)) ? 0.0 : tempAmb;
    
    distanciaArray[i] = leerDistancia();
  }

  // Leer sensores digitales del multiplexor (C0-C7: valores únicos HIGH/LOW)
  hhEstado = leerSensorDigitalMUX(0);        // C0
  hEstado = leerSensorDigitalMUX(1);         // C1
  lEstado = leerSensorDigitalMUX(2);         // C2
  llEstado = leerSensorDigitalMUX(3);        // C3
  hhxEstado = leerSensorDigitalMUX(4);       // C4
  llxEstado = leerSensorDigitalMUX(5);       // C5
  x1Estado = leerSensorDigitalMUX(6);        // C6
  x2Estado = leerSensorDigitalMUX(7);        // C7

  // Leer sensores analógicos que requieren múltiples mediciones (C8-C9)
  leerSensorAnalogicoMultiple(8, phArray, numIter);             // C8 - pH
  leerSensorAnalogicoMultiple(9, conductividadArray, numIter);  // C9 - Conductividad


  // Leer estados de pines digitales
  byte bombaEstado = digitalRead(PIN_BOMBA);
  byte transmitirEstado = digitalRead(PIN_TRASMITIR);
  
  // Debug - mostrar valores
  Serial.println("=== SENSORES ===");
  Serial.print("Temperatura agua: "); Serial.println(tAguaArray[0]);
  Serial.print("Humedad: "); Serial.println(humedadArray[0]);
  Serial.print("Temperatura ambiente: "); Serial.println(tempAmbArray[0]);
  Serial.print("Distancia: "); Serial.println(distanciaArray[0]);
  Serial.print("HH: "); Serial.println(hhEstado ? "HIGH" : "LOW");
  Serial.print("H: "); Serial.println(hEstado ? "HIGH" : "LOW");
  Serial.print("L: "); Serial.println(lEstado ? "HIGH" : "LOW");
  Serial.print("LL: "); Serial.println(llEstado ? "HIGH" : "LOW");
  Serial.print("HHX: "); Serial.println(hhxEstado ? "HIGH" : "LOW");
  Serial.print("LLX: "); Serial.println(llxEstado ? "HIGH" : "LOW");
  Serial.print("X1: "); Serial.println(x1Estado ? "HIGH" : "LOW");
  Serial.print("X2: "); Serial.println(x2Estado ? "HIGH" : "LOW");
  Serial.print("TDS: "); Serial.println(conductividadArray[0]);
  Serial.print("pH: "); Serial.println(phArray[0]);
  Serial.print("Bomba: "); Serial.println(bombaEstado);
  Serial.print("Transmitir: "); Serial.println(transmitirEstado);
  Serial.println("=== FIN ===");


  // Creo un objeto JSON
  StaticJsonDocument<2000> jsonDoc; 
  
  // Arrays para sensores con múltiples mediciones
  JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
  JsonArray humedadJson = jsonDoc.createNestedArray("humedad");
  JsonArray tempAmbJson = jsonDoc.createNestedArray("tempAmb");
  JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
  JsonArray tdsJson = jsonDoc.createNestedArray("tds");
  JsonArray phJson = jsonDoc.createNestedArray("phs");

  // Relleno los arrays JSON con los datos de múltiples mediciones
  for (int i = 0; i < numIter; i++) {
    tAguaJson.add(tAguaArray[i]);
    humedadJson.add(humedadArray[i]);
    tempAmbJson.add(tempAmbArray[i]);
    distanciaJson.add(distanciaArray[i]);
    tdsJson.add(conductividadArray[i]);
    phJson.add(phArray[i]);
  }

  // Valores únicos de sensores digitales del multiplexor (C0-C7)
  jsonDoc["hh"] = hhEstado;
  jsonDoc["h"] = hEstado;
  jsonDoc["l"] = lEstado;
  jsonDoc["ll"] = llEstado;
  jsonDoc["hhx"] = hhxEstado;
  jsonDoc["llx"] = llxEstado;
  jsonDoc["x1"] = x1Estado;
  jsonDoc["x2"] = x2Estado;

  // Estados de pines digitales
  jsonDoc["bomba"] = bombaEstado;

  // TODO LO QUE VIENE ABAJO LO HAGO SI EL PIN_TRASMITIR ESTA HIGH
  if (transmitirEstado == HIGH) {
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
  delay(2000);
  }
  else{
    Serial.println("PIN_TRASMITIR no esta HIGH");
    delay(200);
  }

}