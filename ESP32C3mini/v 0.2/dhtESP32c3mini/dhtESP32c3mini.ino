#include "cred.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//pines del multiplexor
#define pin_S0 0
#define pin_S1 1
#define pin_S2 2
#define pin_S3 3
//pin de sensores analogicos y de on/off de nivel
#define PIN_ANALOGICO 4

//pin de ultrasonico
#define TRIG_PIN 5
#define ECHO_PIN 6
//pin de dht
#define DHTPIN 7
//pin de ds18b20
#define ONE_WIRE_BUS_PIN 8
//pin que habilita la trasmision de datos
#define PIN_TRASMITIR 9
//pin de bomba
#define PIN_BOMBA 10


#define DHTTYPE DHT22
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

const int numIter = 10; 
const char* serverName = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

DHT dht(DHTPIN, DHTTYPE);

float tAguaArray[numIter];
float humedadArray[numIter];
float tempAmbArray[numIter];
float distanciaArray[numIter];

// Arrays para sensores del multiplexor
// Sensores digitales (HIGH/LOW) - valores únicos
byte hhEstado;        // C0
byte hEstado;         // C1
byte lEstado;         // C2
byte llEstado;        // C3
byte hhxEstado;       // C4
byte llxEstado;       // C5
byte x1Estado;        // C6
byte x2Estado;        // C7

// Sensores analógicos - arrays con múltiples mediciones
float conductividadArray[numIter]; // C8
float phArray[numIter];        // C9

// Función para seleccionar canal del multiplexor
void seleccionarCanalMUX(int canal) {
  digitalWrite(pin_S0, canal & 0x01);
  digitalWrite(pin_S1, (canal >> 1) & 0x01);
  digitalWrite(pin_S2, (canal >> 2) & 0x01);
  digitalWrite(pin_S3, (canal >> 3) & 0x01);
  delay(1); // Pequeña pausa para estabilizar
}

// Función para leer sensor analógico del multiplexor
float leerSensorAnalogicoMUX(int canal) {
  seleccionarCanalMUX(canal);
  int valor = analogRead(PIN_ANALOGICO);
  return (float)valor;
}

// Función para leer sensor digital del multiplexor
byte leerSensorDigitalMUX(int canal) {
  seleccionarCanalMUX(canal);
  int valor = analogRead(PIN_ANALOGICO);
  // Convertir valor analógico a digital (HIGH/LOW)
  // Asumiendo que valores > 2048 son HIGH (considerando 12 bits)
  return (valor > 2048) ? HIGH : LOW;
}

// Función para leer múltiples mediciones de un sensor analógico
void leerSensorAnalogicoMultiple(int canal, float* array, int iteraciones) {
  for (int i = 0; i < iteraciones; i++) {
    array[i] = leerSensorAnalogicoMUX(canal);
    delay(10); // Pequeña pausa entre mediciones
  }
}

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
  
  // Configurar pines del multiplexor
  pinMode(pin_S0, OUTPUT);
  pinMode(pin_S1, OUTPUT);
  pinMode(pin_S2, OUTPUT);
  pinMode(pin_S3, OUTPUT);
  
  // Configurar pin analógico
  pinMode(PIN_ANALOGICO, INPUT);
  
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
  leerSensorAnalogicoMultiple(8, conductividadArray, numIter);  // C8 - Conductividad
  leerSensorAnalogicoMultiple(9, phArray, numIter);             // C9 - pH

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
  delay(4000);
  }
  else{
    Serial.println("PIN_TRASMITIR no esta HIGH");
    delay(4000);
  }

}