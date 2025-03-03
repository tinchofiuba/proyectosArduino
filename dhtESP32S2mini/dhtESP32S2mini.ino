#include <WiFi.h>
#include <WebServer.h>  
#include <DHT.h>  // Librería para el sensor DHT
#include <Arduino.h>

#define DHTPIN 3 
#define DHTTYPE DHT22
#define TRIG_PIN 5 
#define ECHO_PIN 7
#define TDS_PIN 12

const char* ssid = "Telecentro-fd55";  
const char* password = "VTWMK4AUKMRW";

WebServer server(80);  

DHT dht(DHTPIN, DHTTYPE);

void handleRoot() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  long distancia = leerDistancia();
  float TDS = leerTDS();
  // Si falla la lectura, muestra un mensaje de error
  if (isnan(h) || isnan(t)) {
    server.send(500, "text/plain", "Error al leer el sensor DHT!");
    return;
  }

  // Página HTML con los valores en vivo
  String html = "<html><head><meta http-equiv='refresh' content='5'></head><body>";
  html += "<h1>ESP32-S2 - Monitoreo DHT</h1>";
  html += "<p><b>Temperatura:</b> " + String(t,2) + "</p>";
  html += "<p><b>Humedad:</b> " + String(h,2) + "</p>";
  html += "<p><b>Distancia:</b> " + String(distancia) + " mm</p>";
  html += "<p><b>Conductividad:</b> " + String(TDS,2) + " uS</p>";
  html += "</body></html>";

  server.send(500, "text/html", html);
}

long leerDistancia() {
  long to=micros();
  while(micros()-to<10)
  {digitalWrite(TRIG_PIN, HIGH);}
   digitalWrite(TRIG_PIN, LOW);  

    // wait for echo pulse to go high
    while(!digitalRead(ECHO_PIN)) ;  
    // start timer
    unsigned long StartTime = micros();
    // wait for end of echo pulse (goes low)
    while(digitalRead(ECHO_PIN)) ;
    unsigned long CurrentTime = micros();
    // calculate length of echo pulse in microseconds
    unsigned long distancia = (CurrentTime - StartTime)*0.1911-37.21;
  delay(1000);
  return distancia;
}

float leerTDS() {
  int sensorValue = analogRead(TDS_PIN);
  float voltage = sensorValue * (5.0 / 4096);
  // Convertir el voltaje a TDS (esto depende de tu sensor y su calibración)
  float tdsValue = (voltage * 1000) / 2; // Ejemplo de conversión
  return tdsValue*1.52-36.13;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // Configura resolución a 12 bits (0-4095)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  dht.begin();  // Iniciar DHT
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
}
