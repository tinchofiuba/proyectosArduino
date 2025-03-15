#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTPIN 5
#define DHTTYPE DHT22
#define TRIG_PIN 7
#define ECHO_PIN 6
#define TDS_PIN 2
#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const char* ssid = "Telecentro-fd55";
const char* password = "VTWMK4AUKMRW";

DHT dht(DHTPIN, DHTTYPE);

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
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    sensors.requestTemperatures();
    float tAgua = sensors.getTempCByIndex(0);
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    long distancia = leerDistancia();
    int TDS = leerTDS();
    Serial.println(tAgua);
    Serial.println(h);
    Serial.println(t);
    Serial.println(TDS);
    Serial.println(distancia);

    if (isnan(h) || isnan(t)) {
      Serial.println("Error al leer el sensor DHT!");
      return;
    }

    String serverPath = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

    http.begin(serverPath.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String httpRequestData = "temp amb=" + String(t, 2) + "&humedad=" + String(h, 2) + "&distancia=" + String(distancia) + "&conductividad=" + String(TDS) + "&temp agua=" + String(tAgua, 2);
    int httpResponseCode = http.POST(httpRequestData);

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

  delay(5000); // Esperar 5 segundos antes de enviar los datos nuevamente
}