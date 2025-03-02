//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <DHT.h>
//#inclue <WiFi.h>
//#include <HTTPClient.h>

//urlPagina="tinchofiuba.pythonanywhere.com"

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





void setup() {
  Serial.begin(9600);
  sensors.begin();
  dht.begin();
  WiFi.begin("SSID", "PASSWORD");
}

void loop() {
  float temperaturaAgua = leerTemperaturaAgua();
  float humedad = leerHumedad();
  float temperaturaAmbiente = leerTemperaturaAmbiente();
  float tds = leerTDS();



  Serial.print("Temperatura del agua: ");
  Serial.print(temperaturaAgua);
  Serial.println(" °C");

  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.println(" %");

  Serial.print("Temperatura ambiente: ");
  Serial.print(temperaturaAmbiente);
  Serial.println(" °C");

  Serial.print("TDS: ");
  Serial.print(tds);
  Serial.println(" ppm");

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String postData = "fecha=23-2-2025";

    http.begin(serverName);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    int httpResponseCode = http.POST(postData);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error en la solicitud: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Error de conexión WiFi");
  }

  delay(1000);
}
