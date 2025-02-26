
#define TDS_PIN A0

float leerTDS() {
  int sensorValue = analogRead(TDS_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  // Convertir el voltaje a TDS (esto depende de tu sensor y su calibración)
  float tdsValue = (voltage * 1000) / 2; // Ejemplo de conversión
  return tdsValue;
}





void setup() {
  Serial.begin(9600);
}

void loop() {
  float tds = leerTDS();

  Serial.print("TDS: ");
  Serial.print(tds);
  Serial.println(" ppm");

  delay(1000);
}
