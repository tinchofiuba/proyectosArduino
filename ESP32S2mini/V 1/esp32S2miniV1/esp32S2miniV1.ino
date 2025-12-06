#include "cred.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define AS5600_ADDR 0x36  // Dirección I2C por defecto del AS5600

//pines analógicos
#define PIN_PH 3 //pin del ph (ADC1_2)
#define PIN_EC 5 //pin del conductimetro (ADC1_4)xx,,,mm    

//pines ON /OFF
#define LL 33 // tacho ppal
#define L 35 // tacho ppal
#define H 37 // tacho ppal
#define HH 39 // tacho ppal

#define PIN_TRASMITIR 7 // habilitación de envío de datos

int pines_on_off[] = {LL, L, H, HH, PIN_TRASMITIR};
int n_pines_on_off = sizeof(pines_on_off) / sizeof(pines_on_off[0]);

//pines periféricos I2C
#define pin_SDA 8
#define pin_SCL 9

//pines digitales sensores
#define DS18B20 11 // pin 18 no puede usar pull-up/pull-down OJO

//pines de bombas y servomotores y/o motores
#define PIN_SERVO_PH 24 //servomotor para el phmetro
#define PIN_BOMBA_PPAL 14
#define PIN_AIREADOR 12
#define PIN_BOMBA_A 10
#define PIN_BOMBA_B 6
#define PIN_BOMBA_micro 4
#define PIN_BOMBA_FE 2
#define PIN_BOMBA_AGUA_RESERVA 1

//queda el GPIO7 libre, o bien el ADC1_6 libre


OneWire oneWire(DS18B20);
DallasTemperature sensors(&oneWire);

const int numIter = 10; 

// ============================================================================
// CONFIGURACIÓN MQTT
// ============================================================================
// Cliente WiFi seguro para MQTT con TLS
WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

// Semáforo para proteger variables compartidas (estados de bombas)
SemaphoreHandle_t mutexBombas;

float tAguaArray[numIter];
float tempAmbArray[numIter];
float distanciaArray[numIter];
float conductividadArray[numIter]; // C9
float phArray[numIter];        // C8

// Variables de estado de las bombas
bool bombaPpalEstado = false;
bool bombaAEstado = false;
bool bombaBEstado = false;
bool bombaMicroEstado = false;
bool bombaFEEstado = false;
bool bombaAguaReservaEstado = false;
bool servoMotorPhEstado = false;

float leerAngulo() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // Dirección del registro RAW ANGLE (MSB)
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return -1;

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();
  uint16_t valor = ((highByte << 8) | lowByte) & 0x0FFF;  // 12 bits

  float angulo = (valor * 360.0) / 4096.0;
  if (angulo > 0 )
  {
  return angulo;
  }
  else
  {
    return 3.1416; //error de lectura I2C
  }
  delay(100);

}

// ============================================================================
// FUNCIONES MQTT
// ============================================================================

// Reconectar a MQTT si se pierde la conexión
void reconectarMQTT() {
  while (!mqttClient.connected()) {
    // Serial.print("🔄 Intentando conectar a MQTT...");
    
    if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
      // Serial.println("✅ Conectado a MQTT!");
    } else {
      // Serial.print("❌ Falló conexión MQTT, código: ");
      // Serial.print(mqttClient.state());
      // Serial.println(". Reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

// Enviar datos de sensores al topic MQTT
void enviarDatosMQTT(String jsonString) {
  if (!mqttClient.connected()) {
    reconectarMQTT();
  }
  
  mqttClient.loop(); // Mantener conexión activa
  
  if (mqttClient.publish(mqttTopicSensores, jsonString.c_str())) {
    // Serial.println("✅ Datos enviados a MQTT");
  } else {
    // Serial.println("❌ Error enviando datos a MQTT");
  }
}



void setup() {
  Serial.begin(115200);
  analogReadResolution(12); //Configura resolución a 12 bits (0-4095)
  analogSetAttenuation(ADC_11db); //Configura rango de medición 0-3.6V (IMPORTANTE)
  Wire.begin();  //Inicia I2C

  //Configurar pines de bombas y establecer estados iniciales
  pinMode(PIN_BOMBA_PPAL, OUTPUT);
  bombaPpalEstado = true;  //Bomba principal ON por defecto
  digitalWrite(PIN_BOMBA_PPAL, HIGH);
  
  pinMode(PIN_BOMBA_A, OUTPUT); 
  bombaAEstado = false;  //Apagada por defecto
  digitalWrite(PIN_BOMBA_A, LOW);
  
  pinMode(PIN_BOMBA_B, OUTPUT);
  bombaBEstado = false;  //Apagada por defecto
  digitalWrite(PIN_BOMBA_B, LOW);
  
  pinMode(PIN_BOMBA_micro, OUTPUT);
  bombaMicroEstado = false;  //Apagada por defecto
  digitalWrite(PIN_BOMBA_micro, LOW);
  
  pinMode(PIN_BOMBA_FE, OUTPUT);
  bombaFEEstado = false;  //Apagada por defecto
  digitalWrite(PIN_BOMBA_FE, LOW);
  
  pinMode(PIN_BOMBA_AGUA_RESERVA, OUTPUT);
  bombaAguaReservaEstado = false;  //Apagada por defecto
  digitalWrite(PIN_BOMBA_AGUA_RESERVA, LOW);
  
  //seteo pull-down a todos los pines de on/off
  for (int i = 0; i < n_pines_on_off; i++) {
    pinMode(pines_on_off[i], INPUT_PULLDOWN);
  }
  // instancio e inicio el DS18B20
  sensors.begin(); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    // Serial.println("Conectando a WiFi...");
  }
  // Serial.println("Conectado a WiFi");
  // Serial.print("📍 IP: ");
  // Serial.println(WiFi.localIP());
  
  // Configurar cliente seguro para MQTT con TLS
  secureClient.setInsecure();  // Ignorar certificado SSL (solo para pruebas)
  
  // Configurar cliente MQTT
  mqttClient.setServer(mqttBrokerHost, mqttBrokerPort);
  
  // Conectar a MQTT
  reconectarMQTT();
  
  // Crear semáforo mutex para proteger variables compartidas
  mutexBombas = xSemaphoreCreateMutex();
  if (mutexBombas == NULL) {
    // Serial.println("❌ Error creando semáforo mutex");
  }
  
  // Serial.println("🌐 Sistema MQTT configurado");
  // Serial.println("📊 Enviando datos al topic: hidroponia/sensores");
  
}

void loop() {
    // ========================================================================
    // LECTURA DE SENSORES Y ENVÍO A MQTT
    // ========================================================================
    for (int i = 0; i < numIter; i++) {
      sensors.requestTemperatures();
      tAguaArray[i] = sensors.getTempCByIndex(0);
      
      distanciaArray[i] = leerAngulo();
      conductividadArray[i] = analogRead(PIN_EC); // leo la conductividad
      phArray[i] = analogRead(PIN_PH); // leo el ph
    }

    StaticJsonDocument<2000> jsonDoc; 
    JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
    JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
    JsonArray tdsJson = jsonDoc.createNestedArray("tds");
    JsonArray phJson = jsonDoc.createNestedArray("phs");

    for (int i = 0; i < numIter; i++) {
      tAguaJson.add(tAguaArray[i]);
      distanciaJson.add(distanciaArray[i]);
      tdsJson.add(conductividadArray[i]);
      phJson.add(phArray[i]);
    }

    byte transmitirEstado = digitalRead(PIN_TRASMITIR);
    
    byte hhEstado = digitalRead(HH);
    byte hEstado = digitalRead(H);
    byte lEstado = digitalRead(L);
    byte llEstado = digitalRead(LL);

    // Serial.println("=== SENSORES ===");
    // Serial.print("Temperatura agua: "); Serial.println(tAguaArray[0]);
    // Serial.print("Distancia: "); Serial.println(distanciaArray[0]);
    // Serial.print("HH: "); Serial.println(hhEstado ? "encontacto" : "libre");
    // Serial.print("H: "); Serial.println(hEstado ? "encontacto" : "libre");
    // Serial.print("L: "); Serial.println(lEstado ? "encontacto" : "libre");
    // Serial.print("LL: "); Serial.println(llEstado ? "encontacto" : "libre");
    // Serial.print("TDS: "); Serial.println(conductividadArray[0]);
    // Serial.print("pH: "); Serial.println(phArray[0]);
    // Serial.print("Bomba ppal: "); Serial.println(bombaPpalEstado ? "ON" : "OFF");
    // Serial.print("Bomba A: "); Serial.println(bombaAEstado ? "ON" : "OFF");
    // Serial.print("Bomba B: "); Serial.println(bombaBEstado ? "ON" : "OFF");
    // Serial.print("Bomba micro: "); Serial.println(bombaMicroEstado ? "ON" : "OFF");
    // Serial.print("Bomba FE: "); Serial.println(bombaFEEstado ? "ON" : "OFF");
    // Serial.print("Bomba agua reserva: "); Serial.println(bombaAguaReservaEstado ? "ON" : "OFF");
    // Serial.print("Transmitir: "); Serial.println(transmitirEstado);
    // Serial.println("=== FIN ===");



    jsonDoc["HH"] = hhEstado;
    jsonDoc["H"] = hEstado;
    jsonDoc["L"] = lEstado;
    jsonDoc["LL"] = llEstado;

    // Proteger lectura de variables compartidas de bombas
    if (xSemaphoreTake(mutexBombas, portMAX_DELAY) == pdTRUE) {
      jsonDoc["bomba_ppal"] = bombaPpalEstado;
      jsonDoc["bombaA"] = bombaAEstado;
      jsonDoc["bombaB"] = bombaBEstado;
      jsonDoc["bombaMicro"] = bombaMicroEstado;
      jsonDoc["bombaFE"] = bombaFEEstado;
      jsonDoc["bombaAguaReserva"] = bombaAguaReservaEstado;
      xSemaphoreGive(mutexBombas);
    }

    //TODO LO QUE VIENE ABAJO LO HAGO SI EL PIN_TRASMITIR ESTA LOW
    // Serial.print("transmitirEstado: "); Serial.println(transmitirEstado);
    if (transmitirEstado == LOW) {

      String jsonString;
      serializeJson(jsonDoc, jsonString);
      
      if (WiFi.status() == WL_CONNECTED) {
        // Enviar datos al topic MQTT
        enviarDatosMQTT(jsonString);
      } else {
        // Serial.println("WiFi desconectado! reconectando...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
          delay(3000);
          // Serial.println("Reconectando a WiFi...");
        }
        // Serial.println("WiFi reconectado");
        reconectarMQTT(); // Reconectar MQTT después de reconectar WiFi
      }
      
      // Delay de 5 segundos entre envíos (emulando mediciones)
      delay(5000);
    }
    else{
      // Serial.println("PIN_TRASMITIR NO HABILITA LA TRANSMISION DE DATOS");
      delay(200);
    }

    // Mantener conexión MQTT activa
    if (mqttClient.connected()) {
      mqttClient.loop();
    }
    
    // Ceder tiempo de CPU a otras tareas
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
