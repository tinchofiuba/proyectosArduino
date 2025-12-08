#include "cred.h"
#include <WiFi.h>
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
// CONFIGURACIÓN MQTT - Mosquitto público
// ============================================================================
const char* mqttBrokerHost = "test.mosquitto.org";
const int mqttBrokerPort = 1883;
const char* mqttTopicSensores = "hidroponia/sensores";
const char* mqttClientId = "esp32s2mini_hidroponia";

// Cliente WiFi y MQTT (sin TLS para Mosquitto público)
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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
  // Verificar WiFi primero
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MQTT] ⚠️ WiFi desconectado, no se puede reconectar MQTT");
    return;
  }
  
  int intentos = 0;
  while (!mqttClient.connected() && intentos < 5) {
    Serial.print("[MQTT] Intentando reconectar... (intento ");
    Serial.print(intentos + 1);
    Serial.print("/5)");
    Serial.print(" | Broker: ");
    Serial.print(mqttBrokerHost);
    Serial.print(":");
    Serial.println(mqttBrokerPort);
    
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("[MQTT] ✅ Reconectado exitosamente");
      Serial.print("[MQTT] Topic: ");
      Serial.println(mqttTopicSensores);
      return;
    } else {
      int estado = mqttClient.state();
      Serial.print("[MQTT] ❌ Error de conexión, código: ");
      Serial.print(estado);
      Serial.print(" (");
      // Explicar el código de error
      switch(estado) {
        case -4: Serial.print("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.print("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.print("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.print("MQTT_DISCONNECTED"); break;
        case 1: Serial.print("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2: Serial.print("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3: Serial.print("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4: Serial.print("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5: Serial.print("MQTT_CONNECT_UNAUTHORIZED"); break;
        default: Serial.print("Desconocido"); break;
      }
      Serial.println(")");
      intentos++;
      delay(2000);
    }
  }
  // Si no se conecta después de 5 intentos, continuar sin MQTT
  Serial.println("[MQTT] ⚠️ No se pudo reconectar después de 5 intentos");
}

// Enviar datos de sensores al topic MQTT
void enviarDatosMQTT(String jsonString) {
  // Verificar WiFi primero
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MQTT] ❌ WiFi desconectado, no se pueden enviar datos");
    return;
  }
  
  // Mantener conexión MQTT activa
  mqttClient.loop();
  
  // Verificar conexión MQTT
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] ⚠️ Cliente desconectado, intentando reconectar...");
    reconectarMQTT();
    
    // Verificar nuevamente después de intentar reconectar
    if (!mqttClient.connected()) {
      Serial.print("[MQTT] ❌ No se pudo reconectar. Estado: ");
      Serial.println(mqttClient.state());
      return; // Salir si no está conectado
    }
  }
  
  // Log del mensaje a enviar
  Serial.println("[MQTT] 📤 Publicando datos...");
  Serial.print("[MQTT] Topic: ");
  Serial.println(mqttTopicSensores);
  Serial.print("[MQTT] Payload: ");
  Serial.println(jsonString);
  
  // Intentar publicar
  bool publicado = mqttClient.publish(mqttTopicSensores, jsonString.c_str());
  
  if (!publicado) {
    Serial.print("[MQTT] ❌ Error enviando datos. Estado: ");
    Serial.print(mqttClient.state());
    Serial.print(" | Conectado: ");
    Serial.println(mqttClient.connected() ? "Sí" : "No");
  } else {
    Serial.println("[MQTT] ✅ Datos enviados exitosamente");
  }
}



void setup() {
  Serial.begin(115200);
  delay(1000); // Esperar a que Serial esté listo
  Serial.println("=== INICIO SETUP ===");
  
  Serial.println("Paso 1: Configurando ADC...");
  analogReadResolution(12); //Configura resolución a 12 bits (0-4095)
  analogSetAttenuation(ADC_11db); //Configura rango de medición 0-3.6V (IMPORTANTE)
  
  Serial.println("Paso 2: Iniciando I2C...");
  Wire.begin();  //Inicia I2C

  Serial.println("Paso 3: Configurando pines de bombas...");
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
  
  Serial.println("Paso 4: Configurando pines de sensores...");
  //seteo pull-down a todos los pines de on/off
  for (int i = 0; i < n_pines_on_off; i++) {
    pinMode(pines_on_off[i], INPUT_PULLDOWN);
  }
  
  Serial.println("Paso 5: Iniciando DS18B20...");
  // instancio e inicio el DS18B20
  sensors.begin(); 

  Serial.println("1. Conectando a WiFi...");
  WiFi.begin(ssid, password);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    intentos++;
    Serial.print("   Intentando WiFi... (");
    Serial.print(intentos);
    Serial.println(")");
    if (intentos > 10) {
      Serial.println("[WiFi] ❌ Error: No se pudo conectar después de 10 intentos");
      break;
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] ❌ Error: No conectado");
  } else {
    Serial.println("   WiFi conectado!");
  }
  
  Serial.println("2. Configurando MQTT...");
  // Configurar cliente MQTT (sin TLS para Mosquitto público)
  mqttClient.setServer(mqttBrokerHost, mqttBrokerPort);
  mqttClient.setBufferSize(2048);  // Buffer más grande para mensajes JSON
  mqttClient.setKeepAlive(60);     // Keepalive de 60 segundos
  
  Serial.println("3. Conectando a MQTT...");
  Serial.print("   Broker: ");
  Serial.print(mqttBrokerHost);
  Serial.print(":");
  Serial.println(mqttBrokerPort);
  Serial.print("   Client ID: ");
  Serial.println(mqttClientId);
  
  // Conectar a MQTT (con timeout para no bloquear)
  int intentosMQTT = 0;
  while (!mqttClient.connected() && intentosMQTT < 5) {
    Serial.print("   Intento ");
    Serial.print(intentosMQTT + 1);
    Serial.println(" de conexión...");
    
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("   ✅ MQTT conectado!");
      Serial.print("   Estado: ");
      Serial.println(mqttClient.state());
    } else {
      Serial.print("[MQTT] ❌ Error de conexión, código: ");
      int estado = mqttClient.state();
      Serial.print(estado);
      Serial.print(" (");
      // Explicar el código de error
      switch(estado) {
        case -4: Serial.print("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.print("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.print("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.print("MQTT_DISCONNECTED"); break;
        case 1: Serial.print("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2: Serial.print("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3: Serial.print("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4: Serial.print("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5: Serial.print("MQTT_CONNECT_UNAUTHORIZED"); break;
        default: Serial.print("Desconocido"); break;
      }
      Serial.println(")");
      intentosMQTT++;
      delay(3000);
    }
  }
  
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] ⚠️ No conectado, continuando sin MQTT...");
  }
  
  // Crear semáforo mutex para proteger variables compartidas
  mutexBombas = xSemaphoreCreateMutex();
  if (mutexBombas == NULL) {
    Serial.println("[SETUP] ❌ Error creando semáforo mutex");
  }
  
  Serial.println("=== FIN SETUP ===");
  
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
      
      // Verificar WiFi
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] ❌ Desconectado, reconectando...");
        WiFi.begin(ssid, password);
        int intentosReconexion = 0;
        while (WiFi.status() != WL_CONNECTED && intentosReconexion < 5) {
          delay(3000);
          intentosReconexion++;
          Serial.print("[WiFi] Intentando reconectar... (");
          Serial.print(intentosReconexion);
          Serial.println("/5)");
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("[WiFi] ✅ Reconectado exitosamente");
          reconectarMQTT(); // Reconectar MQTT después de reconectar WiFi
        } else {
          Serial.println("[WiFi] ❌ Error: No se pudo reconectar");
        }
      }
      
      // Si WiFi está conectado, intentar enviar por MQTT
      if (WiFi.status() == WL_CONNECTED) {
        // Verificar conexión MQTT antes de enviar
        if (!mqttClient.connected()) {
          Serial.println("[MQTT] ⚠️ Desconectado, intentando reconectar...");
          reconectarMQTT();
        }
        
        // Enviar datos al topic MQTT
        enviarDatosMQTT(jsonString);
      }
      
      // Delay de 5 segundos entre envíos (emulando mediciones)
      delay(5000);
    }
    else{
      // Serial.println("PIN_TRASMITIR NO HABILITA LA TRANSMISION DE DATOS");
      delay(200);
    }

    // Mantener conexión MQTT activa siempre (llamar frecuentemente)
    // Esto es importante para mantener la conexión viva
    if (WiFi.status() == WL_CONNECTED) {
      mqttClient.loop();
    }
    
    // Ceder tiempo de CPU a otras tareas
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

