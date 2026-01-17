#include "cred.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
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
const char* serverName = "http://tinchofiuba.pythonanywhere.com/hidroponia/";

// ============================================================================
// CONFIGURACIÓN PARA COMUNICACIÓN CON BACKEND (POLLING)
// ============================================================================
const char* djangoServer = "https://tinchofiuba.pythonanywhere.com";
const char* consultarEndpoint = "/consultar-mensajes/";
const char* confirmacionEndpoint = "/esp32-confirmacion/";

// Variables para polling de mensajes
const unsigned long intervaloConsulta = 500;   // Consultar cada 0.5 segundos

// Cliente HTTPS seguro
WiFiClientSecure secureClient;

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
// FUNCIONES PARA COMUNICACIÓN CON BACKEND (POLLING)
// ============================================================================

// Mapear nombres del frontend a variables de estado del ESP32
void controlarBomba(const char* bombaNombre, bool estado) {
  // Proteger acceso a variables compartidas
  if (xSemaphoreTake(mutexBombas, portMAX_DELAY) == pdTRUE) {
    // Serial.print("🔧 Comando recibido: Bomba '");
    // Serial.print(bombaNombre);
    // Serial.print("' -> ");
    // Serial.println(estado ? "ON" : "OFF");
    
    if (strcmp(bombaNombre, "recirculado") == 0) {
      // Serial.println("  → Mapeado a: Bomba Principal (PIN_BOMBA_PPAL)");
      digitalWrite(PIN_BOMBA_PPAL, estado ? HIGH : LOW);
      bombaPpalEstado = estado;
    }
    else if (strcmp(bombaNombre, "solucionA") == 0) {
      // Serial.println("  → Mapeado a: Bomba A (PIN_BOMBA_A)");
      digitalWrite(PIN_BOMBA_A, estado ? HIGH : LOW);
      bombaAEstado = estado;
    }
    else if (strcmp(bombaNombre, "solucionB") == 0) {
      // Serial.println("  → Mapeado a: Bomba B (PIN_BOMBA_B)");
      digitalWrite(PIN_BOMBA_B, estado ? HIGH : LOW);
      bombaBEstado = estado;
    }
    else if (strcmp(bombaNombre, "micro") == 0) {
      // Serial.println("  → Mapeado a: Bomba Micro (PIN_BOMBA_micro)");
      digitalWrite(PIN_BOMBA_micro, estado ? HIGH : LOW);
      bombaMicroEstado = estado;
    }
    else if (strcmp(bombaNombre, "macro") == 0) {
      // Serial.println("  → Mapeado a: Bomba FE (PIN_BOMBA_FE)");
      digitalWrite(PIN_BOMBA_FE, estado ? HIGH : LOW);
      bombaFEEstado = estado;
    }
    else if (strcmp(bombaNombre, "llenado") == 0) {
      // Serial.println("  → Mapeado a: Bomba Agua Reserva (PIN_BOMBA_AGUA_RESERVA)");
      digitalWrite(PIN_BOMBA_AGUA_RESERVA, estado ? HIGH : LOW);
      bombaAguaReservaEstado = estado;
    }
    else {
      // Serial.print("  ⚠️ Bomba desconocida: ");
      // Serial.println(bombaNombre);
    }
    
    xSemaphoreGive(mutexBombas);
  }
}

// Consultar mensajes pendientes en el backend
void consultarMensajes() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  HTTPClient http;
  String url = String(djangoServer) + String(consultarEndpoint);
  
  // Configurar cliente seguro para HTTPS
  secureClient.setInsecure();  // Ignorar certificado SSL (solo para pruebas)
  http.begin(secureClient, url);
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    
    // Parsear JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      bool hayMensaje = doc["hay_mensaje"];
      
      if (hayMensaje) {        
        const char* bombaNombre = doc["bomba"];
        bool estado = doc["estado"];
        // Serial.println("✅ Mensaje recibido del backend!");
        // Serial.print("  Bomba: ");
        // Serial.println(bombaNombre);
        // Serial.print("  Estado: ");
        // Serial.println(estado ? "ON" : "OFF");
        
        // Controlar la bomba (por ahora solo Serial.println)
        controlarBomba(bombaNombre, estado);
        
        // Enviar confirmación al backend
        enviarConfirmacionAlBackend(bombaNombre, estado);
      }
    } else {
      // Serial.print("❌ Error parseando JSON: ");
      // Serial.println(error.c_str());
    }
  } else {
    // Serial.print("❌ Error en consulta: ");
    // Serial.println(httpResponseCode);
  }
  
  http.end();
}

// Enviar confirmación al backend
void enviarConfirmacionAlBackend(const char* bombaNombre, bool estado) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  HTTPClient http;
  String url = String(djangoServer) + String(confirmacionEndpoint);
  
  // Configurar cliente seguro para HTTPS
  secureClient.setInsecure();  // Ignorar certificado SSL (solo para pruebas)
  http.begin(secureClient, url);
  http.addHeader("Content-Type", "application/json");
  
  // Crear JSON con la información de la bomba
  StaticJsonDocument<200> doc;
  doc["bomba"] = bombaNombre;
  doc["estado"] = estado;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Serial.print("📤 Enviando confirmación: ");
  // Serial.println(jsonString);
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    // Serial.print("✅ Confirmación enviada. Código: ");
    // Serial.println(httpResponseCode);
  } else {
    // Serial.print("❌ Error enviando confirmación: ");
    // Serial.println(httpResponseCode);
  }
  
  http.end();
}

// ============================================================================
// TAREA DE POLLING DEL BACKEND (FreeRTOS Task)
// ============================================================================
void tareaPollingBackend(void *parameter) {
  // Serial.println("🚀 Tarea de polling iniciada");

  // Usamos vTaskDelayUntil para que el PERIODO total sea cercano a intervaloConsulta,
  // compensando el tiempo que tarda consultarMensajes() (HTTP, parseo, etc.).
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(intervaloConsulta);

  for (;;) {
    consultarMensajes();
    vTaskDelayUntil(&lastWakeTime, periodTicks);
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
  
  // Configurar cliente seguro para HTTPS
  secureClient.setInsecure();  // Ignorar certificado SSL (solo para pruebas)
  
  // Crear semáforo mutex para proteger variables compartidas
  mutexBombas = xSemaphoreCreateMutex();
  if (mutexBombas == NULL) {
    // Serial.println("❌ Error creando semáforo mutex");
  }
  
  // Crear tarea de polling del backend (multitarea)
  // Parámetros: función, nombre, stack size, parámetros, prioridad, handle
  xTaskCreate(
    tareaPollingBackend,      // Función de la tarea
    "PollingBackend",         // Nombre de la tarea
    8192,                     // Stack size (bytes) - suficiente para HTTP
    NULL,                     // Parámetros
    2,                        // Prioridad mayor que la tarea loop() de Arduino
    NULL                      // Handle de la tarea (no necesario)
  );
  
  // Serial.println("🌐 Sistema de polling de mensajes activado en tarea separada (cada 1 segundo)");
  // Serial.println("📊 Mediciones de sensores ejecutándose en tarea principal (sin bloqueo)");
  
}

void loop() {
    // ========================================================================
    // LECTURA DE SENSORES (EJECUTÁNDOSE EN TAREA PRINCIPAL)
    // El polling del backend se ejecuta en una tarea separada (FreeRTOS)
    // ========================================================================
    for (int i = 0; i < numIter; i++) {
      sensors.requestTemperatures();
      tAguaArray[i] = sensors.getTempCByIndex(0);
      
      distanciaArray[i] = leerAngulo();
      conductividadArray[i] = analogRead(PIN_EC); // leo la conductividad
      phArray[i] = analogRead(PIN_PH); // leo el ph
    }

    StaticJsonDocument<2000> jsonDoc; 
    
    // Arrays de sensores (el backend espera estos campos)
    JsonArray tAguaJson = jsonDoc.createNestedArray("tAgua");
    JsonArray tempAmbJson = jsonDoc.createNestedArray("tempAmb");  // Vacío (no hay sensor DHT configurado)
    JsonArray humedadJson = jsonDoc.createNestedArray("humedad");  // Vacío (no hay sensor DHT configurado)
    JsonArray distanciaJson = jsonDoc.createNestedArray("distancia");
    JsonArray tdsJson = jsonDoc.createNestedArray("tds");
    JsonArray phJson = jsonDoc.createNestedArray("phs");

    for (int i = 0; i < numIter; i++) {
      tAguaJson.add(tAguaArray[i]);
      distanciaJson.add(distanciaArray[i]);
      tdsJson.add(conductividadArray[i]);
      phJson.add(phArray[i]);
      // tempAmb y humedad se dejan vacíos (arrays vacíos)
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



    // Mapear sensores de nivel a los campos que espera el backend
    jsonDoc["pinLLAHH"] = (bool)hhEstado;   // High-High
    jsonDoc["pinLLALL"] = (bool)llEstado;   // Low-Low
    jsonDoc["pinValvula"] = 0;              // No hay pin específico configurado, se envía 0
    // Campos opcionales de nivel
    jsonDoc["LLAHH_r"] = (bool)hhEstado;
    jsonDoc["LLANH_r"] = (bool)hEstado;     // High intermedio
    jsonDoc["LLANL_r"] = (bool)lEstado;     // Low intermedio
    jsonDoc["LLALL_r"] = (bool)llEstado;

    // Proteger lectura de variables compartidas de bombas
    // Usar los nombres que espera el backend
    if (xSemaphoreTake(mutexBombas, portMAX_DELAY) == pdTRUE) {
      jsonDoc["pinBomba"] = bombaPpalEstado;      // bomba_ppal -> pinBomba
      jsonDoc["bomba_A"] = bombaAEstado;          // bombaA -> bomba_A
      jsonDoc["bomba_B"] = bombaBEstado;          // bombaB -> bomba_B
      jsonDoc["bomba_micro"] = bombaMicroEstado;  // bombaMicro -> bomba_micro
      jsonDoc["bomba_Fe"] = bombaFEEstado;        // bombaFE -> bomba_Fe
      jsonDoc["bomba_agua_reserva"] = bombaAguaReservaEstado;  // Necesario para el backend
      xSemaphoreGive(mutexBombas);
    }

    //TODO LO QUE VIENE ABAJO LO HAGO SI EL PIN_TRASMITIR ESTA LOW
    // Serial.print("transmitirEstado: "); Serial.println(transmitirEstado);
    if (transmitirEstado == LOW) {

      String jsonString;
      serializeJson(jsonDoc, jsonString);
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverName);
        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.POST(jsonString);

        if (httpResponseCode > 0) {
          String response = http.getString();
          // Serial.println(httpResponseCode);
          // Serial.println(response);
        } else {
          // Serial.print("Error en la solicitud POST: ");
          // Serial.println(httpResponseCode);
        }

        http.end();
      } else {
        // Serial.println("WiFi desconectado! reconectando...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
          delay(3000);
          // Serial.println("Reconectando a WiFi...");
        }
        // Serial.println("WiFi reconectado");
      }
      delay(3000);
    }
    else{
      // Serial.println("PIN_TRASMITIR NO HABILITA LA TRANSMISION DE DATOS");
      delay(200);
    }

    // Ceder tiempo de CPU a otras tareas (por ejemplo, tarea de polling)
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
