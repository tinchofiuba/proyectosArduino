#include "cred.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Wire.h>

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
unsigned long ultimaConsulta = 0;
const unsigned long intervaloConsulta = 1000;  // Consultar cada 1 segundo

// Cliente HTTPS seguro
WiFiClientSecure secureClient;

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
  Serial.print("🔧 Comando recibido: Bomba '");
  Serial.print(bombaNombre);
  Serial.print("' -> ");
  Serial.println(estado ? "ON" : "OFF");
  
  // Por ahora solo imprimir, no controlar físicamente los pines
  // TODO: Agregar control físico de pines aquí cuando esté listo
  
  if (strcmp(bombaNombre, "recirculado") == 0) {
    Serial.println("  → Mapeado a: Bomba Principal (PIN_BOMBA_PPAL)");
    // TODO: digitalWrite(PIN_BOMBA_PPAL, estado ? HIGH : LOW);
    // TODO: bombaPpalEstado = estado;
  }
  else if (strcmp(bombaNombre, "solucionA") == 0) {
    Serial.println("  → Mapeado a: Bomba A (PIN_BOMBA_A)");
    // TODO: digitalWrite(PIN_BOMBA_A, estado ? HIGH : LOW);
    // TODO: bombaAEstado = estado;
  }
  else if (strcmp(bombaNombre, "solucionB") == 0) {
    Serial.println("  → Mapeado a: Bomba B (PIN_BOMBA_B)");
    // TODO: digitalWrite(PIN_BOMBA_B, estado ? HIGH : LOW);
    // TODO: bombaBEstado = estado;
  }
  else if (strcmp(bombaNombre, "micro") == 0) {
    Serial.println("  → Mapeado a: Bomba Micro (PIN_BOMBA_micro)");
    // TODO: digitalWrite(PIN_BOMBA_micro, estado ? HIGH : LOW);
    // TODO: bombaMicroEstado = estado;
  }
  else if (strcmp(bombaNombre, "macro") == 0) {
    Serial.println("  → Mapeado a: Bomba FE (PIN_BOMBA_FE)");
    // TODO: digitalWrite(PIN_BOMBA_FE, estado ? HIGH : LOW);
    // TODO: bombaFEEstado = estado;
  }
  else if (strcmp(bombaNombre, "llenado") == 0) {
    Serial.println("  → Mapeado a: Bomba Agua Reserva (PIN_BOMBA_AGUA_RESERVA)");
    // TODO: digitalWrite(PIN_BOMBA_AGUA_RESERVA, estado ? HIGH : LOW);
    // TODO: bombaAguaReservaEstado = estado;
  }
  else {
    Serial.print("  ⚠️ Bomba desconocida: ");
    Serial.println(bombaNombre);
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
        
        Serial.println("✅ Mensaje recibido del backend!");
        Serial.print("  Bomba: ");
        Serial.println(bombaNombre);
        Serial.print("  Estado: ");
        Serial.println(estado ? "ON" : "OFF");
        
        // Controlar la bomba (por ahora solo Serial.println)
        controlarBomba(bombaNombre, estado);
        
        // Enviar confirmación al backend
        enviarConfirmacionAlBackend(bombaNombre, estado);
      }
    } else {
      Serial.print("❌ Error parseando JSON: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("❌ Error en consulta: ");
    Serial.println(httpResponseCode);
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
  
  Serial.print("📤 Enviando confirmación: ");
  Serial.println(jsonString);
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.print("✅ Confirmación enviada. Código: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("❌ Error enviando confirmación: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
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
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");
  Serial.print("📍 IP: ");
  Serial.println(WiFi.localIP());
  
  // Configurar cliente seguro para HTTPS
  secureClient.setInsecure();  // Ignorar certificado SSL (solo para pruebas)
  Serial.println("🌐 Sistema de polling de mensajes activado (cada 1 segundo)");
  
}

void loop() {
    // ========================================================================
    // CONSULTAR MENSAJES DEL BACKEND (POLLING)
    // ========================================================================
    unsigned long tiempoActual = millis();
    if (tiempoActual - ultimaConsulta >= intervaloConsulta) {
      ultimaConsulta = tiempoActual;
      consultarMensajes();
    }

    // ========================================================================
    // LECTURA DE SENSORES (CÓDIGO ORIGINAL)
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

    Serial.println("=== SENSORES ===");
    Serial.print("Temperatura agua: "); Serial.println(tAguaArray[0]);
    Serial.print("Distancia: "); Serial.println(distanciaArray[0]);
    Serial.print("HH: "); Serial.println(hhEstado ? "encontacto" : "libre");
    Serial.print("H: "); Serial.println(hEstado ? "encontacto" : "libre");
    Serial.print("L: "); Serial.println(lEstado ? "encontacto" : "libre");
    Serial.print("LL: "); Serial.println(llEstado ? "encontacto" : "libre");
    Serial.print("TDS: "); Serial.println(conductividadArray[0]);
    Serial.print("pH: "); Serial.println(phArray[0]);
    Serial.print("Bomba ppal: "); Serial.println(bombaPpalEstado ? "ON" : "OFF");
    Serial.print("Bomba A: "); Serial.println(bombaAEstado ? "ON" : "OFF");
    Serial.print("Bomba B: "); Serial.println(bombaBEstado ? "ON" : "OFF");
    Serial.print("Bomba micro: "); Serial.println(bombaMicroEstado ? "ON" : "OFF");
    Serial.print("Bomba FE: "); Serial.println(bombaFEEstado ? "ON" : "OFF");
    Serial.print("Bomba agua reserva: "); Serial.println(bombaAguaReservaEstado ? "ON" : "OFF");
    Serial.print("Transmitir: "); Serial.println(transmitirEstado);
    Serial.println("=== FIN ===");



    jsonDoc["HH"] = hhEstado;
    jsonDoc["H"] = hEstado;
    jsonDoc["L"] = lEstado;
    jsonDoc["LL"] = llEstado;

    jsonDoc["bomba_ppal"] = bombaPpalEstado;
    jsonDoc["bombaA"] = bombaAEstado;
    jsonDoc["bombaB"] = bombaBEstado;
    jsonDoc["bombaMicro"] = bombaMicroEstado;
    jsonDoc["bombaFE"] = bombaFEEstado;
    jsonDoc["bombaAguaReserva"] = bombaAguaReservaEstado;

    //TODO LO QUE VIENE ABAJO LO HAGO SI EL PIN_TRASMITIR ESTA LOW
    Serial.print("transmitirEstado: "); Serial.println(transmitirEstado);
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
      delay(3000);
    }
    else{
      Serial.println("PIN_TRASMITIR NO HABILITA LA TRANSMISION DE DATOS");
      delay(200);
    }}
