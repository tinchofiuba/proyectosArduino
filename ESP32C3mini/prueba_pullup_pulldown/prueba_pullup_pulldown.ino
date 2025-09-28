
int pines[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21}; //solo 2 8 y 9 no se pueden setear pullDown, el rsto todo pullup
int n = sizeof(pines) / sizeof(pines[0]);

void setup() {
  Serial.begin(9600);


}

void loop() {
  // Ejemplo: configurar todos con PULLUP
  for (int i = 0; i < n; i++) {
    pinMode(pines[i], INPUT_PULLUP);
  }

  for (int i = 0; i < n; i++) {
    int val = digitalRead(pines[i]);
    Serial.printf("Pin %d -> %d\n", pines[i], val);
  }
  Serial.println("");
  Serial.printf("--------------------------------------------");
  Serial.println("");
  delay(2000);

  // Ejemplo: configurar todos con PULLUP
  for (int i = 0; i < n; i++) {
    pinMode(pines[i], INPUT_PULLDOWN);
  }

    for (int i = 0; i < n; i++) {
    int val = digitalRead(pines[i]);
    Serial.printf("Pin %d -> %d\n", pines[i], val);
  }
  Serial.println("");
  Serial.printf("--------------------------------------------");
  Serial.println("");
  delay(2000);
}