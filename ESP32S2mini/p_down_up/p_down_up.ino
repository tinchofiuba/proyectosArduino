
//me fijo si puedo setear pull down y pull up a todos los pines
//primero seteo pull_up a todos y luego leo los estados de los pines.
//análogamente para pulldown

int pines[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 33, 34, 35, 36, 37, 38, 39, 40};
int n = sizeof(pines) / sizeof(pines[0]);

void setup() {
    Serial.begin(9600);
}

void loop() {
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
