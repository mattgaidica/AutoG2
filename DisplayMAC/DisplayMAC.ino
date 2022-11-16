#include <ArduinoIoTCloud.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  byte mac[5];
  WiFi.macAddress(mac);
  for (int i = 0; i < 5; i++) {
    Serial.print("0x");
    Serial.print(mac[i], HEX);
    if (i < 4) {
      Serial.print(", ");
    } else {
      Serial.println("");
    }
  }
  delay(5000);
}