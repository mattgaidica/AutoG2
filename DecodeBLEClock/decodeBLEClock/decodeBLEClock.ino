/*
  Scan Callback

  This example scans for Bluetooth® Low Energy peripherals and prints out their advertising details:
  address, local name, advertised service UUIDs. Unlike the Scan example, it uses
  the callback style APIs and disables filtering so the peripheral discovery is
  reported for every single advertisement it makes.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
uint32_t localTime = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  Serial.println("Searching for clock...");
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);
  BLE.scanForUuid("effe", true);
}

void loop() {
  if (localTime == 0) {
    BLE.poll();
  }
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  if (peripheral.hasLocalName()) {
    String localName = peripheral.localName();
    int strLen = localName.length() + 1;
    char charArr[strLen];
    localName.toCharArray(charArr, strLen);
    localTime = strtol(charArr, NULL, 16);

    Serial.print("Local Name: ");
    Serial.println(localName);
    Serial.print("Current time: ");
    Serial.println(localTime, HEX);

    Serial.println("BLE done.");
    BLE.end();
  }
}
