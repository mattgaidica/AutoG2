#include <Arduino_MKRIoTCarrier.h>
MKRIoTCarrier carrier;
#include "ADS1X15.h"

ADS1115 ADS(0x48);
int16_t val_0 = 0;
const int16_t adcError = 40;

void setup() {
  Serial.begin(9600);
  carrier.noCase();
  carrier.begin();

  ADS.begin();
  ADS.setGain(16);
}

void loop() {
  displayAnalog();
    
  // Serial.print("\tAnalog0-1: ");
  // Serial.println(val_0);
  delay(25);
}

void displayAnalog() {
  val_0++; //ADS.readADC_Differential_0_1();
  carrier.display.fillScreen(ST77XX_BLACK); //oled clear()
  carrier.display.setCursor(70, 100);
  carrier.display.print("Load Cell:  ");
  carrier.display.setTextColor(ST77XX_WHITE);
  carrier.display.print(val_0);  
}
