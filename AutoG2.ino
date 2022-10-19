// BLACK, WHITE, RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, ORANGE
/*
I2C device found at address 0x39  !
I2C device found at address 0x5C  !
I2C device found at address 0x5F  !
I2C device found at address 0x60  !
I2C device found at address 0x6A  !
I2C device found at address 0x6B  !
*/
#include <Arduino_MKRIoTCarrier.h>
#include "ADS1X15.h"
#include <Tic.h>

MKRIoTCarrier carrier;
ADS1115 ADS(0x48);
TicI2C tic;

const int TOUCH_THRESH = 600;
const int adcError = 40;
const int TOUCH_TIMEOUT_MS = 50;

int val_0 = 0;
bool touch[5] = {0};
long int touchMsElapsed[5] = {0};

void setup() {
  Serial.begin(9600);
  carrier.noCase();
  carrier.begin();

  carrier.display.fillScreen(ST77XX_BLACK);  //oled clear()
  carrier.display.setCursor(70, 100);
  carrier.display.print("Initializing...");
  carrier.display.setTextColor(ST77XX_WHITE);
  carrier.leds.setBrightness(50);

  ADS.begin();
  ADS.setGain(16);
}

void loop() {
  // displayAnalog();
  buttonsUpdate();
  for (int i = 0; i < 5; i++) {
    if (touch[i]) {
      carrier.leds.setPixelColor(i,0,255,0);
    } else {
      carrier.leds.setPixelColor(i,255,0,0);
    }
    carrier.leds.show();
    Serial.print(touch[i]);
    Serial.print("\t");
  }  
  Serial.println("");
  // Serial.print("\tAnalog0-1: ");
  // Serial.println(val_0);
  // delay(10);
}

void buttonsUpdate() {
  // clear all  
  for (int i = 0; i < 5; i++) {
    if (millis() - touchMsElapsed[i] > TOUCH_TIMEOUT_MS) {
      touch[i] = false;
      touchMsElapsed[i] = 0;
    }
    
  }
  for (int i = 0; i < 5; i++) {
    if (analogRead(i) > TOUCH_THRESH) {
      touch[i] = true;
      touchMsElapsed[i] = millis();
    }
  }
}

void displayAnalog() {
  val_0++;                                   //ADS.readADC_Differential_0_1();
  carrier.display.fillScreen(ST77XX_BLACK);  //oled clear()
  carrier.display.setCursor(70, 100);
  carrier.display.print("Load Cell:  ");
  carrier.display.setTextColor(ST77XX_WHITE);
  carrier.display.print(val_0);
}