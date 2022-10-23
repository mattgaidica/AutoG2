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
#include <ADS1X15.h>
#include <Tic.h>
#include <FlashStorage_SAMD.h>
#include <LinearRegression.h>

MKRIoTCarrier carrier;
ADS1115 ADS(0x48);
TicI2C tic;
LinearRegression lr = LinearRegression();

// Menus
const int MENU_CAL = 0;
const int MENU_SET = 1;
const int MENU_MANUAL = 2;
const int MENU_DEBUG = 3;
const int MENU_HOME = 4;
const String MENU_NONE = ".";
const String WELCOME_MSG = "Hello, Matt.";
// Text settings
const int TEXT_SIZE = 2;
const int MID = 120;
const int ROW = 20;
// Touch settings
const int TOUCH_THRESH = 1000;
const int TOUCH_TIMEOUT_MS = 50;
// Variables
bool touch[5] = { 0 };
bool menuMask[5] = { 0 };
long int touchMsElapsed[5] = { 0 };
long int refreshTime = 0;
int iLED = 0;
bool LEDdir = 1;
// Calibration
int calibrationWeights[3] = { 0, 100, 200 };
int calibrationADC[3] = { 0 };
const int CAL_NVS_ADDR = 0;
double linReg[2];

void setup() {
  Serial.begin(9600);
  carrier.noCase();
  carrier.begin();

  clearScreen();  // default is white text
  carrier.display.setTextSize(TEXT_SIZE);
  centerString("Init...", MID, MID);  // use +/-ROW
  // carrier.display.setTextSize(SMALL_TEXT);

  carrier.leds.setBrightness(50);

  ADS.begin();
  while (!ADS.isConnected()) {
    Serial.println("No conn");
  }
  ADS.setGain(16);

  loadCalibrationValues();
  homeMenu();
}

void loop() {
  buttonsUpdate();

  if (doRefresh(10000)) {
    screenSaver();
  }

  if (touch[MENU_CAL]) calibrateLoad();
  if (touch[MENU_SET]) setUnload();
  if (touch[MENU_MANUAL]) manualControl();
  if (touch[MENU_DEBUG]) debugMode();
  if (touch[MENU_HOME]) refreshTime = millis();
}

void screenSaver() {
  debounceMenu();
  makeButtonMenu("", "", "", "", "");
  carrier.display.setTextSize(1);
  carrier.display.setTextColor(ST77XX_GREEN);
  int x, y, i = 0;

  while (1) {
    buttonsUpdate();

    if (y > random(14,24)) {
      x = random(0, 24);
      y = 0;
      i++;
    }
    if (doRefresh(5)) {
      carrier.display.setCursor(x * 10, y * 10);
      if (random(0, 5) % 3 == 0) {
        carrier.display.print(char(random(0, 127)));
      } else {
        carrier.display.print(" ");
      }
      y++;
      if (i > 40) {
        i = 0;
        clearScreen();
      }
    }

    if (touch[MENU_HOME]) {
      carrier.display.setTextSize(TEXT_SIZE);
      carrier.display.setTextColor(ST77XX_WHITE);
      homeMenu();
      return;
    }
  }
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
  // LED display
  carrier.leds.clear();
  for (int i = 0; i < 5; i++) {
    if (menuMask[i]) {
      if (touch[i]) {
        carrier.leds.setPixelColor(i, 0, 255, 0);
      } else {
        carrier.leds.setPixelColor(i, iLED, 0, 0);
      }
    }
    carrier.leds.show();
  }

  if (iLED >= 200) LEDdir = 0;
  if (iLED == 20) LEDdir = 1;
  if (LEDdir) iLED++;
  if (!LEDdir) iLED--;
}

// MENUS
void homeMenu() {
  debounceMenu();
  centerString(WELCOME_MSG, MID, MID);  // use +/-ROW
  makeButtonMenu("Cal", "Set", "Ctrl", "Debug", "Home");
}

void calibrateLoad() {
  bool doOnce = true;
  debounceMenu();
  makeButtonMenu("Unloaded", "100gr", "200gr", "Save", "Home");

  while (1) {
    buttonsUpdate();
    if (doOnce) {
      doOnce = false;
      loadCalibrationValues();  // load from memory
      showCalibrationValues();
    }
    if (touch[0] | touch[1] | touch[2]) {
      if (doRefresh(250)) {
        int16_t loadVal = ADS.readADC_Differential_0_1();
        for (int i = 0; i < 3; i++) {
          if (touch[i]) calibrationADC[i] = loadVal;
        }
        showCalibrationValues();  // can change but do not save automatically
      }
    }
    if (touch[3]) {  // save to flash
      centerString("SAVED", MID, MID + ROW * 2);
      saveCalibrationValues();
      delay(500);
      showCalibrationValues();  // flash save, clear data area
    }
    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}
void learnCalibration() {
  lr.reset();
  for (int i = 0; i < 3; i++) {
    lr.learn(calibrationWeights[i], calibrationADC[i]);
  }
  lr.parameters(linReg);
}
void saveCalibrationValues() {
  for (int i = 0; i < 3; i++) {
    EEPROM.put(CAL_NVS_ADDR + i * sizeof(calibrationADC[i]), calibrationADC[i]);
  }
  learnCalibration();
}
void loadCalibrationValues() {
  for (int i = 0; i < 3; i++) {
    EEPROM.get(CAL_NVS_ADDR + i * sizeof(calibrationADC[i]), calibrationADC[i]);
  }
  learnCalibration();
}
void showCalibrationValues() {
  String calVals = "";
  for (int i = 0; i < 3; i++) {
    calVals += String(calibrationADC[i]);
    if (i < 2) {
      calVals += ", ";
    }
  }
  char buffer[30];
  sprintf(buffer, "y = %1.1fx + %1.1f", linReg[0], linReg[1]);
  clearDataArea();
  centerString(calVals, MID, MID);
  centerString(String(buffer), MID, MID + ROW);
}

void setUnload() {
  debounceMenu();
  makeButtonMenu("100%", "80%", "60%", "40%", "Home");

  while (1) {
    buttonsUpdate();
    // do stuff

    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}

void manualControl() {
  debounceMenu();
  makeButtonMenu(MENU_NONE, "UP", MENU_NONE, "DOWN", "Home");

  while (1) {
    buttonsUpdate();
    // do stuff

    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}

void debugMode() {
  debounceMenu();
  makeButtonMenu(MENU_NONE, MENU_NONE, MENU_NONE, MENU_NONE, "Home");
  while (1) {
    buttonsUpdate();

    if (doRefresh(200)) {
      clearDataArea();
      float temperature = carrier.Env.readTemperature(FAHRENHEIT);
      float humidity = carrier.Env.readHumidity();
      char buffer[30];
      sprintf(buffer, "%1.0fF, %1.0f%%", temperature, humidity);
      int16_t loadVal = ADS.readADC_Differential_0_1();

      centerString(buffer, MID, MID);  // use +/-ROW
      centerString(String(loadVal), MID, MID + ROW);
    }

    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}

bool doRefresh(int everyMs) {
  if (millis() - refreshTime > everyMs) {
    refreshTime = millis();
    return true;
  }
  return false;
}

void debounceMenu() {
  clearScreen();
  carrier.leds.clear();
  carrier.leds.show();
  delay(50);
  refreshTime = millis();
}

// SCREEN HELPERS
void clearDataArea() {
  carrier.display.fillRect(0, 80, 240, 100, ST77XX_BLACK);
}
void clearScreen() {
  carrier.display.fillScreen(ST77XX_BLACK);
}
void centerString(const String &buf, int x, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  carrier.display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);  //calc width of new string
  carrier.display.setCursor(x - w / 2, y - h / 2);
  carrier.display.print(buf);
}
void leftString(const String &buf, int x, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  carrier.display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);  //calc width of new string
  carrier.display.setCursor(x, y - h / 2);
  carrier.display.print(buf);
}
void rightString(const String &buf, int x, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  carrier.display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);  //calc width of new string
  carrier.display.setCursor(x - w, y - h / 2);
  carrier.display.print(buf);
}
void makeButtonMenu(const String &buf0, const String &buf1, const String &buf2, const String &buf3, const String &buf4) {
  carrier.display.setTextColor(ST77XX_RED);
  rightString(buf0, 220, 70);
  rightString(buf1, 210, 190);
  centerString(buf2, 120, 225);
  leftString(buf3, 30, 190);
  leftString(buf4, 20, 70);
  carrier.display.setTextColor(ST77XX_WHITE);

  // handle mask
  memset(menuMask, 1, sizeof(menuMask));
  if (buf0 == MENU_NONE) menuMask[0] = 0;
  if (buf1 == MENU_NONE) menuMask[1] = 0;
  if (buf2 == MENU_NONE) menuMask[2] = 0;
  if (buf3 == MENU_NONE) menuMask[3] = 0;
  if (buf4 == MENU_NONE) menuMask[4] = 0;
}