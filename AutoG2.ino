// BLACK, WHITE, RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, ORANGE
#include <Arduino_MKRIoTCarrier.h>
#include <ADS1X15.h>
#include <Tic.h>
#include <LinearRegression.h>
#include <Statistic.h>

MKRIoTCarrier carrier;
ADS1115 ADS(0x48);
TicI2C tic;
LinearRegression lr = LinearRegression();
Statistic stats;
File animalFile;
File calibrationFile;

// Menus
const int MENU_CAL = 0;
const int MENU_SET = 1;
const int MENU_MANUAL = 2;
const int MENU_DEBUG = 3;
const int MENU_HOME = 4;
const String MENU_NONE = "-";
const String WELCOME_MSG = "Hello, Matt";
const int LED_BRIGHTNESS = 50;
const int SCREENSAVER_TIMEOUT = 1000 * 10;
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
bool lightOn = false;
int16_t adcVal = 0;
bool adcOnline = false;
bool sdCard = false;
// Calibration
int calibrationWeights[3] = { 0, 100, 200 };
int calibrationADC[3] = { 0 };
const int CAL_NVS_ADDR = 0;
double linReg[2];
// motor
bool motorActive = false;
const int MOTOR_STEPS_PER_S = 5000000;
const uint16_t currentLimitWhileMoving = 500;
const uint16_t currentLimitWhileStopped = 50;
// closed loop
const int CLOSE_LOOP_TIMEOUT = 100;
long int closeLoopTime = 0;
const int LOOP_SAMPLES = 5;
int16_t adcVals[10] = { 0 };
int adcCount = 0;
// SD card
const String ANIMAL_FILE = "ANIMAL.TXT";
int animalWeight = 0;
int animalNumber = 0;
const String CALIBRATION_FILE = "CAL.TXT";

void setup() {
  Serial.begin(9600);
  carrier.begin();
  carrier.display.setRotation(0);

  clearScreen();  // default is white text
  carrier.display.setTextSize(TEXT_SIZE);
  carrier.display.setCursor(120, 120);
  carrier.display.print("+");               // mark center
  centerString("Init...", MID, MID - ROW);  // use +/-ROW

  // init code
  carrier.leds.setBrightness(LED_BRIGHTNESS);

  ADS.begin();
  if (ADS.isConnected()) {
    adcOnline = true;
    ADS.setGain(16);
  }

  animalFile = SD.open(ANIMAL_FILE, FILE_READ);
  if (animalFile) {
    sdCard = true;
    animalNumber = animalFile.parseInt();
    animalWeight = animalFile.parseInt();

    animalFile.close();
    // seems to have some issues, creating files ahead of time    
    if (!SD.exists(CALIBRATION_FILE)) {
      calibrationFile = SD.open(CALIBRATION_FILE, FILE_WRITE);
      if (calibrationFile) {
        calibrationFile.println("0,0,0");  // dummy data
        calibrationFile.close();
      }
    }
  }
  loadCalibrationValues();
  tic.setProduct(TicProduct::T825);
  homeMenu();
}

void loop() {
  buttonsUpdate();  // !!has to run everywhere

  if (doRefresh(SCREENSAVER_TIMEOUT)) {
    screenSaver();
  }

  if (touch[MENU_CAL]) calibrateLoad();
  if (touch[MENU_SET]) setUnload();
  if (touch[MENU_MANUAL]) manualControl();
  if (touch[MENU_DEBUG]) debugMode();
  if (touch[MENU_HOME]) refreshTime = millis();
}

// has to run everywhere: updates buttons, motor keep-alive
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
  if (lightOn) {
    carrier.leds.clear();
    for (int i = 0; i < 5; i++) {
      carrier.leds.setPixelColor(i, 255, 255, 255);
    }
    carrier.leds.setBrightness(255);
    carrier.leds.show();
  } else {
    carrier.leds.clear();
    for (int i = 0; i < 5; i++) {
      if (menuMask[i]) {
        if (touch[i]) {
          carrier.leds.setPixelColor(i, 0, 255, 0);
        } else {
          carrier.leds.setPixelColor(i, iLED, 0, 0);
        }
      }
      carrier.leds.setBrightness(LED_BRIGHTNESS);
      carrier.leds.show();
    }
  }

  if (motorActive) {
    tic.resetCommandTimeout();
  }

  closeMotorLoop();

  if (iLED >= 200) LEDdir = 0;
  if (iLED == 20) LEDdir = 1;
  if (LEDdir) iLED++;
  if (!LEDdir) iLED--;
}

void closeMotorLoop() {
  if (millis() > closeLoopTime + CLOSE_LOOP_TIMEOUT) {
    closeLoopTime = millis();
    if (adcOnline & motorActive) {
      adcVal = ADS.readADC_Differential_0_1();

      if (adcCount < LOOP_SAMPLES) {
        // adcVals[adcCount] = -adcVal;
        stats.add(adcVal / 1.0);
        adcCount++;
      } else {
        // average value, set motor
        adcCount = 0;
        Serial.println(stats.average(), 4);
        stats.clear();
      }
    }
  }
}

void screenSaver() {
  debounceMenu();
  makeButtonMenu("", "", "", "", "");
  carrier.display.setTextSize(1);
  carrier.display.setTextColor(ST77XX_GREEN);
  int x, i = 0;
  int y = 100;

  while (1) {
    buttonsUpdate();

    if (y > random(14, 36)) {
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

// MENUS
void homeMenu() {
  debounceMenu();
  // centerString(WELCOME_MSG, MID, MID - ROW * 2);  // use +/-ROW
  centerString("Animal " + String(animalNumber) + ", " + String(animalWeight) + "gr", MID, MID - ROW);
  String infoString = "";
  if (motorActive) {
    infoString = "M[+],";
  } else {
    infoString = "M[-],";
  }
  if (adcOnline) {
    infoString += "ADC[+],";
  } else {
    infoString += "ADC[-],";
  }
  if (sdCard) {
    infoString += "SD[+]";
  } else {
    infoString += "SD[-]";
  }
  centerString(infoString, MID, MID);
  makeButtonMenu("Cal", "Set", "Motor", "Debug", "Home");
}

void calibrateLoad() {
  bool doOnce = true;
  debounceMenu();
  makeButtonMenu("0gr", "100gr", "200gr", "Save", "Home");

  while (1) {
    buttonsUpdate();
    if (doOnce) {
      doOnce = false;
      loadCalibrationValues();  // load from memory
      showCalibrationValues();
    }
    if (touch[0] | touch[1] | touch[2]) {
      if (doRefresh(250)) {
        adcVal = 0;
        if (adcOnline) adcVal = ADS.readADC_Differential_0_1();
        for (int i = 0; i < 3; i++) {
          if (touch[i]) calibrationADC[i] = adcVal;
        }
        showCalibrationValues();  // can change but do not save automatically
      }
    }
    if (touch[3]) {  // save to flash
      if (saveCalibrationValues()) {
        centerString("SAVED", MID, MID + ROW);
      } else {
        centerString("FAILED SAVE", MID, MID + ROW);
      }
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
bool saveCalibrationValues() {
  bool retVal = false;
  if (sdCard) {
    SD.remove(CALIBRATION_FILE);
    calibrationFile = SD.open(CALIBRATION_FILE, FILE_WRITE); // overwrite
    if (calibrationFile) {
      calibrationFile.println(String(calibrationADC[0]) + "," + String(calibrationADC[1]) + "," + String(calibrationADC[2]));
      calibrationFile.close();
      retVal = true;
    }
  }
  learnCalibration();
  return retVal;
}
void loadCalibrationValues() {
  if (sdCard) {
    calibrationFile = SD.open(CALIBRATION_FILE, FILE_READ);
    if (calibrationFile) {
      calibrationADC[0] = calibrationFile.parseInt();
      calibrationADC[1] = calibrationFile.parseInt();
      calibrationADC[2] = calibrationFile.parseInt();
      calibrationFile.close();
    }
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
  centerString(calVals, MID, MID - ROW);
  centerString(String(buffer), MID, MID);
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

  bool doOnce = true;
  while (1) {
    buttonsUpdate();
    if (doOnce) {
      doOnce = false;
      if (motorActive) {
        makeButtonMenu("Turn Off", "DOWN", MENU_NONE, "UP", "Home");
      } else {
        makeButtonMenu("Turn On", MENU_NONE, MENU_NONE, MENU_NONE, "Home");
      }
    }

    if (touch[0]) {
      if (motorActive) {
        motorOff();
      } else {
        motorOn();
      }
      debounceMenu();
      doOnce = true;
    }
    if (motorActive) {
      if (touch[1]) {
        motorDown();
      } else if (touch[3]) {
        motorUp();
      } else {
        motorStop();  // always send 0 if not up/down
      }
    }
    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}

void debugMode() {
  debounceMenu();
  makeButtonMenu("Light", MENU_NONE, MENU_NONE, MENU_NONE, "Home");

  bool doOnce = true;
  while (1) {
    buttonsUpdate();

    if (doRefresh(2000) | doOnce) {
      doOnce = false;
      clearDataArea();
      float temperature = carrier.Env.readTemperature(FAHRENHEIT);
      float humidity = carrier.Env.readHumidity();
      char buffer[30];
      if (adcOnline) adcVal = ADS.readADC_Differential_0_1();
      sprintf(buffer, "t: 0x%X, %is", millis() / 1000, millis() / 1000);
      centerString(buffer, MID, MID - ROW);
      sprintf(buffer, "Wx: %1.0fF, %1.0f%%", temperature, humidity);
      centerString(buffer, MID, MID);  // use +/-ROW
      centerString("load: " + String(adcVal), MID, MID + ROW);
    }

    if (touch[0]) {
      if (lightOn) {
        lightOn = false;
      } else {
        lightOn = true;
      }
      delay(200);  // debounce inline
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
  leftString(buf0, 30, 190);
  leftString(buf1, 30, 50);
  centerString(buf2, 120, 15);
  rightString(buf3, 210, 50);
  rightString(buf4, 210, 190);
  carrier.display.setTextColor(ST77XX_WHITE);

  // handle mask
  memset(menuMask, 1, sizeof(menuMask));
  if (buf0 == MENU_NONE) menuMask[0] = 0;
  if (buf1 == MENU_NONE) menuMask[1] = 0;
  if (buf2 == MENU_NONE) menuMask[2] = 0;
  if (buf3 == MENU_NONE) menuMask[3] = 0;
  if (buf4 == MENU_NONE) menuMask[4] = 0;
}

// MOTOR HELPERS
void motorStop() {
  tic.setTargetVelocity(0);
}
void motorUp() {
  tic.setTargetVelocity(-MOTOR_STEPS_PER_S);
}
void motorDown() {
  tic.setTargetVelocity(MOTOR_STEPS_PER_S);
}
void motorOn() {
  motorActive = true;
  tic.setCurrentLimit(currentLimitWhileMoving);
  tic.exitSafeStart();
}
void motorOff() {
  tic.setCurrentLimit(currentLimitWhileStopped);
  tic.enterSafeStart();  // !!check if working
  motorActive = false;
}