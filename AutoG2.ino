// BLACK, WHITE, RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, ORANGE
// Note: port 8883 must be allowed thru firewall
// y = m(x) + b
// ADC = m(crane weight (grams)) + b
/*
  |
A |              x
D |        x
C |  x
  |___________________
  0    100   200
  crane weight (gr)
*/
#include <Arduino_MKRIoTCarrier.h>
#include <ADS1X15.h>
#include <Tic.h>
#include <LinearRegression.h>
#include <TimeLib.h>
#include <ArduinoBLE.h>

float version = 1.5;

MKRIoTCarrier carrier;
ADS1115 ADS(0x48);
TicI2C tic;
LinearRegression lr = LinearRegression();
File sdFile;

// Menus
bool touch[5] = { 0 };
bool menuMask[5] = { 0 };
long int touchTime[5] = { 0 };
const int MENU_CAL = 0;
const int MENU_SET = 1;
const int MENU_MANUAL = 2;
const int MENU_DEBUG = 3;
const int MENU_HOME = 4;
const String MENU_NONE = ".";
const int LED_BRIGHTNESS = 50;
const int SCREENSAVER_TIMEOUT = 1000 * 30;
// Text settings
const int TEXT_SIZE = 2;
const int MID = 120;
const int ROW = 20;
// Touch settings
const int TOUCH_THRESH = 1000;
const int TOUCH_TIMEOUT_MS = 50;
// Variables
long int refreshTime = 0;
int iLED = 0;
bool LEDdir = 1;
bool lightOn = false;
// Calibration
int calibrationWeights[3] = { 0, 100, 200 };
int calibrationADC[3] = { 0 };
const int CAL_NVS_ADDR = 0;
double linReg[2];
// Motor
bool motorActive = false;
const uint32_t MOTOR_MAX_ACCEL = 1000000;
const uint32_t MOTOR_STEPS_PER_S = 800000000;
const uint16_t currentLimitWhileMoving = 500;
const uint16_t currentLimitWhileStopped = 0;
const int FORCE_STOP_POS = 10000;
int32_t motorPos = 0;
const int RESET_COMMAND_TIMEOUT = 500;  // ms
long int motorResetTime = 0;
int curMotorState = 0;
const uint32_t MOTOR_STATE_UP = -1;
const uint32_t MOTOR_STATE_DOWN = 1;
const uint32_t MOTOR_STATE_STOP = 0;
// ADC + Closed-loop
const int neg_pin = A5;
const int pos_pin = A6;
int16_t adcVal = 0;
bool adcOnline = false;
bool doClosedLoop = false;
int closedLoopPercent = 100;
const int CLOSED_LOOP_INC = 10;  // percent
const int ADC_ERROR = 30;        // based on ADC resolution/gain
bool cleanupClosedLoop = false;
int adcGrams = 0;
// SD card
bool sdCard = false;
const String ANIMAL_FILE = "ANIMAL.TXT";
const String CALIBRATION_FILE = "CAL.TXT";
const String DATA_FILE = "DATA.TXT";
int animalWeight = 0;
int animalNumber = 0;
const int LOG_DATA_TIMEOUT = 1000;  // ms
long int logDataTime = 0;
const int DATA_INIT = 0;
const int DATA_LOOP = 1;
const int SD_BUFFER_SIZE = 60;
uint32_t dataCols[7][SD_BUFFER_SIZE] = { 0 };
int dataCount = 0;

bool killSwitch = false;
uint32_t localTime = 0;
long int initTime_millis = 0;
uint32_t initTime = 0;

void setup() {
  /* Initialize serial and wait up to 5 seconds for port to open */
  Serial.begin(9600);
  for (unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime > 5000);) {}

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

  sdFile = SD.open(ANIMAL_FILE, FILE_READ);
  if (sdFile) {
    animalNumber = sdFile.parseInt();
    animalWeight = sdFile.parseInt();
    sdFile.close();

    if (loadCalibrationValues()) {
      logData(DATA_INIT);
      sdCard = true;
    }
  }

  // tic set thru USB > Advanced: [Pin configuration] RX = kill switch, [Soft error response] De-energize
  tic.setProduct(TicProduct::T825);
  tic.reset();
  motorOff();
  tic.setMaxAccel(MOTOR_MAX_ACCEL);
  tic.setMaxDecel(MOTOR_MAX_ACCEL);
  tic.setStepMode(TicStepMode::Microstep16);  // max: Microstep32, note this affects motor speed/force stop safety
  tic.haltAndSetPosition(0);

  // BLE clock
  if (BLE.begin()) {
    clearScreen();  // default is white text
    centerString("BLE Clock?", MID, MID);
    makeButtonMenu(MENU_NONE, MENU_NONE, MENU_NONE, MENU_NONE, "Skip");
    BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);
    BLE.scanForUuid("effe", true);
    while (initTime == 0) {
      // exits via BLE callback or menu button
      BLE.poll();
      buttonsUpdate();
      if (touch[MENU_HOME]) {  // millis() - iotTime > IOT_INIT_TIMEOUT || --  || localTime > IOT_INIT_TIMEOUT
        debounceMenu();
        break;
      }
    }
  }

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

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  if (peripheral.hasLocalName()) {
    String localName = peripheral.localName();
    int strLen = localName.length() + 1;
    char charArr[strLen];
    localName.toCharArray(charArr, strLen);
    initTime = strtol(charArr, NULL, 16);
    initTime_millis = millis();
    Serial.print("Current time: ");
    Serial.println(initTime, HEX);
    Serial.println("BLE done.");
  }
}

void onKillSwitchChange() {
  motorOff();
}

// has to run everywhere: updates buttons, motor keep-alive
void buttonsUpdate() {
  // handle button touches
  for (int i = 0; i < 5; i++) {
    if (millis() - touchTime[i] > TOUCH_TIMEOUT_MS) {
      touch[i] = false;
      touchTime[i] = 0;
    }
  }
  for (int i = 0; i < 5; i++) {
    // rep with carrier.Buttons.getTouch(TOUCH0)?
    if (analogRead(i) > TOUCH_THRESH) {
      touch[i] = true;
      touchTime[i] = millis();
    }
  }
  // LED display
  carrier.leds.clear();
  if (lightOn) {
    for (int i = 0; i < 5; i++) {
      carrier.leds.setPixelColor(i, 255, 255, 255);
    }
  } else {
    for (int i = 0; i < 5; i++) {
      if (menuMask[i]) {
        if (touch[i]) {
          carrier.leds.setPixelColor(i, 0, 255, 0);
        } else {
          if (doClosedLoop) {
            carrier.leds.setPixelColor(i, iLED, 0, 0);
          } else {
            carrier.leds.setPixelColor(i, 0, 0, iLED);
          }
        }
      }
    }
  }
  carrier.leds.show();
  if (iLED >= 200) LEDdir = 0;
  if (iLED == 20) LEDdir = 1;
  if (LEDdir) iLED++;
  if (!LEDdir) iLED--;

  localTime = ((millis() - initTime_millis) / 1000) + initTime;

  if (motorActive) {
    if (millis() - motorResetTime > RESET_COMMAND_TIMEOUT) {
      tic.resetCommandTimeout();
      motorResetTime = millis();
    }
    motorPos = tic.getCurrentPosition();
    if (abs(motorPos) > FORCE_STOP_POS) {
      motorOff();
      debounceMenu();
      homeMenu();  // force reset of UI
    }
  }
  if (closeMotorLoop()) {
    if (millis() - logDataTime > LOG_DATA_TIMEOUT) {
      logDataTime = millis();
      logData(DATA_LOOP);
    }
  }
}

void readADC() {
  adcVal = ADS.readADC_Differential_0_1();
  adcGrams = int((double(adcVal) - linReg[1]) / linReg[0]);  // use int for IoT
}

bool closeMotorLoop() {
  bool retVal = false;
  if (adcOnline) {
    readADC();
  }
  if (adcOnline & motorActive & doClosedLoop & sdCard) {
    retVal = true;
    cleanupClosedLoop = true;  // turn off gracefully
    double targetCraneWeight = ((100 - closedLoopPercent) / 100.0) * animalWeight;
    double targetADC = lr.calculate(targetCraneWeight);
    if (targetADC > calibrationADC[0]) {
      if (abs(adcVal - targetADC) > ADC_ERROR) {
        if (targetADC < adcVal) {
          motorDown();
        } else {
          motorUp();
        }
      } else {
        motorStop();
      }
    } else {
      motorStop();
    }
  } else {
    motorStop();
  }
  if (!doClosedLoop & cleanupClosedLoop) {  // if closed loop recently turned off
    cleanupClosedLoop = false;
    motorStop();
  }
  return retVal;
}

void screenSaver() {
  debounceMenu();
  makeButtonMenu(MENU_NONE, MENU_NONE, MENU_NONE, MENU_NONE, "");
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
  if (sdCard) {
    centerString("Animal " + String(animalNumber) + ", " + String(animalWeight) + "g", MID, MID - ROW);
  } else {
    centerString("NO SD CARD!", MID, MID - ROW);
  }


  if (adcOnline & motorActive & doClosedLoop & sdCard) {
    carrier.display.setTextSize(3);
    centerString(String(closedLoopPercent) + "%", MID, MID + ROW);
  } else {
    String infoString = "";
    if (motorActive) {
      infoString = "Motor ON, ";
    } else {
      infoString = "Motor OFF, ";
    }
    if (doClosedLoop) {
      infoString += "Loop ON, ";
    } else {
      infoString += "Loop OFF, ";
    }
    if (adcOnline) {
      infoString += "ADC+, ";
    } else {
      infoString += "(ADC), ";
    }
    if (sdCard) {
      infoString += "SD+";
    } else {
      infoString += "(SD)";
    }
    carrier.display.setTextSize(1);
    centerString(infoString, MID, MID);
  }

  carrier.display.setTextSize(TEXT_SIZE);
  makeButtonMenu("Cal", "Unload", "Motor", "Debug", "Home");
}

void calibrateLoad() {
  doClosedLoop = false;
  debounceMenu();
  makeButtonMenu("0g", "100g", "200g", "Save", "Home");

  bool doOnce = true;
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
        if (adcOnline) readADC();
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
  if (sdCard) {
    SD.remove(CALIBRATION_FILE);
    sdFile = SD.open(CALIBRATION_FILE, FILE_WRITE);  // overwrite
    if (sdFile) {
      sdFile.println(String(calibrationADC[0]) + "," + String(calibrationADC[1]) + "," + String(calibrationADC[2]));
      sdFile.close();
      learnCalibration();
      return true;
    }
  }
  return false;
}
bool loadCalibrationValues() {
  sdFile = SD.open(CALIBRATION_FILE, FILE_READ);
  if (sdFile) {
    calibrationADC[0] = sdFile.parseInt();
    calibrationADC[1] = sdFile.parseInt();
    calibrationADC[2] = sdFile.parseInt();
    sdFile.close();
    learnCalibration();
    return true;
  }
  return false;
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

  bool doOnce = true;
  while (1) {
    buttonsUpdate();

    if (doOnce) {
      doOnce = false;
      if (doClosedLoop) {
        makeButtonMenu("(-)", "(+)", MENU_NONE, "Turn Off", "Home");
      } else {
        if (motorActive) {
          makeButtonMenu("(-)", "(+)", MENU_NONE, "Turn On", "Home");
        } else {
          makeButtonMenu("(-)", "(+)", MENU_NONE, "(need motor)", "Home");
        }
      }
      showUnloadSettings();
    }

    if (touch[0]) {
      if (closedLoopPercent - CLOSED_LOOP_INC >= 0) {
        closedLoopPercent = closedLoopPercent - CLOSED_LOOP_INC;
      }
      debounceMenu();
      doOnce = true;
    }
    if (touch[1]) {
      if (closedLoopPercent + CLOSED_LOOP_INC <= 100) {
        closedLoopPercent = closedLoopPercent + CLOSED_LOOP_INC;
      }
      debounceMenu();
      doOnce = true;
    }
    if (touch[3]) {
      if (doClosedLoop) {
        doClosedLoop = false;
      } else {
        doClosedLoop = true;
      }
      debounceMenu();
      doOnce = true;
    }

    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}
void showUnloadSettings() {
  clearDataArea();
  centerString("ground: " + String(animalWeight * (closedLoopPercent / 100.0)) + "g", MID, MID - ROW * 2);
  centerString("crane: " + String(animalWeight * ((100 - closedLoopPercent) / 100.0)) + "g", MID, MID - ROW);
  carrier.display.setTextSize(3);
  centerString(String(closedLoopPercent) + "% WB", MID, MID + ROW);
  carrier.display.setTextSize(TEXT_SIZE);
}

void manualControl() {
  doClosedLoop = false;
  debounceMenu();

  bool doOnce = true;
  while (1) {
    buttonsUpdate();
    if (doOnce) {
      doOnce = false;
      if (motorActive) {
        makeButtonMenu("Turn Off", "DOWN", "Rst", "UP", "Home");
        showMotorPosition();
      } else {
        makeButtonMenu("Turn On", MENU_NONE, MENU_NONE, MENU_NONE, "Home");
      }
    }

    if (doRefresh(1000) & motorActive) {
      showMotorPosition();
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
    if (touch[2]) {
      tic.haltAndSetPosition(0);
    }
    if (touch[MENU_HOME]) {
      homeMenu();
      return;
    }
  }
}
void showMotorPosition() {
  clearDataArea();
  centerString("Steps: " + String(motorPos), MID, MID);
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
      char buffer[30];
      if (adcOnline) readADC();
      sprintf(buffer, "t: %i:%i:%i (%is)", hour(localTime), minute(localTime), second(localTime), millis() / 1000);
      centerString(buffer, MID, MID);
      centerString("ADC: " + String(adcVal) + " (" + String(adcGrams) + "g)", MID, MID + ROW);
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

// MENU HELPERS
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
  if (curMotorState != MOTOR_STATE_STOP) {
    tic.setTargetVelocity(0);
    curMotorState = MOTOR_STATE_STOP;
  }
}
void motorUp() {
  if (curMotorState != MOTOR_STATE_UP) {
    tic.setTargetVelocity(MOTOR_STATE_UP * MOTOR_STEPS_PER_S);
    curMotorState = MOTOR_STATE_UP;
  }
}
void motorDown() {
  if (curMotorState != MOTOR_STATE_DOWN) {
    tic.setTargetVelocity(MOTOR_STATE_DOWN * MOTOR_STEPS_PER_S);
    curMotorState = MOTOR_STATE_DOWN;
  }
}
void motorOn() {
  Serial.println("Motor on");
  motorActive = true;
  tic.setCurrentLimit(currentLimitWhileMoving);
  tic.energize();
  tic.exitSafeStart();
}
void motorOff() {
  Serial.println("Motor off");
  tic.haltAndSetPosition(0);
  tic.deenergize();
  tic.setCurrentLimit(currentLimitWhileStopped);
  tic.enterSafeStart();
  motorActive = false;
  doClosedLoop = false;
  curMotorState = MOTOR_STATE_STOP;
}

// DATA HELPERS
// !! target unload
// dataType, time, data1, data2
void logData(int dataType) {
  int lum, none;
  while (!carrier.Light.colorAvailable()) {
    delay(5);
  }
  carrier.Light.readColor(none, none, none, lum);

  Serial.print("Log Count: ");
  Serial.println(dataCount);
  dataCols[0][dataCount] = dataType;
  dataCols[1][dataCount] = millis() / 1000;
  dataCols[2][dataCount] = localTime;
  if (dataType == 0) {  // init
    dataCols[3][dataCount] = animalNumber;
    dataCols[4][dataCount] = animalWeight;
    dataCols[5][dataCount] = version * 10;  // null
    dataCols[6][dataCount] = 0;
  }
  if (dataType == 1) {                       // closed loop
    dataCols[3][dataCount] = abs(adcVal);    // !!this comes in as int, dataCols is uint tho
    dataCols[4][dataCount] = abs(adcGrams);  // make abs() for now
    dataCols[5][dataCount] = closedLoopPercent;
    dataCols[6][dataCount] = lum;
  }
  dataCount++;
  if (dataCount == SD_BUFFER_SIZE) {
    Serial.println("Writing data to SD...");
    motorStop();  // pause the motor
    carrier.leds.clear();
    carrier.leds.show();
    sdFile = SD.open(DATA_FILE, FILE_WRITE);
    if (sdFile) {
      dataCount = 0;
      for (int i = 0; i < SD_BUFFER_SIZE; i++) {
        String serialString = "";
        for (int j = 0; j < sizeof(dataCols) / sizeof(dataCols[0]); j++) {
          serialString += String(dataCols[j][i]);
          if (j < sizeof(dataCols) / sizeof(dataCols[0]) - 1) {
            serialString += ",";
          }
        }
        sdFile.println(serialString);
      }
      sdFile.close();
    }
  }
}