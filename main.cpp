#include <Arduino.h>
#include <Wire.h>
#include <I2CKeyPad.h>
#include <LiquidCrystal_I2C.h>
#include <TFT_eSPI.h>

// ===================== Configuration =====================
// I2C LCD address
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define I2C_ADDR 0x20

// NOTE: Pour désactiver un des écrans selon le montage matériel:
//  - Commenter les appels/initialisations LCD (lcd.begin, lcd.init, lcd.backlight, printSpinLCD, blinkLCD)
//  - Ou commenter les appels TFT (tft.init, drawThickRect, updateXXXTFT) si vous n'avez que le LCD.
// Vous pouvez aussi utiliser des macros de compilation:
//    #define USE_LCD 1
//    #define USE_TFT 1
// et entourer le code par #if USE_LCD / #if USE_TFT.
// Ici le code est écrit pour fonctionner simultanément.

// Keypad 4x3 via PCF8574
I2CKeyPad clavier(I2C_ADDR);
char keymap[19] = "123 456 789 *0# NF"; // N=NoKey F=Fail

// TFT instance
TFT_eSPI tft = TFT_eSPI();

// ===================== Variables =====================
String inputString = "";
int VITESSE = 0;                   // 0-100 km/h
volatile int spinPercent = 0;      // -50 to +50
const int spinMax = 50;
const int spinMin = -50;

// Dynamic values for TFT (placeholders for future sensors)
float tension1 = 0.0;
float tension2 = 0.0;
int regime1 = 0;
int regime2 = 0;

// State & timing
byte currentRow1 = 1;
byte currentRow2 = 1;
unsigned long lastUpdate = 0;
const unsigned long interval = 50;
volatile bool majAffichage = false; // spin update flag for LCD

// Buttons
const int pinBoutonPlus  = 14; // spin +
const int pinBoutonMoins = 27; // spin -
const int BUTTON_ON  = 15;     // start motors
const int BUTTON_OFF = 32;     // stop motors (moved from 26 to avoid PWM conflict)

// Motor PWM pins (adjusted to avoid button conflicts)
const int RPWM = 25;  // forward
const int LPWM = 33;  // reverse
int pwmValue = 0;     // 0-255 mapped from VITESSE

// Engine state
volatile bool ledState = false;      // LED mirrors engine state
volatile bool start_flag = false;
volatile bool stop_flag  = false;
volatile unsigned long lastInterruptTime = 0; // debounce shared

// ===================== Forward declarations =====================
void updateMotorSpeed();
void updateAllTFT();
void updateVitesseTFT();
void updateSpinTFT();
void updateTension1TFT();
void updateTension2TFT();
void updateRegime1TFT();
void updateRegime2TFT();

// ===================== LCD helpers =====================
void blinkLCD(int times = 2, int delayMs = 250) {
  for (int i = 0; i < times; i++) {
    lcd.noDisplay(); delay(delayMs);
    lcd.display();   delay(delayMs);
  }
}

void lcdPrintln(String text) {
  lcd.setCursor(0, currentRow1);
  lcd.print(text);
}

void printSpinLCD(int percent) {
  lcd.setCursor(0, 3);
  lcd.print("Spin: ");
  lcd.print(percent);
  lcd.print(" %");
  lcd.setCursor(0, 0);
}

// ===================== TFT drawing helpers =====================
void drawThickRect(int x, int y, int w, int h, int thickness, uint16_t color) {
  for (int i = 0; i < thickness; i++) {
    tft.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, color);
  }
}

void updateVitesseTFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(20, 80); tft.println("Vitesse de ballon");
  tft.setFreeFont(&FreeSans24pt7b);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.fillRect(30, 110, 100, 50, TFT_BLACK);
  tft.setCursor(30, 150); tft.printf("%3d", VITESSE);
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(145, 150); tft.println("km/h");
}

void updateSpinTFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(300, 80); tft.println("Effet (spin)");
  tft.setFreeFont(&FreeSans24pt7b);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.fillRect(320, 110, 90, 50, TFT_BLACK);
  tft.setCursor(320, 150); tft.printf("%2d", spinPercent);
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(420, 150); tft.println("%");
}

void updateTension1TFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(17, 210); tft.println("Tension");
  tft.setCursor(15, 237); tft.println("moteur 1");
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(10, 260, 100, 40, TFT_BLACK);
  tft.setCursor(12, 290); tft.printf("%.2f v", tension1);
}

void updateTension2TFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(138, 210); tft.println("Tension");
  tft.setCursor(135, 237); tft.println("moteur 2");
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(130, 260, 100, 40, TFT_BLACK);
  tft.setCursor(135, 290); tft.printf("%.2f v", tension2);
}

void updateRegime1TFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(259, 210); tft.println("Regime");
  tft.setCursor(256, 237); tft.println("moteur 1");
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(255, 250, 85, 35, TFT_BLACK);
  tft.setCursor(260, 279); tft.printf("%d", regime1);
  tft.setFreeFont(&FreeSans12pt7b); tft.setCursor(270, 305); tft.println("tr/min");
}

void updateRegime2TFT() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(380, 210); tft.println("Regime");
  tft.setCursor(377, 237); tft.println("moteur 2");
  tft.setFreeFont(&FreeSans18pt7b); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(375, 250, 85, 35, TFT_BLACK);
  tft.setCursor(380, 279); tft.printf("%d", regime2);
  tft.setFreeFont(&FreeSans12pt7b); tft.setCursor(390, 305); tft.println("tr/min");
}

void updateAllTFT() {
  updateVitesseTFT();
  updateSpinTFT();
  updateTension1TFT();
  updateTension2TFT();
  updateRegime1TFT();
  updateRegime2TFT();
}

// ===================== Motor control =====================
void updateMotorSpeed() {
  pwmValue = map(VITESSE, 0, 100, 0, 255);
  if (ledState) {
    ledcWrite(0, pwmValue); // forward
    ledcWrite(1, 0);        // reverse off
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

// ===================== Interrupt callbacks =====================
void IRAM_ATTR button_start() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {
    ledState = true;
    lastInterruptTime = currentTime;
    digitalWrite(LED_BUILTIN, ledState);
    start_flag = true;
  }
}

void IRAM_ATTR button_stop() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {
    ledState = false;
    lastInterruptTime = currentTime;
    digitalWrite(LED_BUILTIN, ledState);
    stop_flag = true;
  }
}

void IRAM_ATTR augmenterSpin() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 150) { // faster debounce for spin
    if (spinPercent + 5 <= spinMax) {
      spinPercent += 5;
      majAffichage = true; // update LCD spin
      updateSpinTFT();      // immediate TFT update
    }
    lastInterruptTime = currentTime;
  }
}

void IRAM_ATTR diminuerSpin() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 150) {
    if (spinPercent - 5 >= spinMin) {
      spinPercent -= 5;
      majAffichage = true;
      updateSpinTFT();
    }
    lastInterruptTime = currentTime;
  }
}

// ===================== Engine start/stop handlers =====================
void start_engine() {
  if (start_flag) {
    lcd.clear();
    lcd.print("Demarrage moteurs");
    updateMotorSpeed();
    delay(1000);
    lcd.clear();
    printSpinLCD(spinPercent);
    lcd.print("Vitesse [0-100]: ");
    lcdPrintln("Retenue: ");
    lcd.print(VITESSE); lcd.print(" km/h");
    lcd.setCursor(17, 0);
    updateVitesseTFT(); // reflect current speed
    start_flag = false;
  }
}

void stop_engine() {
  if (stop_flag) {
    lcd.clear();
    lcd.print("Arret moteurs");
    updateMotorSpeed(); // sets PWM to 0
    delay(1000);
    lcd.clear();
    printSpinLCD(spinPercent);
    lcd.print("Vitesse [0-100]: ");
    lcdPrintln("Retenue: ");
    lcd.print(VITESSE); lcd.print(" km/h");
    lcd.setCursor(17, 0);
    updateVitesseTFT();
    stop_flag = false;
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  // LCD init
  lcd.begin(20, 4);
  lcd.init();
  lcd.backlight();

  Wire.begin();
  Wire.setClock(400000);

  // TFT init
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Frames
  drawThickRect(0,   50, 239, 130, 3, TFT_YELLOW);
  drawThickRect(241, 50, 238, 130, 3, TFT_GREEN);
  drawThickRect(0,  182, 118, 132, 2, TFT_LIGHTGREY);
  drawThickRect(120,182, 119, 132, 2, TFT_LIGHTGREY);
  drawThickRect(241,182, 119, 132, 2, TFT_RED);
  drawThickRect(362,182, 118, 132, 2, TFT_RED);
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(155, 30); tft.println("Reglages actuels");
  updateAllTFT();

  // Motor PWM setup
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  ledcSetup(0, 20000, 8);
  ledcSetup(1, 20000, 8);
  ledcAttachPin(RPWM, 0);
  ledcAttachPin(LPWM, 1);

  // Engine status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Buttons
  pinMode(BUTTON_ON, INPUT_PULLUP);
  pinMode(BUTTON_OFF, INPUT_PULLUP);
  pinMode(pinBoutonPlus, INPUT_PULLUP);
  pinMode(pinBoutonMoins, INPUT_PULLUP);

  // Keypad
  if (!clavier.begin()) {
    Serial.println("Cannot communicate with keypad. Please check wiring.");
    while (true) { delay(100); }
  }
  clavier.loadKeyMap(keymap);

  // Initial LCD content
  printSpinLCD(spinPercent);
  lcd.setCursor(0, 0);
  Serial.println("Entrez une vitesse entre 0 et 100 km/h, puis appuyez sur #");
  lcd.print("Vitesse [0-100]: ");

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_ON),  button_start,   FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_OFF), button_stop,    FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonPlus),  augmenterSpin, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonMoins), diminuerSpin,  FALLING);
}

// ===================== Loop =====================
void loop() {
  char key = clavier.getChar();
  if (key != 0) {
    if (key >= '0' && key <= '9') {
      inputString += key;
      Serial.print(key);
      lcd.print(key);
    } else if (key == '#') {
      int temp = inputString.toInt();
      if (temp >= 0 && temp <= 100) {
        VITESSE = temp;
        Serial.println();
        Serial.print("VITESSE enregistrée : "); Serial.println(VITESSE);
        updateMotorSpeed();
        lcd.clear();
        lcd.print("Vitesse [0-100]: ");
        lcdPrintln("Retenue: ");
        lcd.print(VITESSE); lcd.print(" km/h");
        blinkLCD();
        printSpinLCD(spinPercent);
        lcd.setCursor(17, 0);
        updateVitesseTFT();
      } else {
        Serial.println();
        Serial.println("Valeur invalide. Entrez une vitesse entre 0 et 100.");
        lcd.clear(); lcd.print("Vitesse invalide");
        delay(1200);
        lcd.clear();
        printSpinLCD(spinPercent);
        lcd.print("Vitesse [0-100]: ");
        lcdPrintln("Retenue: ");
        lcd.print(VITESSE); lcd.print(" km/h");
        lcd.setCursor(17, 0);
      }
      inputString = "";
    } else if (key == '*') {
      inputString = "";
      Serial.println(); Serial.println("Entrée réinitialisée.");
      lcd.clear(); lcd.print("Reinitialisation");
      delay(1000);
      lcd.clear();
      printSpinLCD(spinPercent);
      lcd.print("Vitesse [0-100]: ");
      lcdPrintln("Retenue: ");
      lcd.print(VITESSE); lcd.print(" km/h");
      lcd.setCursor(17, 0);
    }
    delay(180);
  }

  start_engine();
  stop_engine();

  // Periodic LCD spin refresh using flag
  if (millis() - lastUpdate > interval) {
    lastUpdate = millis();
    if (majAffichage) {
      lcd.setCursor(0, 3);
      lcd.print("Spin :              ");
      lcd.setCursor(7, 3);
      lcd.print(spinPercent); lcd.print(" %");
      lcd.setCursor(17, 0);
      majAffichage = false;
    }
  }
}
