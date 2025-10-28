#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <I2CKeyPad.h>
#include <LiquidCrystal_I2C.h>

//======================== Déclaration des adresses I2C ===================================

// Adresse I2C LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Adresse du PCF8574
#define I2C_ADDR 0x20

//========================= TFT instance ==================================================

TFT_eSPI tft = TFT_eSPI();

//========================= Déclaration des variables =====================================

// Clavier 4x3 via utilisation d'un PCF8574
I2CKeyPad clavier(I2C_ADDR); 
char keymap[19] = "123 456 789 *0# NF"; //N=NoKey F=Fail

// Variables pour la gestion du LCD
String inputString = "";
byte currentRow1 = 1;
byte currentRow2 = 1;

// Variables pour la vitesse ballon
int VITESSE = 0;                  // Vitesse de ballon souhaitée (km/h)
const int V_MAX = 100;            // Vitesse de ballon maximum (km/h)
const int MAX_RPM = 2500;         // Régime de rotation maximum du moteur (tr/min)
int rpm_input = 0;                // régime de rotation moteur pour la vitesse ballon demandée
int pwm_value = 0;                // valeur de pwm pour atteindre le régime souhaité



// Gestion du delay pour l'affichage du spin
unsigned long lastUpdate = 0;
const unsigned long interval = 50;


// Pins des boutons ON, OFF et de contrôle du spin
const int pinBoutonPlus = 14;   // GPIO14
const int pinBoutonMoins = 27;  // GPIO27
const int BUTTON_ON = 26;       // GPIO15
const int Led_ON = 12;          // GPIO12 pour led état bouton start/stop
const int RPWM_1 = 25;          // GPIO25 pour sens de rotation à droite moteur 1
const int LPWM_1 = 33;          // GPIO33 pour sens de rotation à gauche moteur 1
const int RPWM_2 = 32;          // GPIO32 pour sens de rotation à droite moteur 2
const int LPWM_2 = 19;          // GPIO19 pour sens de rotation à gauche moteur 2

// PWM Channels
#define RPWM1_CHANNEL 0
#define LPWM1_CHANNEL 1
#define RPWM2_CHANNEL 0
#define LPWM2_CHANNEL 1


// Variable pour suivre la mis en route des moteurs (via une LED)
volatile bool motorRunning = false;  // État du moteur (true = ON, false = OFF)
bool lastMotorState = false;         // Pour détecter les changements d'état
volatile bool ledState = false;
volatile unsigned long lastInterruptTime = 0;

// Variables pour le contrôle du spin et de son affichage
volatile int spinPercent = 0;
volatile bool majAffichage = false;
const int spinMax = 50;
const int spinMin = -50;

// Variables pour la gestion anti-rebond du clavier
unsigned long lastKeyPressTime = 0;
const unsigned long keyDebounceDelay = 200;  // Délai anti-rebond en millisecondes
char lastKey = 0;  // Dernière touche pressée

// Variables dynamiques - affichage sur TFT
int vitesse = 0;
int spin = 0;
float tension1 = 0.0;
float tension2 = 0.0;
int regime1 = 0;
int regime2 = 0;

//========================= Déclaration des fonctions =====================================

// Fonction affichage LCD et gestion des moteurs
void blinkLCD(int times = 2, int delayMs = 250) {
  for (int i = 0; i < times; i++) {
    lcd.noDisplay();
    delay(delayMs);
    lcd.display();
    delay(delayMs);
      }
}

void lcdPrintln(String text) {
  lcd.setCursor(0, currentRow1);
  lcd.print(text);
  //currentRow++;
  //if (currentRow > 1) currentRow = 0; // Pour un écran 2 lignes
}

void lcdPrintln_20x4(String text) {
  lcd.setCursor(0, currentRow2); // Positionne le curseur au début de la ligne
  lcd.print("                   "); // Efface la ligne (20 espaces pour écran 20x4)
  lcd.setCursor(0, currentRow2);
  lcd.print(text);
  currentRow2++;
  if (currentRow2 > 3) currentRow2 = 0; // Repart à la ligne 0 après la 4e
}

void printSpin(int percent) {
  lcd.setCursor(0, 3);
  lcd.print("Spin: ");
  lcd.print(percent);
  lcd.print(" %");
  lcd.setCursor(0, 0);
}

void engine_ss() {
  if (motorRunning != lastMotorState) {
    lcd.clear();               // Efface l'écran
    lcd.setCursor(0, 0);       // Curseur en haut à gauche
    if (motorRunning) {
      lcd.print("demarrage moteurs");
    //start_flag = !start_flag;
    }
    else {
      lcd.print("Arret moteurs");      // Message d'arrêt
    }
    delay(1500);
    lastMotorState = motorRunning;    // Mise à jour de l'état précédent
    lcd.clear();
    printSpin(spinPercent);
    lcd.print("Vitesse [0-100]: ");
    lcdPrintln("Retenue: ");
    lcd.print(VITESSE);
    lcd.print(" km/h");
    lcd.setCursor(17, 0);    
  }
}

void spin_update() {
  if (millis() - lastUpdate > interval) {
    lastUpdate = millis();
    if (majAffichage) {
      lcd.setCursor(0, 3);
      lcd.print("Spin :              "); // Efface ligne
      lcd.setCursor(7, 3);
      lcd.print(spinPercent);
      lcd.print(" %");
      lcd.setCursor(17, 0);
      majAffichage = false;
    }
  }
}

void rpm_pwm_calculation() {
  VITESSE = constrain(VITESSE, 0, V_MAX);
  rpm_input = (VITESSE * 1000 / 60) / (0.254 * 3.14159);
  pwm_value = map(rpm_input, 0, MAX_RPM, 0, 255);
}

// Fonction de commande du moteur
void commandeMoteur1() {
  if (motorRunning) {
    ledcWrite(RPWM1_CHANNEL, pwm_value);
    ledcWrite(LPWM1_CHANNEL, 0);
    Serial.println(pwm_value);
  } else {
    ledcWrite(RPWM1_CHANNEL, 0);
    ledcWrite(LPWM1_CHANNEL, 0);
  }
}

// Fonction gestion écran TFT
void drawThickRect(int x, int y, int w, int h, int thickness, uint16_t color) {
  for (int i = 0; i < thickness; i++) {
    tft.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, color);
  }
}

void updateVitesse() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(20, 80);
  tft.println("Vitesse de ballon");
  
  tft.setFreeFont(&FreeSans24pt7b);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.fillRect(30, 110, 100, 50, TFT_BLACK);
  tft.setCursor(30, 150);
  tft.printf("%3d", vitesse);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(145, 150);
  tft.println("km/h");
}

void updateSpin() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(300, 80);
  tft.println("Effet (spin)");

  tft.setFreeFont(&FreeSans24pt7b);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.fillRect(320, 110, 90, 50, TFT_BLACK);
  tft.setCursor(320, 150);
  tft.printf("%2d", spin);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(420, 150);
  tft.println("%");
}

void updateTension1() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(17, 210);
  tft.println("Tension");
  tft.setCursor(15, 237);
  tft.println("moteur 1");
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(10, 260, 100, 40, TFT_BLACK);
  tft.setCursor(12, 290);
  tft.printf("%.2f v", tension1);
}

void updateTension2() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(138, 210);
  tft.println("Tension");
  tft.setCursor(135, 237);
  tft.println("moteur 2");
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(130, 260, 100, 40, TFT_BLACK);
  tft.setCursor(135, 290);
  tft.printf("%.2f v", tension2);
}

void updateRegime1() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(259, 210);
  tft.println("Regime");
  tft.setCursor(256, 237);
  tft.println("moteur 1");
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(255, 250, 85, 35, TFT_BLACK);
  tft.setCursor(260, 279);
  tft.printf("%d", regime1);
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(270, 305);
  tft.println("tr/min");
}

void updateRegime2() {
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(380, 210);
  tft.println("Regime");
  tft.setCursor(377, 237);
  tft.println("moteur 2");
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(375, 250, 85, 35, TFT_BLACK);
  tft.setCursor(380, 279);
  tft.printf("%d", regime2);
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(390, 305);
  tft.println("tr/min");
}

//========================= Déclaration des interruptions =====================================

// Interruption de démarrage des moteurs
void IRAM_ATTR button_start() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {      // Anti-rebond : 300 ms
    motorRunning = !motorRunning;                   // Inversion de l'état moteur
    ledState = !ledState;                           // Allumage/extinction de la LED  
    lastInterruptTime = currentTime;
    digitalWrite(Led_ON, ledState);            // Mettre à jour la LED
  }           
}

// Interruption pour augmenter le spin
void IRAM_ATTR augmenterSpin() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {      // Anti-rebond : 300 ms
    if (spinPercent + 5 <= spinMax) {
      spinPercent += 5;
      majAffichage = true;
    }
    lastInterruptTime = currentTime;
  }
}

// Interruption pour diminuer le spin
void IRAM_ATTR diminuerSpin() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {      // Anti-rebond : 300 ms
    if (spinPercent - 5 >= spinMin) {
      spinPercent -= 5;
      majAffichage = true;
    }
    lastInterruptTime = currentTime;
  }
}


//===========================================================================================================
//===========================================================================================================

void setup() {
  Serial.begin(115200);

  // Initialisation de l'écran LCD
  lcd.begin (20, 4);
  lcd.init();
  lcd.backlight();

  // Initialisation de la liaison I2C
  Wire.begin(); 
  Wire.setClock(100000);

  // Configuration de la LED bouton engine start/stop
  pinMode(Led_ON, OUTPUT);
  digitalWrite(Led_ON, LOW);               // LED éteinte au démarrage

  // Configuration des boutons
  pinMode(BUTTON_ON, INPUT_PULLUP);            // Résistance de pull-up activée
  pinMode(pinBoutonPlus, INPUT_PULLUP);        // Résistance de pull-up activée
  pinMode(pinBoutonMoins, INPUT_PULLUP);       // Résistance de pull-up activée

  // Initialisation du clavier numérique
  if (clavier.begin() == false){
    Serial.println("Cannot communicate with keypad. Please check");
    while (1);
    }
  clavier.loadKeyMap(keymap);

  // Affichage de la valeur de spin
  printSpin(spinPercent);

  // Affichage de la demande de vitesse
  lcd.setCursor(0, 0);
  Serial.println("Entrez une vitesse entre 0 et 100 km/h, puis appuyez sur #");
  lcd.print("Vitesse [0-100]: ");

  // Attachement des interruptions sur front descendant (appui bouton)
  attachInterrupt(digitalPinToInterrupt(BUTTON_ON), button_start, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonPlus), augmenterSpin, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonMoins), diminuerSpin, FALLING);
  
  //============================ Setup écran TFT ============================
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Dessin des rectangles (X, Y, largeur, hauteur, épaisseur)
  drawThickRect(0, 50, 239, 130, 3, TFT_YELLOW); // Rectangle "Vitesse ballon"
  drawThickRect(241, 50, 238, 130, 3, TFT_GREEN); // Rectangle "% Spin"
  drawThickRect(0, 182, 118, 132, 2, TFT_LIGHTGREY); // Rectangle "Tension moteur 1"
  drawThickRect(120, 182, 119, 132, 2, TFT_LIGHTGREY); // Rectangle "Tension moteur 2"
  drawThickRect(241, 182, 119, 132, 2, TFT_RED); // Rectangle "Régime moteur 1"
  drawThickRect(362, 182, 118, 132, 2, TFT_RED); // Rectangle "Régime moteur 2"

  // Titre
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(155, 30);
  tft.println("Reglages actuels");

  // Affichage initial
  updateVitesse();
  updateSpin();
  updateTension1();
  updateTension2();
  updateRegime1();
  updateRegime2();

  // PWM mot1 setup
  ledcSetup(RPWM1_CHANNEL, 25000, 8); // fréquence 25kHz, 8-bit resolution
  ledcAttachPin(RPWM_1, RPWM1_CHANNEL);
  ledcSetup(LPWM1_CHANNEL, 25000, 8);
  ledcAttachPin(LPWM_1, LPWM1_CHANNEL);
}


//===========================================================================================================
//===========================================================================================================

void loop() {

  char key = clavier.getChar();

  // Gestion anti-rebond du clavier
  if (key != 0) { // Si une touche est pressée (key différente de "null")
    // Vérifier si on peut traiter cette touche (nouvelle touche OU délai écoulé)
    if ((key != lastKey) || (millis() - lastKeyPressTime > keyDebounceDelay)) {
      lastKeyPressTime = millis();
      lastKey = key;

  
      if (key >= '0' && key <= '9') {
        inputString += key;
        Serial.print(key);
        lcd.print(key);
      } 
      else if (key == '#') {
        int temp = inputString.toInt();
        if (temp >= 0 && temp <= 100) {
          VITESSE = temp;
          Serial.println();
          Serial.print("VITESSE enregistrée : ");
          Serial.println(VITESSE);
          lcd.clear();
  
          lcd.print("Vitesse [0-100]: ");
          lcdPrintln("Retenue: ");
          lcd.print(VITESSE);
          lcd.print(" km/h");
          blinkLCD(); // Clignotement pour confirmation
          printSpin(spinPercent);
          lcd.setCursor(17, 0);
          } 
        else {
          Serial.println();
          Serial.println("Valeur invalide. Entrez une vitesse entre 0 et 100.");
          lcd.clear();
          lcd.print("Vitesse invalide");
          delay(1500);
          lcd.clear();
          printSpin(spinPercent);
          lcd.print("Vitesse [0-100]: ");
          lcdPrintln("Retenue: ");
          lcd.print(VITESSE);
          lcd.print(" km/h");
          lcd.setCursor(17, 0);
          }
        inputString = "";
      }
      else if (key == '*') {
        inputString = "";
        Serial.println();
        Serial.println("Entrée réinitialisée.");
        lcd.clear();
        lcd.print("Reinitialisation");
        delay(1500);
        lcd.clear();
        printSpin(spinPercent);
        lcd.print("Vitesse [0-100]: ");
        lcdPrintln("Retenue: ");
          lcd.print(VITESSE);
          lcd.print(" km/h");
          lcd.setCursor(17, 0);
      }
    //  delay(200);
    }
  }
  engine_ss();
  spin_update();
  rpm_pwm_calculation();
  commandeMoteur1();

//============= gestion affichage TFT =================

    if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("V=")) {
      vitesse = input.substring(2).toInt();
      updateVitesse();
    } else if (input.startsWith("S=")) {
      spin = input.substring(2).toInt();
      updateSpin();
    } else if (input.startsWith("T1=")) {
      tension1 = input.substring(3).toFloat();
      updateTension1();
    } else if (input.startsWith("T2=")) {
      tension2 = input.substring(3).toFloat();
      updateTension2();
    } else if (input.startsWith("R1=")) {
      regime1 = input.substring(3).toInt();
      updateRegime1();
    } else if (input.startsWith("R2=")) {
      regime2 = input.substring(3).toInt();
      updateRegime2();
    }
  }
}

