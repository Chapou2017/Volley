#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
// #include <I2CKeyPad.h>  // Bibliothèque standard (câblage croisé)
#include "I2CKeyPad_Custom.h"  // Bibliothèque modifiée (câblage direct Adafruit PID 3845)
#include <LiquidCrystal_I2C.h>
#include <TM1637Display.h>
#include <PCF8574.h>

//======================== Déclaration des adresses I2C ===================================

// Adresse I2C LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Adresse du PCF8574 pour clavier
#define I2C_ADDR 0x20

// Adresse du PCF8574 pour GPIO extender (TM1637)
#define PCF8574_ADDR 0x21  // À ajuster selon votre module (0x20, 0x21, 0x22, etc.)
PCF8574 pcf8574(PCF8574_ADDR);

// Pins des afficheurs TM1637 pour affichage RPM (via PCF8574)
// PCF8574 P0-P3 pour TM1637 moteur 1, P4-P5 pour TM1637 moteur 2
const int PCF_TM1637_CLK_1 = 0;    // P0 du PCF8574 pour CLK afficheur RPM moteur 1
const int PCF_TM1637_DIO_1 = 1;    // P1 du PCF8574 pour DIO afficheur RPM moteur 1
const int PCF_TM1637_CLK_2 = 2;    // P2 du PCF8574 pour CLK afficheur RPM moteur 2
const int PCF_TM1637_DIO_2 = 3;    // P3 du PCF8574 pour DIO afficheur RPM moteur 2

//========================= TFT instance ==================================================

TFT_eSPI tft = TFT_eSPI();

//======================== Classe TM1637 via PCF8574 ======================================

// Classe wrapper pour utiliser TM1637 via PCF8574
class TM1637_PCF {
private:
  PCF8574* pcf;
  uint8_t clkPin;
  uint8_t dioPin;
  uint8_t brightness;
  
  void bitDelay() { delayMicroseconds(100); }
  
  void writePin(uint8_t pin, uint8_t value) {
    pcf->write(pin, value);
  }
  
  void start() {
    writePin(dioPin, HIGH);
    writePin(clkPin, HIGH);
    bitDelay();
    writePin(dioPin, LOW);
  }
  
  void stop() {
    writePin(clkPin, LOW);
    bitDelay();
    writePin(dioPin, LOW);
    bitDelay();
    writePin(clkPin, HIGH);
    bitDelay();
    writePin(dioPin, HIGH);
  }
  
  void writeByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
      writePin(clkPin, LOW);
      bitDelay();
      writePin(dioPin, (data & 0x01) ? HIGH : LOW);
      bitDelay();
      writePin(clkPin, HIGH);
      bitDelay();
      data >>= 1;
    }
    // ACK
    writePin(clkPin, LOW);
    bitDelay();
    writePin(clkPin, HIGH);
    bitDelay();
  }

public:
  TM1637_PCF(PCF8574* pcfPtr, uint8_t clk, uint8_t dio) : pcf(pcfPtr), clkPin(clk), dioPin(dio), brightness(7) {}
  
  void begin() {
    // Pas besoin de pinMode avec PCF8574 de Rob Tillaart
    // Les pins sont en sortie par défaut
    writePin(clkPin, HIGH);
    writePin(dioPin, HIGH);
  }
  
  void setBrightness(uint8_t b) {
    brightness = b & 0x0f;
  }
  
  void showNumberDec(int num, bool leadingZero = false) {
    const uint8_t digitToSegment[] = {
      0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F  // 0-9
    };
    
    uint8_t digits[4] = {0, 0, 0, 0};
    bool negative = num < 0;
    if (negative) num = -num;
    
    digits[3] = num % 10;
    digits[2] = (num / 10) % 10;
    digits[1] = (num / 100) % 10;
    digits[0] = (num / 1000) % 10;
    
    // Commande d'écriture de données
    start();
    writeByte(0x40);  // Mode écriture automatique
    stop();
    
    // Écrire les 4 chiffres
    start();
    writeByte(0xC0);  // Adresse de départ
    for (int i = 0; i < 4; i++) {
      if (!leadingZero && i < 3 && digits[i] == 0 && num < pow(10, 3-i)) {
        writeByte(0x00);  // Éteindre les zéros de tête
      } else {
        writeByte(digitToSegment[digits[i]]);
      }
    }
    stop();
    
    // Contrôle de luminosité
    start();
    writeByte(0x88 | brightness);
    stop();
  }
};

TM1637_PCF displayRPM1(&pcf8574, PCF_TM1637_CLK_1, PCF_TM1637_DIO_1);
TM1637_PCF displayRPM2(&pcf8574, PCF_TM1637_CLK_2, PCF_TM1637_DIO_2);

//========================= Déclaration des variables =====================================

// Clavier 4x3 via utilisation d'un PCF8574
I2CKeyPad_Custom clavier(I2C_ADDR);  // Version personnalisée pour câblage direct

// KEYMAP - Câblage direct Adafruit PID 3845 (Broches 1-7 → P0-P6)
// Utilise la bibliothèque I2CKeyPad_Custom.h qui gère le pinout spécifique
char keymap[19] = "123456789*0#    NF";

// Note: La bibliothèque I2CKeyPad_Custom.h mappe automatiquement:
// Adafruit Br1(C2)→P0, Br2(L1)→P1, Br3(C1)→P2, Br4(L4)→P3, Br5(C3)→P4, Br6(L3)→P5, Br7(L2)→P6

// Variables pour la gestion du LCD
String inputString = "";
byte currentRow1 = 1;
byte currentRow2 = 1;

// Variables pour la vitesse ballon
int VITESSE = 0;                  // Vitesse de ballon souhaitée (km/h)
const int V_MAX = 100;            // Vitesse de ballon maximum (km/h)
const int MAX_RPM = 2500;         // Régime de rotation maximum du moteur (tr/min)
int rpm_input = 0;                // régime de rotation moteur pour la vitesse ballon demandée
int pwm_value_1 = 0;              // valeur de pwm moteur 1 (avec spin appliqué)
int pwm_value_2 = 0;              // valeur de pwm moteur 2 (avec spin appliqué)

// Gestion du delay pour l'affichage du spin
unsigned long lastUpdate = 0;
const unsigned long interval = 50;

// Pins des boutons ON, OFF et de contrôle du spin
const int pinBoutonPlus = 14;   // GPIO14
const int pinBoutonMoins = 27;  // GPIO27
const int BUTTON_ON = 26;       // GPIO26
const int Led_ON = 12;          // GPIO12 pour led état bouton start/stop
const int RPWM_1 = 16;          // GPIO16 pour sens de rotation à droite moteur 1
const int LPWM_1 = 33;          // GPIO33 pour sens de rotation à gauche moteur 1
const int RPWM_2 = 32;          // GPIO32 pour sens de rotation à droite moteur 2
const int LPWM_2 = 19;          // GPIO19 pour sens de rotation à gauche moteur 2

// Pins des capteurs Reed Switch (RS 268-4855) pour mesure de vitesse
const int REED_MOTEUR_1 = 5;    // GPIO5 pour capteur Reed Switch moteur 1
const int REED_MOTEUR_2 = 13;   // GPIO13 pour capteur Reed Switch moteur 2
const int PULSES_PER_REV = 4;   // Nombre de cibles magnétiques par tour (4 aimants sur le disque)

// PWM Channels
#define RPWM1_CHANNEL 0
#define LPWM1_CHANNEL 1
#define RPWM2_CHANNEL 2
#define LPWM2_CHANNEL 3


// Variable pour suivre la mis en route des moteurs (via une LED)
volatile bool motorRunning = false;  // État du moteur (true = ON, false = OFF)
bool lastMotorState = false;         // Pour détecter les changements d'état
volatile bool ledState = false;
volatile unsigned long lastInterruptTime = 0;

// Variables pour la mesure de vitesse (Reed Switch RS 268-4855)
volatile unsigned long pulseCount1 = 0;    // Compteur d'impulsions moteur 1
volatile unsigned long pulseCount2 = 0;    // Compteur d'impulsions moteur 2
volatile unsigned long lastPulseTime1 = 0; // Temps de la dernière impulsion moteur 1
volatile unsigned long lastPulseTime2 = 0; // Temps de la dernière impulsion moteur 2
unsigned long lastRPMCalc = 0;             // Dernier calcul de RPM
const unsigned long RPM_CALC_INTERVAL = 1000; // Intervalle de calcul RPM en ms. En augmentant, on améliore la précision mais on réduit la réactivité
const unsigned long DEBOUNCE_TIME = 10;     // Anti-rebond 10ms (Reed Switch : rebond mécanique ~1-2ms, sûr jusqu'à 1500 RPM avec 4 cibles)
volatile int rpm_moteur_1 = 0;             // RPM mesuré moteur 1 (volatile pour accès multi-thread)
volatile int rpm_moteur_2 = 0;             // RPM mesuré moteur 2 (volatile pour accès multi-thread)

// Handle de la tâche RPM sur Core 0
TaskHandle_t taskRPMHandle = NULL;

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
int rpm_theorique = 0;  // NOUVEAU : RPM théorique calculé
float spin_reel = 0.0;  // NOUVEAU : Spin réel mesuré en %
float tension1 = 0.0;
float tension2 = 0.0;
float courant1 = 0.0;
float courant2 = 0.0;
int regime1 = 0;
int regime2 = 0;

// Variables pour détecter les changements d'affichage
int vitesse_prev = -1;
int spin_prev = -999;
int rpm_theorique_prev = -1;  // NOUVEAU : pour détecter changement RPM théorique
float spin_reel_prev = -999.0;  // NOUVEAU : pour détecter changement spin réel
float tension_moteur_1_prev = -1.0;
float tension_moteur_2_prev = -1.0;
float courant1_prev = -1.0;
float courant2_prev = -1.0;
int regime1_prev = -1;
int regime2_prev = -1;

// Variables de mesure de tension
// Broche analogique
const int analogPinV1 = 34;
const int analogPinV2 = 39;
const int analogPinV3 = 36;
// Variables
int rawValue_1 = 0;
int rawValue_2 = 0;
int rawValue_3 = 0;
float tension_moteur_1 = 0.00;
float tension_moteur_2 = 0.00;
float tension_alimentation = 0.00;
// Résistances du pont diviseur
const float R1 = 30000.0;
const float R2 = 7500.0;

// Variables de mesure de courant (ACS712-20A)
// Broches analogiques pour les capteurs de courant
const int analogPinI1 = 35;  // GPIO35 pour capteur courant moteur 1
const int analogPinI2 = 25;  // GPIO32 pour capteur courant moteur 2 (à adapter selon votre câblage)
// Variables de mesure brute
int rawValueI1 = 0;
int rawValueI2 = 0;
// Constantes ACS712-20A
const float ACS712_SENSITIVITY = 0.100;  // 100 mV/A pour le modèle 20A
const float ACS712_ZERO_CURRENT = 1.43;   // Point milieu à 1.65V pour alimentation 3.3V (3.3V/2) pour un ESP32, 1.43V sur le GPIO35 après calibration

// Paramètres de filtrage pour stabiliser la mesure
const int NB_ECHANTILLONS = 100;   // Nombre d'échantillons pour moyenner les mesures analogiques (meilleure stabilité)
const float ALPHA_FILTRE = 0.2;   // Coefficient du filtre passe-bas (0.1 à 0.3 recommandé)

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

// MODIFIÉ : Calcul PWM avec application du spin (contrôle différentiel)
void rpm_pwm_calculation() {
  VITESSE = constrain(VITESSE, 0, V_MAX);
  rpm_input = (VITESSE * 1000 / 60) / (0.254 * 3.14159);
  
  // Calcul PWM de base
  int base_pwm = map(rpm_input, 0, MAX_RPM, 0, 255);
  
  // Application du spin (différence entre moteurs)
  // spinPercent positif = moteur 1 plus rapide (backspin)
  // spinPercent négatif = moteur 2 plus rapide (topspin)
  float spin_factor = spinPercent / 100.0;
  
  pwm_value_1 = constrain(base_pwm * (1.0 + spin_factor), 0, 255);
  pwm_value_2 = constrain(base_pwm * (1.0 - spin_factor), 0, 255);
}

// MODIFIÉ : Fonction de commande des 2 moteurs avec spin
void commandeMoteurs() {
  if (motorRunning) {
    // Moteur 1 (haut)
    ledcWrite(RPWM1_CHANNEL, pwm_value_1);
    ledcWrite(LPWM1_CHANNEL, 0);
    
    // Moteur 2 (bas)
    ledcWrite(RPWM2_CHANNEL, pwm_value_2);
    ledcWrite(LPWM2_CHANNEL, 0);
    
    Serial.print("PWM1: ");
    Serial.print(pwm_value_1);
    Serial.print(" PWM2: ");
    Serial.println(pwm_value_2);
  } else {
    ledcWrite(RPWM1_CHANNEL, 0);
    ledcWrite(LPWM1_CHANNEL, 0);
    ledcWrite(RPWM2_CHANNEL, 0);
    ledcWrite(LPWM2_CHANNEL, 0);
  }
}

// Fonction gestion écran TFT
void drawThickRect(int x, int y, int w, int h, int thickness, uint16_t color) {
  for (int i = 0; i < thickness; i++) {
    tft.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, color);
  }
}

// MODIFIÉ : Affichage Vitesse ballon en mode portrait (haut gauche, 24pt)
void updateVitesse() {
  // Ne mettre à jour que si la valeur a changé
  if (vitesse != vitesse_prev) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(15, 60);
    tft.println("Vitesse ballon");
    
    tft.setFreeFont(&FreeSans24pt7b);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.fillRect(5, 70, 150, 40, TFT_BLACK);
    tft.setCursor(25, 105);
    tft.printf("%3d", vitesse);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(100, 105);
    tft.println("km/h");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    vitesse_prev = vitesse;
  }
}

// MODIFIÉ : Affichage RPM cible en mode portrait (18pt)
void updateRPMTheorique() {
  // Ne mettre à jour que si la valeur a changé
  if (rpm_theorique != rpm_theorique_prev) {
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(70, 135);
    tft.println("Regime des moteurs");
    
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.fillRect(80, 140, 160, 30, TFT_BLACK);
    tft.setCursor(100, 165);
    tft.printf("%4d", rpm_theorique);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(190, 165);
    tft.println("rpm");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    rpm_theorique_prev = rpm_theorique;
  }
}

// MODIFIÉ : Affichage Spin théorique en mode portrait (haut droite, 24pt)
void updateSpin() {
  // Ne mettre à jour que si la valeur a changé
  if (spin != spin_prev) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(220, 60);
    tft.println("Spin");
    
    tft.setFreeFont(&FreeSans24pt7b);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillRect(165, 70, 150, 40, TFT_BLACK);
    tft.setCursor(205, 105);
    tft.printf("%3d", spin);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(280, 105);
    tft.println("%");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    spin_prev = spin;
  }
}

// MODIFIÉ : Affichage Tension Moteur 1 en mode portrait (18pt)
void updateTension1() {
  // Ne mettre à jour que si la valeur a changé (avec seuil de 0.1V pour éviter les micro-variations)
  if (abs(tension_moteur_1 - tension_moteur_1_prev) > 0.05) {
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.setCursor(10, 228);
    tft.println("Moteur 1");
    
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.fillRect(5, 240, 150, 30, TFT_BLACK);
    tft.setCursor(10, 265);
    tft.printf("U: %.1f", tension_moteur_1);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(125, 265);
    tft.println("V");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    tension_moteur_1_prev = tension_moteur_1;
  }
}

// MODIFIÉ : Affichage Courant Moteur 1 en mode portrait (18pt)
void updateCourant1() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(courant1 - courant1_prev) > 0.05) {
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.fillRect(165, 240, 150, 30, TFT_BLACK);
    tft.setCursor(175, 265);
    tft.printf("I: %.1f", courant1);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(280, 265);
    tft.println("A");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    courant1_prev = courant1;
  }
}

// MODIFIÉ : Affichage Tension Moteur 2 en mode portrait (18pt)
void updateTension2() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(tension_moteur_2 - tension_moteur_2_prev) > 0.05) {
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.setCursor(10, 298);
    tft.println("Moteur 2");
    
    tft.setFreeFont(&FreeSans18pt7b);
    tft.fillRect(5, 310, 150, 30, TFT_BLACK);
    tft.setCursor(10, 335);
    tft.printf("U: %.1f", tension_moteur_2);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(125, 335);
    tft.println("V");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    tension_moteur_2_prev = tension_moteur_2;
  }
}

// MODIFIÉ : Affichage Courant Moteur 2 en mode portrait (18pt)
void updateCourant2() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(courant2 - courant2_prev) > 0.05) {
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.fillRect(155, 310, 150, 30, TFT_BLACK);
    tft.setCursor(175, 335);
    tft.printf("I: %.1f", courant2);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(280, 335);
    tft.println("A");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // restaurer la couleur de police par défaut
    courant2_prev = courant2;
  }
}

// NOUVEAU : Affichage RPM mesurés (24pt) + Spin réel (18pt) en mode portrait
void updateRPMMesures() {
  if (abs(regime1 - regime1_prev) > 10 || abs(regime2 - regime2_prev) > 10) {
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(8, 370);
    tft.println("Regime des moteurs");
    
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.fillRect(0, 380, 179, 100, TFT_BLACK);
    tft.setCursor(10, 415);
    tft.printf("M1 : ");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.printf("%4d", regime1);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(150, 415);
    tft.println("rpm");
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setCursor(10, 465);
    tft.printf("M2 : ");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.printf("%4d", regime2);
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(150, 465);
    tft.println("rpm");
    
    regime1_prev = regime1;
    regime2_prev = regime2;
  }
  
  // Affichage spin réel (calculé à partir des RPM mesurés)
  if (abs(spin_reel - spin_reel_prev) > 0.5) {
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(215, 370);
    tft.println("SPIN reel");

    // Déterminer si le spin dépasse la valeur max autorisée
    float spin_affiche = spin_reel;
    bool spin_depasse = false;
    
    if (abs(spin_reel) > spinMax) {
      spin_affiche = (spin_reel > 0) ? spinMax : -spinMax;  // Capper à ±spinMax
      spin_depasse = true;
    }

    tft.setFreeFont(&FreeSans24pt7b);
    tft.fillRect(200, 380, 120, 100, TFT_BLACK);
    
    // Afficher en rouge si dépassement, sinon en vert
    if (spin_depasse) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    }
    
    tft.setCursor(200, 445);
    tft.printf("%3.0f", spin_affiche);  // %.0f pour arrondir sans décimales
    tft.setFreeFont(&FreeSans9pt7b);
    tft.setCursor(280, 445);
    tft.println("%");
    
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Restaurer couleur par défaut
    spin_reel_prev = spin_reel;
  }
}

// Fonction d'affichage RPM sur TM1637
void afficher_rpm_tm1637() {
  // Affichage moteur 1 sur TM1637
  if (abs(regime1 - regime1_prev) > 10) {
    displayRPM1.showNumberDec(regime1, false);  // Affiche le RPM sans zéros de tête
    regime1_prev = regime1;
  }
  
  // Affichage moteur 2 sur TM1637
  if (abs(regime2 - regime2_prev) > 10) {
    displayRPM2.showNumberDec(regime2, false);
    regime2_prev = regime2;
  }
}


// fonction de correction de mesure de tension
float corrigerTension(float tension_lue) {
  return 1.016 * tension_lue + 0.72;
}

// Fonction de calcul de tension
float MesureTension(int voltage) {
  float tension_mesure = (voltage / 4095.0) * 3.3;
  float tension_reelle = tension_mesure * ((R1 + R2) / R2);
  tension_reelle = corrigerTension(tension_reelle);
  return tension_reelle;
}

// Fonction de calcul de courant (ACS712-20A)
float MesureCourant(int rawValue) {
  // Conversion de la valeur ADC en tension (0-3.3V sur ESP32)
  float voltage = (rawValue / 4095.0) * 3.3;
  
  // Calcul du courant : (Tension - Point zéro) / Sensibilité
  // Pour ACS712-20A : 100mV/A, point zéro à 1.65V (pour VCC=3.3V)
  float courant = (voltage - ACS712_ZERO_CURRENT) / ACS712_SENSITIVITY;
  
  // Retourner la valeur absolue pour avoir le courant en Ampères
  return abs(courant);
}


// Fonction de calibration du point zéro (à appeler sans courant dans le moteur)
float calibrerPointZero(int pin) {
  long somme = 0;
  for (int i = 0; i < 200; i++) {
    somme += analogRead(pin);
    delay(5);
  }
  int rawAverage = somme / 200;
  float voltageZero = (rawAverage / 4095.0) * 3.3;
  Serial.print("Calibration capteur sur GPIO");
  Serial.print(pin);
  Serial.print(" : Point zero = ");
  Serial.print(voltageZero);
  Serial.println("V");
  return voltageZero;
}

// Fonction de mesure analogique avec moyenne (oversampling)
int mesureAnalogAvecMoyenne(int pin) {
  long somme = 0;
  for (int i = 0; i < NB_ECHANTILLONS; i++) {
    somme += analogRead(pin);
    delayMicroseconds(200);  // Délai entre les mesures pour couvrir plusieurs cycles PWM
  }
  return somme / NB_ECHANTILLONS;
}

// fonction de mesure de tension avec filtrage
void mesure_tension() {
  // Mesure avec moyenne de plusieurs échantillons
  rawValue_1 = mesureAnalogAvecMoyenne(analogPinV1);
  rawValue_2 = mesureAnalogAvecMoyenne(analogPinV2);
  rawValue_3 = mesureAnalogAvecMoyenne(analogPinV3);
  
  // Calcul de la nouvelle tension
  float nouvelle_tension_1 = MesureTension(rawValue_1);
  float nouvelle_tension_2 = MesureTension(rawValue_2);
  float nouvelle_tension_alim = MesureTension(rawValue_3);
  
  // Application d'un filtre passe-bas (lissage exponentiel)
  // Formule: valeur_filtrée = alpha * nouvelle_valeur + (1 - alpha) * ancienne_valeur
  tension_moteur_1 = ALPHA_FILTRE * nouvelle_tension_1 + (1.0 - ALPHA_FILTRE) * tension_moteur_1;
  tension_moteur_2 = ALPHA_FILTRE * nouvelle_tension_2 + (1.0 - ALPHA_FILTRE) * tension_moteur_2;
  tension_alimentation = ALPHA_FILTRE * nouvelle_tension_alim + (1.0 - ALPHA_FILTRE) * tension_alimentation;
}

// fonction de mesure de courant avec filtrage (ACS712)
void mesure_courant() {
  // Mesure avec moyenne de plusieurs échantillons
  rawValueI1 = mesureAnalogAvecMoyenne(analogPinI1);
  rawValueI2 = mesureAnalogAvecMoyenne(analogPinI2);
  
  // Calcul du nouveau courant
  float nouveau_courant_1 = MesureCourant(rawValueI1);
  float nouveau_courant_2 = MesureCourant(rawValueI2);
  
  // Application d'un filtre passe-bas (lissage exponentiel)
  courant1 = ALPHA_FILTRE * nouveau_courant_1 + (1.0 - ALPHA_FILTRE) * courant1;
  courant2 = ALPHA_FILTRE * nouveau_courant_2 + (1.0 - ALPHA_FILTRE) * courant2;
}

// Fonction de calcul du RPM à partir des impulsions
// Cette fonction sera appelée par la tâche dédiée sur Core 0
void calculer_rpm() {
  static unsigned long lastValidRPMTime1 = 0;  // Dernier moment où on a eu des pulses (moteur 1)
  static unsigned long lastValidRPMTime2 = 0;  // Dernier moment où on a eu des pulses (moteur 2)
  const unsigned long RPM_TIMEOUT = 500;       // Timeout 500ms avant d'afficher 0
  
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRPMCalc;
  
  if (deltaTime >= RPM_CALC_INTERVAL) {
    // Désactiver temporairement les interruptions pour lecture atomique
    noInterrupts();
    unsigned long pulses1 = pulseCount1;
    unsigned long pulses2 = pulseCount2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    interrupts();
    
    // Calcul RPM basé sur le temps réel écoulé
    if (deltaTime > 0) {
      // Moteur 1
      // MODIFIÉ : Seuil minimum de 2 pulses pour éviter les faux positifs à l'arrêt
      if (pulses1 >= 2) {
        rpm_moteur_1 = (pulses1 * 60000) / (PULSES_PER_REV * deltaTime);
        lastValidRPMTime1 = currentTime;  // Mise à jour du dernier moment valide
      } else if (currentTime - lastValidRPMTime1 > RPM_TIMEOUT) {
        // Pas de pulse depuis RPM_TIMEOUT ms = moteur arrêté
        rpm_moteur_1 = 0;
      }
      // Sinon, on garde la dernière valeur RPM (pas de mise à jour)
      
      // Moteur 2
      // MODIFIÉ : Seuil minimum de 2 pulses pour éviter les faux positifs à l'arrêt
      if (pulses2 >= 2) {
        rpm_moteur_2 = (pulses2 * 60000) / (PULSES_PER_REV * deltaTime);
        lastValidRPMTime2 = currentTime;
      } else if (currentTime - lastValidRPMTime2 > RPM_TIMEOUT) {
        rpm_moteur_2 = 0;
      }
    }
    
    lastRPMCalc = currentTime;
    
    // Debug : Afficher les valeurs brutes
    Serial.print("Pulses1: ");
    Serial.print(pulses1);
    Serial.print(" DeltaTime: ");
    Serial.print(deltaTime);
    Serial.print("ms -> RPM1: ");
    Serial.println(rpm_moteur_1);
  }
}

// Tâche FreeRTOS dédiée à la mesure RPM sur Core 0
void taskCalculRPM(void *parameter) {
  Serial.println("Tache RPM demarree sur Core 0");
  
  for(;;) {  // Boucle infinie
    calculer_rpm();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Délai 10ms (libère le CPU)
  }
}


//========================= Déclaration des interruptions =====================================

// Interruption Reed Switch moteur 1 avec anti-rebond
void IRAM_ATTR compteur_moteur_1() {
  unsigned long currentTime = millis();
  // Anti-rebond : ignorer les impulsions trop rapprochées (10ms sûr jusqu'à 1500 RPM)
  if (currentTime - lastPulseTime1 > DEBOUNCE_TIME) {
    pulseCount1++;
    lastPulseTime1 = currentTime;
  }
}

// Interruption Reed Switch moteur 2 avec anti-rebond
void IRAM_ATTR compteur_moteur_2() {
  unsigned long currentTime = millis();
  // Anti-rebond : ignorer les impulsions trop rapprochées (10ms sûr jusqu'à 1500 RPM)
  if (currentTime - lastPulseTime2 > DEBOUNCE_TIME) {
    pulseCount2++;
    lastPulseTime2 = currentTime;
  }
}

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
  analogReadResolution(12);

  // Initialisation de l'écran LCD
  lcd.begin (20, 4);
  lcd.init();
  lcd.backlight();

  // Initialisation de la liaison I2C
  Wire.begin(); 
  Wire.setClock(100000);

  // Initialisation du PCF8574 pour GPIO extender
  pcf8574.begin();
  
  // Configuration de la LED bouton engine start/stop
  pinMode(Led_ON, OUTPUT);
  digitalWrite(Led_ON, LOW);               // LED éteinte au démarrage

  // Configuration des boutons
  pinMode(BUTTON_ON, INPUT_PULLUP);            // Résistance de pull-up activée
  pinMode(pinBoutonPlus, INPUT_PULLUP);        // Résistance de pull-up activée
  pinMode(pinBoutonMoins, INPUT_PULLUP);       // Résistance de pull-up activée

  // Configuration des capteurs Reed Switch (RS 268-4855)
  pinMode(REED_MOTEUR_1, INPUT_PULLUP);        // Résistance de pull-up activée (Reed Switch = contact passif 2 fils)
  pinMode(REED_MOTEUR_2, INPUT_PULLUP);        // Résistance de pull-up activée

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
  
  // Attachement des interruptions Reed Switch (front descendant = aimant détecté, contact fermé)
  attachInterrupt(digitalPinToInterrupt(REED_MOTEUR_1), compteur_moteur_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(REED_MOTEUR_2), compteur_moteur_2, FALLING);
  
  // Création de la tâche dédiée au calcul RPM sur Core 0
  xTaskCreatePinnedToCore(
    taskCalculRPM,      // Fonction de la tâche
    "TaskRPM",          // Nom de la tâche
    4096,               // Taille de la pile (bytes)
    NULL,               // Paramètre passé à la tâche
    2,                  // Priorité (2 = haute priorité)
    &taskRPMHandle,     // Handle de la tâche
    0                   // Core 0 (Core 1 = loop principal)
  );
  
  Serial.println("Tache RPM creee sur Core 0");
  
  //============================ Setup écran TFT MODE PORTRAIT 320x480 ============================
  tft.init();
  tft.setRotation(0);  // MODIFIÉ : Portrait 320x480
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Titre avec fond bordeaux pour les valeurs cibles
  tft.fillRect(0, 0, 320, 33, TFT_DARKGREY);
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(60, 25);
  tft.println("VALEURS CIBLES");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Séparateur
  tft.drawFastHLine(0, 115, 320, TFT_DARKGREY);
  
  // Titre pour les mesures
  tft.fillRect(0, 175, 320, 30, TFT_DARKGREY);
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(39, 198);
  tft.println("MESURES MOTEURS");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // Séparateurs mesures
  tft.drawFastHLine(0, 275, 320, TFT_DARKGREY);
  tft.drawFastHLine(0, 345, 320, TFT_DARKGREY);
  tft.drawFastVLine(190, 360, 120, TFT_DARKGREY);
  
  // Affichage initial en mode portrait
  updateVitesse();
  updateSpin();
  updateRPMTheorique();
  updateTension1();
  updateCourant1();
  updateTension2();
  updateCourant2();
  updateRPMMesures();

  // PWM moteurs setup (MODIFIÉ : ajout moteur 2)
  ledcSetup(RPWM1_CHANNEL, 25000, 8); // fréquence 25kHz, 8-bit resolution
  ledcAttachPin(RPWM_1, RPWM1_CHANNEL);
  ledcSetup(LPWM1_CHANNEL, 25000, 8);
  ledcAttachPin(LPWM_1, LPWM1_CHANNEL);
  
  ledcSetup(RPWM2_CHANNEL, 25000, 8);
  ledcAttachPin(RPWM_2, RPWM2_CHANNEL);
  ledcSetup(LPWM2_CHANNEL, 25000, 8);
  ledcAttachPin(LPWM_2, LPWM2_CHANNEL);

  // Initialisation afficheurs TM1637 via PCF8574
  displayRPM1.begin();
  displayRPM1.setBrightness(0x0f);  // Luminosité maximale (0x00 à 0x0f)
  displayRPM1.showNumberDec(0);     // Affiche 0 au démarrage
  
  displayRPM2.begin();
  displayRPM2.setBrightness(0x0f);
  displayRPM2.showNumberDec(0);

  // Calibration des capteurs de courant (décommenter pour calibrer, moteurs à l'arrêt)
  // delay(2000);
  // Serial.println("=== CALIBRATION CAPTEURS COURANT ===");
  // float zero1 = calibrerPointZero(analogPinI1);
  // float zero2 = calibrerPointZero(analogPinI2);
  // Serial.println("Utilisez ces valeurs pour ACS712_ZERO_CURRENT");

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
      delay(200);
    }
  }
  engine_ss();
  spin_update();
  rpm_pwm_calculation();
  commandeMoteurs();  // MODIFIÉ : commande des 2 moteurs avec spin
  mesure_tension();
  mesure_courant();
  // calculer_rpm();  // SUPPRIMÉ : maintenant géré par la tâche sur Core 0

  // Calcul du spin réel à partir des RPM mesurés (NOUVEAU)
  // MODIFIÉ : Cast en float pour éviter les débordements et capper à ±100%
  float rpm_moyen = (rpm_moteur_1 + rpm_moteur_2) / 2.0;
  if (rpm_moyen > 50) {  // Évite division par zéro et calcul à très faible vitesse
    float spin_reel_rpm = rpm_moteur_1 - rpm_moteur_2;
    spin_reel = (spin_reel_rpm * 100.0) / rpm_moyen;
    // Capper le spin réel calculé à ±100% (sécurité pour l'affichage)
    spin_reel = constrain(spin_reel, -100.0, 100.0);
  } else {
    spin_reel = 0.0;
  }
  
  // Debug : afficher le spin réel calculé
  if (abs(rpm_moteur_1 - rpm_moteur_2) > 10) {
    Serial.print("RPM1: ");
    Serial.print(rpm_moteur_1);
    Serial.print(" RPM2: ");
    Serial.print(rpm_moteur_2);
    Serial.print(" -> Spin réel: ");
    Serial.print(spin_reel);
    Serial.println("%");
  }

  // Mise à jour des variables de fonctionnement avec valeurs mesurées
  vitesse = VITESSE;
  spin = spinPercent;
  rpm_theorique = rpm_input;  // RPM théorique pour affichage
  tension1 = tension_moteur_1;
  tension2 = tension_moteur_2;
  courant1 = courant1;
  courant2 = courant2;
  regime1 = rpm_moteur_1;  // RPM mesuré du moteur 1
  regime2 = rpm_moteur_2;  // RPM mesuré du moteur 2

  // Mise à jour de l'affichage TFT en mode portrait
  updateVitesse();
  updateSpin();
  updateRPMTheorique();
  updateTension1();
  updateCourant1();
  updateTension2();
  updateCourant2();
  updateRPMMesures();  // NOUVEAU : affiche RPM mesurés + spin réel
  
  // Mise à jour des afficheurs TM1637 pour les régimes
  afficher_rpm_tm1637();
}
