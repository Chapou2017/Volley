#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <I2CKeyPad.h>
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
I2CKeyPad clavier(I2C_ADDR); 

// KEYMAP corrigé pour votre câblage direct spécifique
// Câblage physique: P0=C2, P1=L1, P2=C1, P3=L4, P4=C3, P5=L3, P6=L2
// 
// Matrice du clavier standard:
//        C1   C2   C3
//   L1    1    2    3
//   L2    4    5    6
//   L3    7    8    9
//   L4    *    0    #
//
// Format: exactement 18 caractères (même longueur que "123 456 789 *0# NF")
char keymap[19] = " 21 3 546879*0# NF"; // 18 chars: espace-2-1-espace-3-espace-5-4-6-8-7-9-*-0-#-espace-N-F

// Ancien câblage croisé (pour référence)
// char keymap[19] = "123 456 789 *0# NF"; //Câblage: L1→P5, L2→P0, L3→P4, L4→P3, C1→P6, C2→P2, C3→P1

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

// Variables pour la régulation PID du régime moteur
float Kp = 0.5;                   // Gain proportionnel (à ajuster)
float Ki = 0.1;                   // Gain intégral (à ajuster)
float Kd = 0.05;                  // Gain dérivé (à ajuster)
float erreur_precedente_1 = 0;    // Erreur précédente moteur 1
float erreur_precedente_2 = 0;    // Erreur précédente moteur 2
float somme_erreurs_1 = 0;        // Somme des erreurs moteur 1 (terme intégral)
float somme_erreurs_2 = 0;        // Somme des erreurs moteur 2 (terme intégral)
int pwm_value_moteur_1 = 0;       // PWM régulé moteur 1
int pwm_value_moteur_2 = 0;       // PWM régulé moteur 2
const float INTEGRAL_MAX = 100.0; // Limite anti-windup pour l'intégrale

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

// Pins des capteurs TCRT5000 pour mesure de vitesse
const int TCRT_MOTEUR_1 = 5;    // GPIO5 pour capteur vitesse moteur 1
const int TCRT_MOTEUR_2 = 13;   // GPIO13 pour capteur vitesse moteur 2
const int PULSES_PER_REV = 1;   // Nombre de trous/encoches par tour (à ajuster selon votre disque)

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

// Variables pour la mesure de vitesse (TCRT5000)
volatile unsigned long pulseCount1 = 0;    // Compteur d'impulsions moteur 1
volatile unsigned long pulseCount2 = 0;    // Compteur d'impulsions moteur 2
unsigned long lastRPMCalc = 0;             // Dernier calcul de RPM
const unsigned long RPM_CALC_INTERVAL = 100; // Intervalle de calcul RPM en ms
int rpm_moteur_1 = 0;                      // RPM mesuré moteur 1
int rpm_moteur_2 = 0;                      // RPM mesuré moteur 2

// Variables pour le contrôle du spin et de son affichage
volatile int spinPercent = 0;
volatile bool majAffichage = false;
const int spinMax = 50;
const int spinMin = -50;

// Variables pour la gestion anti-rebond du clavier
unsigned long lastKeyPressTime = 0;
const unsigned long keyDebounceDelay = 400;  // Délai anti-rebond en millisecondes
char lastKey = 0;  // Dernière touche pressée

// Variables dynamiques - affichage sur TFT
int vitesse = 0;
int spin = 0;
float tension1 = 0.0;
float tension2 = 0.0;
float courant1 = 0.0;
float courant2 = 0.0;
int regime1 = 0;
int regime2 = 0;

// Variables pour détecter les changements d'affichage
int vitesse_prev = -1;
int spin_prev = -999;
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
const int NB_ECHANTILLONS = 100;  // Nombre d'échantillons pour la moyenne
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

void rpm_pwm_calculation() {
  VITESSE = constrain(VITESSE, 0, V_MAX);
  rpm_input = (VITESSE * 1000 / 60) / (0.254 * 3.14159);
  pwm_value = map(rpm_input, 0, MAX_RPM, 0, 255);  // Valeur de départ (feedforward)
}

// Fonction de régulation PID pour le moteur 1
void reguler_moteur_1() {
  if (!motorRunning) {
    // Réinitialiser la régulation quand le moteur est arrêté
    pwm_value_moteur_1 = 0;
    somme_erreurs_1 = 0;
    erreur_precedente_1 = 0;
    return;
  }
  
  // Calcul de l'erreur (consigne - mesure)
  float erreur = rpm_input - rpm_moteur_1;
  
  // Terme proportionnel
  float P = Kp * erreur;
  
  // Terme intégral (avec anti-windup)
  somme_erreurs_1 += erreur;
  somme_erreurs_1 = constrain(somme_erreurs_1, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * somme_erreurs_1;
  
  // Terme dérivé
  float D = Kd * (erreur - erreur_precedente_1);
  erreur_precedente_1 = erreur;
  
  // Calcul de la correction PID
  float correction = P + I + D;
  
  // Application de la correction au PWM de base (feedforward + feedback)
  pwm_value_moteur_1 = constrain(pwm_value + correction, 0, 255);
}

// Fonction de régulation PID pour le moteur 2
void reguler_moteur_2() {
  if (!motorRunning) {
    // Réinitialiser la régulation quand le moteur est arrêté
    pwm_value_moteur_2 = 0;
    somme_erreurs_2 = 0;
    erreur_precedente_2 = 0;
    return;
  }
  
  // Calcul de l'erreur (consigne - mesure)
  float erreur = rpm_input - rpm_moteur_2;
  
  // Terme proportionnel
  float P = Kp * erreur;
  
  // Terme intégral (avec anti-windup)
  somme_erreurs_2 += erreur;
  somme_erreurs_2 = constrain(somme_erreurs_2, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * somme_erreurs_2;
  
  // Terme dérivé
  float D = Kd * (erreur - erreur_precedente_2);
  erreur_precedente_2 = erreur;
  
  // Calcul de la correction PID
  float correction = P + I + D;
  
  // Application de la correction au PWM de base (feedforward + feedback)
  pwm_value_moteur_2 = constrain(pwm_value + correction, 0, 255);
}

// Fonction de commande du moteur 1
void commandeMoteur1() {
  if (motorRunning) {
    ledcWrite(RPWM1_CHANNEL, pwm_value_moteur_1);  // Utilise le PWM régulé
    ledcWrite(LPWM1_CHANNEL, 0);
    // Debug : afficher consigne, mesure et PWM
    Serial.print("M1 - Cible:");
    Serial.print(rpm_input);
    Serial.print(" Mesure:");
    Serial.print(rpm_moteur_1);
    Serial.print(" PWM:");
    Serial.println(pwm_value_moteur_1);
  } else {
    ledcWrite(RPWM1_CHANNEL, 0);
    ledcWrite(LPWM1_CHANNEL, 0);
  }
}

// Fonction de commande du moteur 2
void commandeMoteur2() {
  if (motorRunning) {
    ledcWrite(RPWM2_CHANNEL, pwm_value_moteur_2);  // Utilise le PWM régulé
    ledcWrite(LPWM2_CHANNEL, 0);
    // Debug
    Serial.print("M2 - Cible:");
    Serial.print(rpm_input);
    Serial.print(" Mesure:");
    Serial.print(rpm_moteur_2);
    Serial.print(" PWM:");
    Serial.println(pwm_value_moteur_2);
  } else {
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

void updateVitesse() {
  // Ne mettre à jour que si la valeur a changé
  if (vitesse != vitesse_prev) {
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
    vitesse_prev = vitesse;
  }
}

void updateSpin() {
  // Ne mettre à jour que si la valeur a changé
  if (spin != spin_prev) {
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
    spin_prev = spin;
  }
}

void updateTension1() {
  // Ne mettre à jour que si la valeur a changé (avec seuil de 0.1V pour éviter les micro-variations)
  if (abs(tension_moteur_1 - tension_moteur_1_prev) > 0.05) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setCursor(17, 210);
    tft.println("Tension");
    tft.setCursor(15, 237);
    tft.println("moteur 1");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(10, 260, 100, 40, TFT_BLACK);
    tft.setCursor(15, 290);
    tft.printf("%.1f  v", tension_moteur_1);
    tension_moteur_1_prev = tension_moteur_1;
  }
}

void updateCourant1() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(courant1 - courant1_prev) > 0.05) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setCursor(137, 210);
    tft.println("Courant");
    tft.setCursor(135, 237);
    tft.println("moteur 1");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(130, 260, 100, 40, TFT_BLACK);
    tft.setCursor(135, 290);
    tft.printf("%.1f  A", courant1);
    courant1_prev = courant1;
  }
}

void updateTension2() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(tension_moteur_2 - tension_moteur_2_prev) > 0.05) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setCursor(259, 210);
    tft.println("Tension");
    tft.setCursor(256, 237);
    tft.println("moteur 2");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(250, 260, 100, 40, TFT_BLACK);
    tft.setCursor(255, 290);
    tft.printf("%.1f  v", tension_moteur_2);
    tension_moteur_2_prev = tension_moteur_2;
  }
}

void updateCourant2() {
  // Ne mettre à jour que si la valeur a changé
  if (abs(courant2 - courant2_prev) > 0.05) {
    tft.setFreeFont(&FreeSans12pt7b);
    tft.setCursor(380, 210);
    tft.println("Courant");
    tft.setCursor(377, 237);
    tft.println("moteur 2");
    tft.setFreeFont(&FreeSans18pt7b);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(370, 260, 100, 40, TFT_BLACK);
    tft.setCursor(375, 290);
    tft.printf("%.1f  A", courant2);
    courant2_prev = courant2;
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
void calculer_rpm() {
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
    
    // Calcul RPM : (impulsions / encoches_par_tour) * (60000 / deltaTime_ms)
    rpm_moteur_1 = (pulses1 * 60000) / (PULSES_PER_REV * deltaTime);
    rpm_moteur_2 = (pulses2 * 60000) / (PULSES_PER_REV * deltaTime);
    
    lastRPMCalc = currentTime;
  }
}


//========================= Déclaration des interruptions =====================================

// Interruption TCRT5000 moteur 1
void IRAM_ATTR compteur_moteur_1() {
  pulseCount1++;
}

// Interruption TCRT5000 moteur 2
void IRAM_ATTR compteur_moteur_2() {
  pulseCount2++;
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

  // Configuration des capteurs TCRT5000
  pinMode(TCRT_MOTEUR_1, INPUT_PULLUP);        // Résistance de pull-up activée
  pinMode(TCRT_MOTEUR_2, INPUT_PULLUP);        // Résistance de pull-up activée

  // Initialisation du clavier numérique
  if (clavier.begin() == false){
    Serial.println("Cannot communicate with keypad. Please check");
    while (1);
    }
  clavier.loadKeyMap(keymap);
  
  // MODE DEBUG : Afficher les index des touches détectées
  Serial.println("=== MODE DEBUG CLAVIER ===");
  Serial.println("Appuyez sur chaque touche et notez l'index affiche:");
  Serial.println("Cela permettra de construire le bon keymap");
  Serial.println("Format attendu: touche physique -> index detecte");
  Serial.println("==============================");

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
  
  // Attachement des interruptions TCRT5000 (front descendant = passage du noir au blanc)
  attachInterrupt(digitalPinToInterrupt(TCRT_MOTEUR_1), compteur_moteur_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(TCRT_MOTEUR_2), compteur_moteur_2, FALLING);
  
  //============================ Setup écran TFT ============================
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Dessin des rectangles (X, Y, largeur, hauteur, épaisseur)
  drawThickRect(0, 50, 239, 130, 3, TFT_YELLOW); // Rectangle "Vitesse ballon"
  drawThickRect(241, 50, 238, 130, 3, TFT_GREEN); // Rectangle "% Spin"
  drawThickRect(0, 182, 118, 132, 2, TFT_LIGHTGREY); // Rectangle "Tension moteur 1"
  drawThickRect(120, 182, 119, 132, 2, TFT_LIGHTGREY); // Rectangle "Courant moteur 1"
  drawThickRect(241, 182, 119, 132, 2, TFT_LIGHTGREY); // Rectangle "Tension moteur 2"
  drawThickRect(362, 182, 118, 132, 2, TFT_LIGHTGREY); // Rectangle "Courant moteur 2"

  // Titre
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setCursor(155, 30);
  tft.println("Reglages actuels");

  // Affichage initial
  updateVitesse();
  updateSpin();
  updateTension1();
  updateCourant1();
  updateTension2();
  updateCourant2();

  // PWM mot1 setup
  ledcSetup(RPWM1_CHANNEL, 25000, 8); // fréquence 25kHz, 8-bit resolution
  ledcAttachPin(RPWM_1, RPWM1_CHANNEL);
  ledcSetup(LPWM1_CHANNEL, 25000, 8);
  ledcAttachPin(LPWM_1, LPWM1_CHANNEL);

  // PWM mot2 setup
  ledcSetup(RPWM2_CHANNEL, 25000, 8); // fréquence 25kHz, 8-bit resolution
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

  // DEBUG : Afficher l'index brut détecté (ignorer N et F qui sont les valeurs par défaut)
  if (key != 0 && key != 'N' && key != 'F') {
    uint8_t rawIndex = clavier.getLastKey();
    Serial.print("Touche detectee: '");
    Serial.print(key);
    Serial.print("' -> Index brut: ");
    Serial.println(rawIndex);
  }

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
  rpm_pwm_calculation();    // Calcul du RPM cible et PWM de base
  calculer_rpm();           // Calcul des RPM mesurés à partir des impulsions TCRT5000
  
  // Régulation PID des moteurs
  reguler_moteur_1();       // Ajuste le PWM du moteur 1 selon la mesure
  reguler_moteur_2();       // Ajuste le PWM du moteur 2 selon la mesure
  
  // Commande des moteurs avec PWM régulé
  commandeMoteur1();
  commandeMoteur2();
  
  mesure_tension();
  mesure_courant();

  // Mise à jour des variables de fonctionnement avec valeurs mesurées
  vitesse = VITESSE;
  spin = spinPercent;
  tension1 = tension_moteur_1;
  tension2 = tension_moteur_2;
  courant1 = courant1;
  courant2 = courant2;
  regime1 = rpm_moteur_1;  // RPM mesuré du moteur 1
  regime2 = rpm_moteur_2;  // RPM mesuré du moteur 2

  // Mise à jour de l'affichage TFT
  updateVitesse();
  updateSpin();
  updateTension1();
  updateCourant1();
  updateTension2();
  updateCourant2();
  
  // Mise à jour des afficheurs TM1637 pour les régimes
  afficher_rpm_tm1637();
}

