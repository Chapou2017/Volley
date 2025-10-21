#include <Arduino.h>
#include <Wire.h>
#include <I2CKeyPad.h>
#include <LiquidCrystal_I2C.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Déclaration des adresses I2C */

// Adresse I2C LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Adresse du PCF8574
#define I2C_ADDR 0x20

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Déclaration des variables */

// Clavier 4x3 via utilisation d'un PCF8574
I2CKeyPad clavier(I2C_ADDR); 
char keymap[19] = "123 456 789 *0# NF"; //N=NoKey F=Fail

String inputString = "";
int VITESSE = 0;
byte currentRow1 = 1;
byte currentRow2 = 1;

// Gestion du delay pour l'affichage du spin
unsigned long lastUpdate = 0;
const unsigned long interval = 50;


// Pins des boutons ON, OFF et de contrôle du spin
const int pinBoutonPlus = 14;   // GPIO14
const int pinBoutonMoins = 27;  // GPIO27
const int BUTTON_ON = 15;       // GPIO15
const int BUTTON_OFF = 26;      // GPIO26

// Variable pour suivre la mis en route des moteurs (via une LED)
volatile bool ledState = false;
volatile bool start_flag = false;
volatile bool stop_flag = false;
volatile unsigned long lastInterruptTime = 0;

// Variables pour le contrôle du spin et de son affichage
volatile int spinPercent = 0;
volatile bool majAffichage = false;
const int spinMax = 50;
const int spinMin = -50;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Déclaration des fonctions */ 

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

void start_engine() {
  if (start_flag == true) {
    lcd.clear();
    lcd.print("demarrage moteurs");
    start_flag = false;
    delay(1500);
    lcd.clear();
    printSpin(spinPercent);
    lcd.print("Vitesse [0-100]: ");
    lcdPrintln("Retenue: ");
    lcd.print(VITESSE);
    lcd.print(" km/h");
    lcd.setCursor(17, 0);    
  }
}

void stop_engine() {
  if (stop_flag == true) {
    lcd.clear();
    lcd.print("Arret moteurs");
    stop_flag = false;
    delay(1500);
    lcd.clear();
    printSpin(spinPercent);
    lcd.print("Vitesse [0-100]: ");
    lcdPrintln("Retenue: ");
    lcd.print(VITESSE);
    lcd.print(" km/h");
    lcd.setCursor(17, 0);
  }
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Déclaration des interruptions */

// Fonction de démarrage des moteurs
void IRAM_ATTR button_start() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {      // Anti-rebond : 300 ms
    ledState = true;                                // Allumage de la LED  
    lastInterruptTime = currentTime;
    digitalWrite(LED_BUILTIN, ledState);            // Mettre à jour la LED
    start_flag = true;  
  }           
}

// Fonction d'arrêt des moteurs
void IRAM_ATTR button_stop() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 300) {  // Anti-rebond : 300 ms
    ledState = false;                           // Extinction de la LED  
    lastInterruptTime = currentTime;
    digitalWrite(LED_BUILTIN, ledState);        // Mettre à jour la LED
    stop_flag = true; 
  }       
}

// Fonction d'interruption pour augmenter le spin
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

// Fonction d'interruption pour diminuer le spin
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


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup() {
    
  Serial.begin(115200);
  
  // Initialisation de l'écran LCD
  lcd.begin (20, 4);
  lcd.init();
  lcd.backlight();
 
  // Initialisation de la liaison I2C
  Wire.begin(); 
  Wire.setClock(400000);

  // Configuration de la LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);               // LED éteinte au démarrage

  // Configuration des boutons
  pinMode(BUTTON_ON, INPUT_PULLUP);            // Résistance de pull-up activée
  pinMode(BUTTON_OFF, INPUT_PULLUP);           // Résistance de pull-up activée
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
  attachInterrupt(digitalPinToInterrupt(BUTTON_OFF), button_stop, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonPlus), augmenterSpin, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinBoutonMoins), diminuerSpin, FALLING);
}


void loop() {

  char key = clavier.getChar();
  if (key != 0) { // Si une touche est pressée (key différente de "null")
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

  start_engine();
  stop_engine();

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


// Broches de contrôle
const int RPWM = 25;
const int LPWM = 26;

int pwmValue = 0; // Valeur PWM reçue via le port série

void setup() {
  Serial.begin(115200);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // Configuration PWM
  ledcSetup(0, 20000, 8); // Canal 0, 20 kHz, 8 bits
  ledcSetup(1, 20000, 8); // Canal 1, 20 kHz, 8 bits

  ledcAttachPin(RPWM, 0);
  ledcAttachPin(LPWM, 1);

  Serial.println("Entrez une valeur entre 0 et 255 pour régler la vitesse du moteur.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Supprime les espaces

    int value = input.toInt();
    if (value >= 0 && value <= 255) {
      pwmValue = value;
      Serial.print("Vitesse réglée à : ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Valeur invalide. Entrez un nombre entre 0 et 255.");
    }
  }

  // Marche avant
  ledcWrite(0, pwmValue); // RPWM
  ledcWrite(1, 0);        // LPWM
}
