#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// Variables dynamiques
int vitesse = 0;
int spin = 0;
float tension1 = 0.0;
float tension2 = 0.0;
int regime1 = 0;
int regime2 = 0;

void drawThickRect(int x, int y, int w, int h, int thickness, uint16_t color) {
  for (int i = 0; i < thickness; i++) {
    tft.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, color);
  }
}

// Fonctions d'affichage
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

// --------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
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

  // Affichage Vitesse
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(20, 80);
//  tft.println("Vitesse de ballon");

//  tft.setFreeFont(&FreeSans24pt7b);
//  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
//  tft.setCursor(30, 150);
//  tft.println("XXX");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setTextColor(TFT_WHITE, TFT_BLACK);
//  tft.setCursor(145, 150);
//  tft.println("km/h");

  // Affichage Effet ballon
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(300, 80);
//  tft.println("Effet (spin)");

//  tft.setFreeFont(&FreeSans24pt7b);
//  tft.setTextColor(TFT_GREEN, TFT_BLACK);
//  tft.setCursor(320, 150);
//  tft.println("XX");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setTextColor(TFT_WHITE, TFT_BLACK);
//  tft.setCursor(420, 150);
//  tft.println("%");

  // Affichage Tension moteur 1
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(17, 210);
//  tft.println("Tension");
//  tft.setCursor(15, 237);
//  tft.println("moteur 1");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setCursor(12, 290);
//  tft.println("X.XX v");

  // Affichage Tension moteur 2
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(138, 210);
//  tft.println("Tension");
//  tft.setCursor(135, 237);
//  tft.println("moteur 2");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setCursor(135, 290);
//  tft.println("X.XX v");

  // Affichage Régime moteur 1
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(259, 210);
//  tft.println("Regime");
//  tft.setCursor(256, 237);
//  tft.println("moteur 1");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setCursor(260, 279);
//  tft.println("XXXX");
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(270, 305);
//  tft.println("tr/min");

  // Affichage Régime moteur 2
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(380, 210);
//  tft.println("Regime");
//  tft.setCursor(377, 237);
//  tft.println("moteur 2");
//  tft.setFreeFont(&FreeSans18pt7b);
//  tft.setCursor(380, 279);
//  tft.println("XXXX");
//  tft.setFreeFont(&FreeSans12pt7b);
//  tft.setCursor(390, 305);
//  tft.println("tr/min");
}

void loop() {
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

