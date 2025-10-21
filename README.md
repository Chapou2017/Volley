# Volley

Projet ESP32 pour machine de lancement de ballon volley :
 - Affichage LCD I2C 20x4 (vitesse, spin, messages)
 - Affichage graphique TFT (vitesse, spin, tensions, régimes)
 - Saisie de la vitesse via keypad I2C (PCF8574)
 - Contrôle moteurs via PWM (RPWM / LPWM)
 - Boutons start/stop + réglage spin +/- (interruptions)

## Structure PlatformIO
```
platformio.ini
src/
	main.cpp
lib/ (optionnel pour config TFT_eSPI)
```

## Configuration rapide
1. Installer l'extension PlatformIO IDE dans VS Code.
2. Ouvrir ce dossier (`Volley`). PlatformIO détectera `platformio.ini`.
3. Adapter la configuration de l'écran TFT dans `lib/TFT_eSPI/User_Setup.h` (cf. notes dans `platformio.ini`).
4. Vérifier câblage :
	 - I2C (SDA, SCL) pour LCD + keypad (PCF8574).
	 - Pins moteurs: RPWM=25, LPWM=33 (adapter si besoin). 
	 - Boutons: START=15, STOP=32, SPIN+=14, SPIN-=27.
	 - LED intégrée `LED_BUILTIN` pour état moteur.

## Dépendances (gérées automatiquement)
Déclarées dans `platformio.ini`:
 - I2CKeyPad
 - LiquidCrystal_I2C
 - TFT_eSPI

## Compilation & Upload (PowerShell)
Via interface VS Code (boutons PlatformIO) ou en ligne de commande:
```powershell
pio run              # Compile
pio run --target upload  # Upload vers carte
pio device monitor   # Ouvrir le moniteur série (115200)
```

## Utilisation
1. Démarrer carte, TFT affiche les réglages, LCD demande la vitesse.
2. Saisir vitesse (0-100) sur le keypad puis '#' pour valider.
3. Appuyer sur bouton START pour démarrer moteurs (PWM appliquée).
4. Boutons SPIN+ / SPIN- modifient le pourcentage d'effet (affiché instantanément TFT + LCD). 
5. Bouton STOP coupe les moteurs (PWM=0).
6. '*' réinitialise l'entrée vitesse avant validation.

## Adaptations possibles
Pour désactiver un des deux écrans (si absence matériel), encapsuler le code par des macros:
```cpp
#define USE_LCD 1
#define USE_TFT 1
#if USE_LCD
	// Code LCD...
#endif
#if USE_TFT
	// Code TFT...
#endif
```

## Mesures supplémentaires
Les champs tension1/2 et regime1/2 sont des placeholders. Ajouter vos lectures ADC / capteurs et appeler:
```cpp
tension1 = lireTensionMoteur1();
updateTension1TFT();
```
Idem pour régime.

## Débogage
Messages série: confirmation vitesse, erreurs (valeur hors plage), impossibilité d'initialiser le keypad.
Vérifier le câblage si le programme se fige au démarrage (boucle d'erreur keypad).

## Licence
Projet privé (adapter si diffusion publique).

