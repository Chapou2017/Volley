#ifndef I2CKEYPAD_CUSTOM_H
#define I2CKEYPAD_CUSTOM_H

#include <Wire.h>

// Classe I2CKeyPad personnalisée pour Adafruit PID 3845
// Pinout Adafruit: Br1=C2, Br2=L1, Br3=C1, Br4=L4, Br5=C3, Br6=L3, Br7=L2
// Câblage direct souhaité: Br1→P0, Br2→P1, Br3→P2, Br4→P3, Br5→P4, Br6→P5, Br7→P6

class I2CKeyPad_Custom {
private:
    uint8_t _address;
    char* _keymap;
    uint8_t _lastKey;
    
    // Mapping personnalisé des pins PCF8574 vers matrice logique
    // P0=C2, P1=L1, P2=C1, P3=L4, P4=C3, P5=L3, P6=L2
    // Réorganisé en: C1=P2, C2=P0, C3=P4, L1=P1, L2=P6, L3=P5, L4=P3
    
    void writeToExpander(uint8_t value) {
        Wire.beginTransmission(_address);
        Wire.write(value);
        Wire.endTransmission();
    }
    
    uint8_t readFromExpander() {
        Wire.requestFrom(_address, (uint8_t)1);
        if (Wire.available()) {
            return Wire.read();
        }
        return 0xFF;
    }
    
    // Fonction pour activer une colonne selon le pinout Adafruit
    uint8_t activateColumn(uint8_t col) {
        // col: 0=C1, 1=C2, 2=C3
        // Mapping: C1=P2, C2=P0, C3=P4
        uint8_t pattern = 0xFF;  // Toutes les pins à HIGH par défaut
        
        switch(col) {
            case 0: pattern &= ~(1 << 2); break;  // C1 sur P2
            case 1: pattern &= ~(1 << 0); break;  // C2 sur P0
            case 2: pattern &= ~(1 << 4); break;  // C3 sur P4
        }
        
        return pattern;
    }
    
    // Fonction pour lire les lignes selon le pinout Adafruit
    uint8_t readRows(uint8_t rawData) {
        // Mapping: L1=P1, L2=P6, L3=P5, L4=P3
        uint8_t rows = 0;
        
        if (!(rawData & (1 << 1))) rows |= (1 << 0);  // L1 depuis P1
        if (!(rawData & (1 << 6))) rows |= (1 << 1);  // L2 depuis P6
        if (!(rawData & (1 << 5))) rows |= (1 << 2);  // L3 depuis P5
        if (!(rawData & (1 << 3))) rows |= (1 << 3);  // L4 depuis P3
        
        return rows;
    }

public:
    I2CKeyPad_Custom(uint8_t address) : _address(address), _keymap(nullptr), _lastKey(16) {}
    
    bool begin() {
        Wire.beginTransmission(_address);
        return (Wire.endTransmission() == 0);
    }
    
    void loadKeyMap(char* keymap) {
        _keymap = keymap;
    }
    
    uint8_t getKey() {
        for (uint8_t col = 0; col < 3; col++) {
            // Active la colonne
            uint8_t pattern = activateColumn(col);
            writeToExpander(pattern);
            delayMicroseconds(100);  // Délai de stabilisation
            
            // Lit l'état des lignes
            uint8_t rawData = readFromExpander();
            uint8_t rows = readRows(rawData);
            
            // Détecte quelle ligne est active
            for (uint8_t row = 0; row < 4; row++) {
                if (rows & (1 << row)) {
                    _lastKey = row * 3 + col;
                    return _lastKey;
                }
            }
        }
        
        _lastKey = 16;  // NoKey
        return 16;
    }
    
    uint8_t getLastKey() {
        return _lastKey;
    }
    
    char getChar() {
        uint8_t key = getKey();
        if (_keymap && key < 18) {
            return _keymap[key];
        }
        return 0;
    }
};

#endif
