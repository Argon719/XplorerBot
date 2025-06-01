#ifndef XPLORER74HC4052_H
#define XPLORER74HC4052_H

#include <Arduino.h>

class Xplorer74HC4052 {
public:
    // Konstruktor zum Festlegen der Steuerpins
    Xplorer74HC4052(uint8_t s0_pin, uint8_t s1_pin);

    // Initialisierungsfunktion
    void begin();

    // Funktion zum Setzen des Mux-Kanals
    void setChannel(uint8_t channel);

private:
    uint8_t s0_pin, s1_pin;
};

#endif // XPLORER74HC4052_H
