#ifndef XPLORERLED_H
#define XPLORERLED_H

#include <Arduino.h>

// Struktur zur Speicherung des LED-Zustands
struct LEDState {
    bool isOn;           // Speichert, ob die LED an oder aus ist
    uint8_t brightness;  // Helligkeit der LED (0-255)
};

// LED-Steuerungsklasse
class XplorerLED {
public:
    // Konstruktor zum Festlegen des LED-Pins
    XplorerLED(uint8_t pin);

    // Initialisierungsfunktion
    void begin();

    // Funktion zum Einschalten der LED
    void turnOn();

    // Funktion zum Ausschalten der LED
    void turnOff();

    // Funktion zum Setzen der Helligkeit
    void setBrightness(uint8_t brightness);

    // Funktion zum Abrufen des aktuellen Zustands
    LEDState getState() const;

private:
    uint8_t pin;
    LEDState state;

    // Funktion zum Anwenden des aktuellen Zustands auf die LED
    void apply();
};

#endif // XPLORERLED_H
