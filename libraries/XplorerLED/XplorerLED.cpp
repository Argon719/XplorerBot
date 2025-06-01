#include "XplorerLED.h"

// Konstruktor: Pin für die LED festlegen
XplorerLED::XplorerLED(uint8_t pin) : pin(pin) {
    state.isOn = false;
    state.brightness = 0;
}

// Initialisierungsfunktion
void XplorerLED::begin() {
    pinMode(pin, OUTPUT);
    apply();  // Setze die LED in den initialen Zustand
}

// LED einschalten
void XplorerLED::turnOn() {
    state.isOn = true;
    apply();
}

// LED ausschalten
void XplorerLED::turnOff() {
    state.isOn = false;
    apply();
}

// Helligkeit der LED setzen
void XplorerLED::setBrightness(uint8_t brightness) {
    state.brightness = brightness;
    if (state.isOn) {
        apply();  // Wende die Helligkeit an, wenn die LED eingeschaltet ist
    }
}

// aktuellen Zustand der LED abrufen
LEDState XplorerLED::getState() const {
    return state;
}

// Anwenden des Zustands auf die LED
void XplorerLED::apply() {
    if (state.isOn) {
        analogWrite(pin, state.brightness);  // Setze die Helligkeit
    } else {
        analogWrite(pin, 0);  // Schalte die LED aus
    }
}
