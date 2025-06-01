#include "Xplorer74HC4052.h"

// Konstruktor zur Festlegung der Pins für S0 und S1
Xplorer74HC4052::Xplorer74HC4052(uint8_t s0, uint8_t s1) : s0_pin(s0), s1_pin(s1) {}

void Xplorer74HC4052::begin() {
    pinMode(s0_pin, OUTPUT);
    pinMode(s1_pin, OUTPUT);

    // Setze die Pins initial auf LOW
    digitalWrite(s0_pin, LOW);
    digitalWrite(s1_pin, LOW);
}

// Funktion zur Auswahl des Kanals (0 bis 3)
void Xplorer74HC4052::setChannel(uint8_t channel) {
    // Begrenze den Kanalwert auf 2 Bit (0-3)
    channel = channel & 0x03;

    // Setze die Steuersignale S0 und S1 basierend auf dem Kanalwert
    digitalWrite(s0_pin, (channel & 0x01));
    digitalWrite(s1_pin, (channel & 0x02) >> 1);
}
