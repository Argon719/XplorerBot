#ifndef XPLORERTCA9548A_H
#define XPLORERTCA9548A_H

#include <Arduino.h>
#include <Wire.h>

class XplorerTCA9548A {
public:
  // Konstruktor, um die I2C-Adresse des Multiplexers festzulegen (Standard: 0x70)
  XplorerTCA9548A(uint8_t address = 0x70);

  // Funktion zum Auswählen eines Kanals (0–7) auf dem Multiplexer
  void selectI2CMuxChannel(uint8_t channel);

private:
  uint8_t _address;  // I2C-Adresse des TCA9548A
};

#endif
