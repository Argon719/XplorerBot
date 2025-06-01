#include "XplorerTCA9548A.h"

// Konstruktor setzt die Adresse des Multiplexers
XplorerTCA9548A::XplorerTCA9548A(uint8_t address) : _address(address) {}

// Auswahl eines Kanals auf dem TCA9548A
void XplorerTCA9548A::selectI2CMuxChannel(uint8_t channel) {
  if (channel > 7) return;  // Sicherstellen, dass der Kanal gültig ist (0-7)
  
  Wire.beginTransmission(_address);
  Wire.write(1 << channel);  // Wählt den angegebenen Kanal durch Bitshift
  Wire.endTransmission();
}
