// XplorerTOFvl53l1x.h
#ifndef XPLORERTOFVL53L1X_H
#define XPLORERTOFVL53L1X_H

#include <Wire.h>
#include <VL53L1X.h>

class XplorerTOFvl53l1x {
private:
  VL53L1X sensor;
  uint8_t address;

public:
  // ctor: pass your sensor's I2C address (default 0x29)
  XplorerTOFvl53l1x(uint8_t addr = 0x29);

  // initialize bus, set long‐range, 200 ms budget, continuous @220 ms
  bool begin();

  // blocking read in mm (old behavior)
  int16_t readDistance();

  // non-blocking read: returns the last available sample immediately
  // (pass false to avoid any polling)
  int16_t readDistance(bool blocking);

  // optional: expose setter for I/O timeout (in milliseconds)
  void setTimeout(uint16_t timeout_ms) { sensor.setTimeout(timeout_ms); }
};

#endif // XPLORERTOFVL53L1X_H
