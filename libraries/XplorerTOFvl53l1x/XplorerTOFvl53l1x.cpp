// XplorerTOFvl53l1x.cpp
#include "XplorerTOFvl53l1x.h"

XplorerTOFvl53l1x::XplorerTOFvl53l1x(uint8_t addr)
  : address(addr) {}

bool XplorerTOFvl53l1x::begin() {
  sensor.setAddress(address);
  if (!sensor.init(true)) return false;

  // long-range preset
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(100000);
  sensor.startContinuous(100);

  // give the driver a finite timeout so it never blocks forever
  sensor.setTimeout(100);  // 100 ms

  return true;
}


// new overload: choose blocking (true) vs non-blocking (false)
int16_t XplorerTOFvl53l1x::readDistance(bool blocking) {
  return sensor.read(blocking);
}
