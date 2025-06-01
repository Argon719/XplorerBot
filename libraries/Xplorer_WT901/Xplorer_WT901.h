#ifndef XPLORER_WT901_H
#define XPLORER_WT901_H

#include <Arduino.h>
#include "wit_c_sdk.h"
#include "REG.h"
#include "Xplorer74HC4052.h"

// Aus wit_c_sdk: sReg[] enthält die Roh-Registerwerte vom WT901.
extern int16_t sReg[];

class Xplorer_WT901 {
public:
  Xplorer_WT901(HardwareSerial& serial, int rxPin, int txPin, int s0Pin, int s1Pin, int muxChannel);

  // Muss einmal aufgerufen werden, um WT901-SDK und UART/MUX einzurichten
  void begin();

  // Muss periodisch (z.B. in einer FreeRTOS-Task) aufgerufen werden, 
  // damit neue Roh-Daten eingelesen und die gespeicherten Referenzen abgezogen werden.
  void update();

  // Liefert den aktuellen, gegen die Referenz korrigierten Roll-Winkel (±180°)
  float getRoll() const;

  // Liefert den aktuellen, gegen die Referenz korrigierten Pitch-Winkel (±180°)
  float getPitch() const;

  // Liefert den absoluten Yaw-Winkel (±180°), Referenz wird hier nicht verändert.
  float getYaw() const;

  // Setzt Roll und Pitch auf die aktuelle Ausrichtung als „0“. 
  // Yaw bleibt unberührt.
  void resetReference();

private:
  // Callback für WitReadReg / WitWriteReg
  static void sensorDataCallback(uint32_t reg, uint32_t regNum);
  static void sensorSendCallback(uint8_t* data, uint32_t size);
  static void delayCallback(uint16_t ms);

  static volatile char dataUpdateFlags;  // Flag: neues Yaw-/Angle-Paket angekommen

  // Roh-Offset-Werte, die beim resetReference() einmal gemerkt werden:
  static int16_t refRollRaw;
  static int16_t refPitchRaw;

  // Letzte (korrigierten) Ergebnisse
  static float roll;
  static float pitch;
  static float yaw;

  HardwareSerial& imuSerial;
  int rx, tx;
  int s0, s1;
  int channel;
  Xplorer74HC4052 mux;
  bool initialized;
};

#endif // XPLORER_WT901_H
