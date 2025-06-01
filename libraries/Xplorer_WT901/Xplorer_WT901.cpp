#include "Xplorer_WT901.h"

// Definition der statischen Member:
extern int16_t sReg[];  // Kommt von wit_c_sdk
volatile char Xplorer_WT901::dataUpdateFlags = 0;
int16_t      Xplorer_WT901::refRollRaw   = 0;
int16_t      Xplorer_WT901::refPitchRaw  = 0;
float        Xplorer_WT901::roll         = 0.0f;
float        Xplorer_WT901::pitch        = 0.0f;
float        Xplorer_WT901::yaw          = 0.0f;

Xplorer_WT901::Xplorer_WT901(HardwareSerial& serial,
                             int rxPin, int txPin,
                             int s0Pin, int s1Pin,
                             int muxChannel)
  : imuSerial(serial),
    rx(rxPin),
    tx(txPin),
    s0(s0Pin),
    s1(s1Pin),
    channel(muxChannel),
    mux(s1Pin, s0Pin),
    initialized(false)
{}


void Xplorer_WT901::begin() {
  // 1) MUX initialisieren und auf WT901-Kanal schalten
  mux.begin();
  mux.setChannel(channel);
  delay(2);

  // 2) UART am WT901 auf 115200 Baud starten
  imuSerial.begin(115200, SERIAL_8N1, rx, tx);

  // 3) WT901-SDK initialisieren
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(sensorSendCallback);
  WitRegisterCallBack(sensorDataCallback);
  WitDelayMsRegister(delayCallback);

  initialized = true;
}

void Xplorer_WT901::update() {
  if (!initialized) return;

  // Sicherstellen, dass MUX auf WT901 steht
  mux.setChannel(channel);

  // a) Alle empfangenen Bytes verarbeiten, bis keiner mehr da ist
  while (imuSerial.available()) {
    WitSerialDataIn(imuSerial.read());
  }

  // b) Sobald ein neues Yaw-/Winkel-Paket eingetroffen ist, sReg[...] enthält nun Rohdaten
  if (dataUpdateFlags & 0x04) {
    // Roh-Roll und Roh-Pitch liegen in sReg[Roll] und sReg[Pitch]:
    int16_t rawR = sReg[Roll];
    int16_t rawP = sReg[Pitch];
    int16_t rawY = sReg[Yaw];

    // Jetzt von Roh-Roll/-Pitch den gespeicherten Offset abziehen
    float deltaR = static_cast<float>(rawR - refRollRaw)  / 32768.0f * 180.0f;
    float deltaP = static_cast<float>(rawP - refPitchRaw) / 32768.0f * 180.0f;
    float absY   = static_cast<float>(rawY)               / 32768.0f * 180.0f;

    roll  = deltaR;
    pitch = deltaP;
    yaw   = absY;  // Yaw bleibt absolut, ohne Referenz-Offset

    dataUpdateFlags &= ~0x04;
  }
}

float Xplorer_WT901::getRoll()  const { return roll;  }
float Xplorer_WT901::getPitch() const { return pitch; }
float Xplorer_WT901::getYaw()   const { return yaw;   }

void Xplorer_WT901::sensorDataCallback(uint32_t reg, uint32_t regNum) {
  // Wird vom SDK aufgerufen, sobald eine WitReadReg-Antwort eintrifft.
  // Wir markieren „Yaw gesehen“ um zu wissen, dass wir ein vollständiges
  // Winkel-Paket erhalten haben. Yaw steht üblicherweise am Ende des Angle-Pakets.
  for (uint32_t i = 0; i < regNum; i++) {
    if (reg == Yaw) {
      dataUpdateFlags |= 0x04;
    }
    reg++;
  }
}

void Xplorer_WT901::sensorSendCallback(uint8_t* data, uint32_t size) {
  // Das SDK schreibt hier die Bytes, die an den WT901 gesendet werden sollen
  Serial0.write(data, size);
}

void Xplorer_WT901::delayCallback(uint16_t ms) {
  // Wird vom SDK aufgerufen, um Verzögerungen einzubauen
  delay(ms);
}

void Xplorer_WT901::resetReference() {
  if (!initialized) return;

  // 1) Sicherstellen, dass der MUX auf WT901 steht
  mux.setChannel(channel);
  delay(2);

  // 2) Aktuelle Roh-Register von Roll und Pitch vom WT901 anfordern
  //    (2 Register: Roll‐Adresse, Pitch‐Adresse)
  WitReadReg(Roll, 2);
  delay(10);  // Warte kurz, bis sReg[...] aktualisiert ist

  // 3) Die neuen Rohwerte aus sReg[] auslesen
  refRollRaw  = sReg[Roll];
  refPitchRaw = sReg[Pitch];
  // Yaw-Register lassen wir unberührt, wir speichern nur Roll/Pitch‐Offset

  // 4) (Optional) In die internen REF-Register des WT901 schreiben,
  //    falls der Sensor sie selbst intern verwenden soll. 
  //    Da wir die Offset‐Subtraktion aber in unserer Software selbst machen,
  //    ist das nicht zwingend nötig und kann entfallen.
  //
  //    Wollt ihr trotzdem, dass WT901 intern Referenz‐Register befüllt:
  //    WitWriteReg(KEY, KEY_UNLOCK);
  //    delay(5);
  //    WitWriteReg(REFROLL,  refRollRaw);
  //    delay(5);
  //    WitWriteReg(REFPITCH, refPitchRaw);
  //    delay(5);
  //
  //    Dann würde WT901 intern ab sofort selbst seine Winkel „relativ“ berechnen.
  //
  // In unserem Fall genügt es, refRollRaw / refPitchRaw in der Software zu subtrahieren.
}
