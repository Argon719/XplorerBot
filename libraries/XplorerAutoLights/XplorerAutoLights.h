#ifndef XPLORER_AUTO_LIGHTS_H
#define XPLORER_AUTO_LIGHTS_H

#include <Adafruit_NeoPixel.h>

class XplorerAutoLights {
public:
    XplorerAutoLights(uint8_t pin, uint8_t numPixels);
    void begin();
    void setBrakeLights(bool isBraking);
    void setReverseLights(bool isReversing);
    void toggleLeftBlinker();
    void toggleRightBlinker();
    void updateBlinkers();

private:
    Adafruit_NeoPixel pixels;
    bool isLeftBlinking = false;
    bool isRightBlinking = false;
    bool blinkState = false;
    unsigned long previousMillis = 0;
    const long blinkInterval = 250;

    // LED-Kanalzuweisungen
    const uint8_t redLights[6]     = {0, 1, 2, 8, 7, 6};  // Rotes Licht LEDs
    const uint8_t rightBlinkers[2] = {11, 9};             // Rechte Blinker LEDs  (was {3,5})
    const uint8_t leftBlinkers[2]  = {3, 5};              // Linke Blinker LEDs   (was {11,9})
    const uint8_t reverseLights[2] = {4, 10};             // Rücklichter LEDs

    const uint8_t BRIGHTNESS_FULL = 255;
    const uint8_t BRIGHTNESS_DIM  = 50;

    void setRedLights(uint8_t brightness);
    void setBlinkers(const uint8_t blinkers[], uint8_t numBlinkers, bool isBlinking);
    void setReverseLightColor(uint32_t color);
};

#endif // XPLORER_AUTO_LIGHTS_H
