#include "XplorerAutoLights.h"

XplorerAutoLights::XplorerAutoLights(uint8_t pin, uint8_t numPixels)
    : pixels(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void XplorerAutoLights::begin() {
    pixels.begin();
    pixels.show();  // Alle LEDs ausschalten
}

void XplorerAutoLights::setBrakeLights(bool isBraking) {
    uint8_t brightness = isBraking ? BRIGHTNESS_FULL : BRIGHTNESS_DIM;
    setRedLights(brightness);
}

void XplorerAutoLights::setReverseLights(bool isReversing) {
    uint32_t color = isReversing
        ? pixels.Color(255, 255, 255)
        : pixels.Color(0, 0, 0); // Weiß oder Aus
    setReverseLightColor(color);
}

void XplorerAutoLights::toggleLeftBlinker() {
    isLeftBlinking = !isLeftBlinking;
}

void XplorerAutoLights::toggleRightBlinker() {
    isRightBlinking = !isRightBlinking;
}

void XplorerAutoLights::updateBlinkers() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        blinkState = !blinkState;

        if (isLeftBlinking) {
            setBlinkers(leftBlinkers, 2, blinkState);
        } else {
            setBlinkers(leftBlinkers, 2, false);
        }

        if (isRightBlinking) {
            setBlinkers(rightBlinkers, 2, blinkState);
        } else {
            setBlinkers(rightBlinkers, 2, false);
        }
        pixels.show();
    }
}

void XplorerAutoLights::setRedLights(uint8_t brightness) {
    for (uint8_t i = 0; i < 6; i++) {
        pixels.setPixelColor(redLights[i], pixels.Color(brightness, 0, 0));
    }
    pixels.show();
}

void XplorerAutoLights::setBlinkers(const uint8_t blinkers[], uint8_t numBlinkers, bool isBlinking) {
    uint32_t color = isBlinking
        ? pixels.Color(255, 100, 0)
        : pixels.Color(0, 0, 0); // Orange für Blinken, aus wenn nicht
    for (uint8_t i = 0; i < numBlinkers; i++) {
        pixels.setPixelColor(blinkers[i], color);
    }
}

void XplorerAutoLights::setReverseLightColor(uint32_t color) {
    for (uint8_t i = 0; i < 2; i++) {
        pixels.setPixelColor(reverseLights[i], color);
    }
    pixels.show();
}
