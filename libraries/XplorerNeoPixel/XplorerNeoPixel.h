#ifndef XPLORER_NEOPIXEL_H
#define XPLORER_NEOPIXEL_H

#include <Adafruit_NeoPixel.h>

class XplorerNeoPixel {
public:
    XplorerNeoPixel(uint8_t pin, uint8_t numPixels);
    void begin();
    void setPixel(uint8_t id, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);
    void clear();  // Alle LEDs ausschalten
    void show();   // LEDs aktualisieren

private:
    Adafruit_NeoPixel pixels;
    uint8_t numPixels;
};

#endif
