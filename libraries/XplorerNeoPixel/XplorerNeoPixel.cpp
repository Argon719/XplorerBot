#include "XplorerNeoPixel.h"

XplorerNeoPixel::XplorerNeoPixel(uint8_t pin, uint8_t numPixels)
    : pixels(numPixels, pin, NEO_GRB + NEO_KHZ800), numPixels(numPixels) {}

void XplorerNeoPixel::begin() {
    pixels.begin();
    pixels.show();
}

void XplorerNeoPixel::setPixel(uint8_t id, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
    if (id >= numPixels) return;
    pixels.setBrightness(brightness);
    pixels.setPixelColor(id, pixels.Color(red, green, blue));
}

void XplorerNeoPixel::clear() {
    pixels.clear();
}

void XplorerNeoPixel::show() {
    pixels.show();
}
