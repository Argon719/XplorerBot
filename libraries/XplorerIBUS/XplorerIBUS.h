#ifndef XPLORER_IBUS_H
#define XPLORER_IBUS_H

#include <Arduino.h>
#include <Stream.h>

class XplorerIBUS {
public:
    void begin(Stream& stream);
    void loop();
    uint16_t readChannel(uint8_t channelNr);

    // Funktionen für Joystick, Schalter und Knöpfe mit Invertierung
    int readJoystick(uint8_t channelNr, int minLimit, int maxLimit, int defaultValue, bool invert = false);
    bool readTwoPositionSwitch(uint8_t channelNr, bool defaultValue, bool invert = false);
    int readThreePositionSwitch(uint8_t channelNr, bool invert = false);
    int readKnob(uint8_t channelNr, int minLimit, int maxLimit, bool invert = false);

private:
    enum State {
        GET_LENGTH,
        GET_DATA,
        GET_CHKSUML,
        GET_CHKSUMH,
        DISCARD,
    };

    static const uint8_t PROTOCOL_LENGTH = 0x20;
    static const uint8_t PROTOCOL_CHANNELS = 10;
    static const uint8_t PROTOCOL_COMMAND40 = 0x40;
    static const uint8_t PROTOCOL_OVERHEAD = 3;

    State state = GET_LENGTH;
    Stream* stream = nullptr;
    uint8_t buffer[PROTOCOL_LENGTH];
    uint8_t ptr = 0;
    uint8_t len = 0;
    uint16_t channel[PROTOCOL_CHANNELS];
    uint16_t chksum = 0;
    uint8_t lchksum = 0;
    uint32_t last = 0;
};

#endif
