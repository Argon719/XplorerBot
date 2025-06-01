#include "XplorerIBUS.h"

void XplorerIBUS::begin(Stream& serialStream) {
    stream = &serialStream;
    for (uint8_t i = 0; i < PROTOCOL_CHANNELS; i++) {
        channel[i] = 0;
    }
}

void XplorerIBUS::loop() {
    while (stream->available() > 0) {
        uint32_t now = millis();
        if (now - last >= 3) {
            state = GET_LENGTH;
        }
        last = now;
        uint8_t b = stream->read();
        
        switch (state) {
            case GET_LENGTH:
                if (b > PROTOCOL_LENGTH) {
                    state = DISCARD;
                    break;
                }
                ptr = 0;
                len = b - PROTOCOL_OVERHEAD;
                chksum = 0xFFFF - b;
                state = GET_DATA;
                break;

            case GET_DATA:
                buffer[ptr++] = b;
                chksum -= b;
                if (ptr == len) {
                    state = GET_CHKSUML;
                }
                break;

            case GET_CHKSUML:
                lchksum = b;
                state = GET_CHKSUMH;
                break;

            case GET_CHKSUMH:
                if (chksum == (b << 8) + lchksum && ptr >= 18 && buffer[0] == PROTOCOL_COMMAND40) {
                    for (uint8_t i = 0; i < PROTOCOL_CHANNELS; i++) {
                        channel[i] = buffer[i * 2 + 1] | (buffer[i * 2 + 2] << 8);
                    }
                }
                state = GET_LENGTH;
                break;

            case DISCARD:
            default:
                break;
        }
    }
}

uint16_t XplorerIBUS::readChannel(uint8_t channelNr) {
    if (channelNr < PROTOCOL_CHANNELS) {
        return channel[channelNr];
    }
    return 0;
}

int XplorerIBUS::readJoystick(uint8_t channelNr, int minLimit, int maxLimit, int defaultValue, bool invert) {
    uint16_t ch = readChannel(channelNr);
    if (ch < 100) return defaultValue;
    int value = map(ch, 1000, 2000, minLimit, maxLimit);
    return invert ? (maxLimit - value + minLimit) : value;
}


bool XplorerIBUS::readTwoPositionSwitch(uint8_t channelNr, bool defaultValue, bool invert) {
    int intDefaultValue = (defaultValue) ? 0 : 100;
    int ch = readJoystick(channelNr, 0, 100, intDefaultValue);
    return invert ? (ch < 50) : (ch > 50);
}

int XplorerIBUS::readThreePositionSwitch(uint8_t channelNr, bool invert) {
    int ch = readJoystick(channelNr, 0, 100, 50);
    if (invert) {
        if (ch > 75) return -1;
        else if (ch < 25) return 1;
    } else {
        if (ch < 25) return -1;
        else if (ch > 75) return 1;
    }
    return 0;
}

int XplorerIBUS::readKnob(uint8_t channelNr, int minLimit, int maxLimit, bool invert) {
    uint16_t ch = readChannel(channelNr);
    if (ch < 100) return 0;
    int value = map(ch, 1000, 2000, minLimit, maxLimit);
    return invert ? (maxLimit - value + minLimit) : value;
}
