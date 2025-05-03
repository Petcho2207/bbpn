#ifndef BQ79600_H
#define BQ79600_H

#include <Arduino.h>

class BQ79600 {
public:
    BQ79600(HardwareSerial &serial, uint32_t baud = 1000000);
    void begin();
    void wakeUp();
    void sendPing();
    void sendCommand(uint8_t command, const uint8_t *data, uint8_t len);
    bool readResponse(uint8_t *buffer, size_t length, uint32_t timeout = 100);

private:
    HardwareSerial &uart;
    uint32_t baudRate;

    void sendFrame(const uint8_t *frame, uint8_t length);
    uint8_t calculateCRC(const uint8_t *data, uint8_t length);
};

#endif
