#include "BQ79600.h"

BQ79600::BQ79600(HardwareSerial &serial, uint32_t baud) : uart(serial), baudRate(baud) {}

void BQ79600::begin() {
    uart.begin(baudRate);
}

void BQ79600::wakeUp() {
    uart.write(0xFF);
    delayMicroseconds(300);
}

void BQ79600::sendPing() {
    uint8_t pingFrame[] = { 0x80, 0x01, 0x01, 0x7E }; // ตัวอย่าง frame
    sendFrame(pingFrame, sizeof(pingFrame));
}

void BQ79600::sendCommand(uint8_t command, const uint8_t *data, uint8_t len) {
    uint8_t frame[32];
    frame[0] = 0x80;               // Start byte
    frame[1] = len + 1;            // Length
    frame[2] = command;            // Command
    memcpy(&frame[3], data, len);  // Payload
    frame[3 + len] = calculateCRC(frame, 3 + len);
    sendFrame(frame, 4 + len);
}

bool BQ79600::readResponse(uint8_t *buffer, size_t length, uint32_t timeout) {
    uint32_t start = millis();
    size_t index = 0;
    while ((millis() - start) < timeout && index < length) {
        if (uart.available()) {
            buffer[index++] = uart.read();
        }
    }
    return index == length;
}

void BQ79600::sendFrame(const uint8_t *frame, uint8_t length) {
    uart.write(frame, length);
}

uint8_t BQ79600::calculateCRC(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= data[i];
    }
    return crc;
}
