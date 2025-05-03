#include "BQ79600.h"

BQ79600::BQ79600(HardwareSerial &serial, uint32_t baud , int tx_pin = {}) : uart(serial), baudRate(baud), tx_pin_(tx_pin) {}

void BQ79600::initialize() {
    //uart.begin(baudRate);
}

void BQ79600::beginUart()
{
    uart.begin(baudRate); //, SERIAL_8N1_HALF_DUPLEX);  // BQ79656 uart interface is half duplex
                                // switch from write to read fast enough
} 

void BQ79600::wakeUp() {
    uart.write(0xFF);
    delayMicroseconds(300);
}

void BQ79600::sendPing() {
    uint8_t pingFrame[] = { 0x80, 0x01, 0x01, 0x7E }; // ตัวอย่าง frame
    sendFrame(pingFrame, sizeof(pingFrame));
}


void BQ79600::sendCommandTo(uint8_t addr, uint8_t command, const uint8_t *data, uint8_t len) {
    uint8_t frame[32];
    frame[0] = addr;                // ใส่ address ที่ส่งมา
    frame[1] = len + 1;             // ความยาวของข้อมูล
    frame[2] = command;             // คำสั่ง
    memcpy(&frame[3], data, len);   // ใส่ payload
    frame[3 + len] = calculateCRC(frame, 3 + len); // คำนวณ CRC
    sendFrame(frame, 4 + len);      // ส่ง frame
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

void BQ79600::wakePing() {
     // Output a pulse of low on RX for ~2.5ms to wake chip
    uart.end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(2600);    // 2.6ms pulse
    digitalWrite(tx_pin_, HIGH);
    beginUart();
    

}