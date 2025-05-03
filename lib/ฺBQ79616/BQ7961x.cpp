#include "BQ7961x.h"

BQ7961x::BQ7961x(uint8_t address, BQ79600 &bridge) : addr(address), bqBridge(bridge) {}

void BQ7961x::ping() {
    uint8_t data[] = { addr };
    sendStackCommand(0x01, data, 1); // PING command
}

void BQ7961x::readCellVoltages() {
    uint8_t data[] = { addr, 0x14 }; // Register address example
    sendStackCommand(0x03, data, sizeof(data)); // READ
}

void BQ7961x::readTemperature() {
    uint8_t data[] = { addr, 0x18 }; // Register address example
    sendStackCommand(0x03, data, sizeof(data)); // READ
}

void BQ7961x::reset() {
    uint8_t data[] = { addr };
    sendStackCommand(0x0F, data, 1); // RESET command
}

void BQ7961x::sendStackCommand(uint8_t command, const uint8_t *data, uint8_t len) {
    bqBridge.sendCommandTo(addr,command, data, len);
}