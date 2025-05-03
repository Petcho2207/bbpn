#ifndef BQ7961X_H
#define BQ7961X_H

#include "BQ79600.h"
#include <vector>
class BQ7961x {
public:
    BQ7961x(uint8_t address, BQ79600 &bridge);
    void ping();
    void readCellVoltages();
    void readTemperature();
    void reset();
    void enableBalancing(const std::vector<uint8_t> &cellsToBalance); // ← เพิ่มตรงนี้

private:
    uint8_t addr;
    BQ79600 &bqBridge;

    void sendStackCommand(uint8_t command, const uint8_t *data, uint8_t len);
};

#endif
