#include "BatteryStack.h"
#include <Arduino.h> // For Serial.print etc., optional

BatteryStack::BatteryStack(const std::vector<BQ7961x*> &devices) : cells(devices) {}

void BatteryStack::readAllVoltages() {
    for (auto &cell : cells) {
        cell->readCellVoltages();
    }
}

void BatteryStack::checkStackHealth() {
    Serial.println("Checking stack health...");
    for (auto &cell : cells) {
        // ในอนาคตอาจเพิ่ม logic ตรวจค่าเฉพาะ เช่น overvoltage
        cell->readCellVoltages(); // สมมุติว่าในฟังก์ชันนี้มีการเก็บค่ามาเช็คด้วย
    }
}

void BatteryStack::autoBalance() {
    Serial.println("Running auto-balance...");
    for (auto &cell : cells) {
        // อาจต้องอ่านค่าแรงดันก่อนแล้วเลือกว่าจะ balance cell ไหน
        cell->enableBalancing({0, 2, 4}); // ตัวอย่าง: balance cell 0, 2, 4
    }
}
