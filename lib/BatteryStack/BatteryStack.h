#ifndef BATTERYSTACK_H
#define BATTERYSTACK_H

#include <vector>
#include "BQ7961x.h"

class BatteryStack {
public:
    BatteryStack(const std::vector<BQ7961x*> &devices);

    void readAllVoltages();
    void checkStackHealth();     // Example: check if any voltage is too low/high
    void autoBalance();          // Start balancing based on simple logic

private:
    std::vector<BQ7961x*> cells;
};

#endif
