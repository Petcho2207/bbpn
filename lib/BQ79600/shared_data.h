#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include "BQ79600.h"

// Hardware pins
#define MAX_CELLS 16
#define MAX_STACKS 6
#define Fault_led 21
#define Active_led 22
#define Relay 13
#define Nfault 4
#define CAN_CS_PIN 5 
// CAN IDs
#define CAN_ID_SUMMARY_BASE  0x400
#define CAN_ID_DETAIL_BASE   0x500
#define CAN_ID_CONFIG        0x700

// BMS States
enum State {
    Active,
    Sleep,
    Balance,
    Fault
};

// Stack statistics structure
struct Stack_data {
    float vmin;                          // Minimum cell voltage in stack
    float vmax;                          // Maximum cell voltage in stack
    float Vdiff;                         // Voltage difference (vmax - vmin)
    float averageVcell;                  // Average cell voltage
    float total_voltage;                 // Total stack voltage
    float current;                       // Stack current
    size_t vminCells[MAX_CELLS];        // Array of cells with vmin
    uint8_t vminCellsCount;             // Count of vmin cells
    size_t vmaxCells[MAX_CELLS];        // Array of cells with vmax
    uint8_t vmaxCellsCount;             // Count of vmax cells
};

// BMS data packet for inter-core communication (Core 1 -> Core 0)
struct BMS_Data_Packet {
    uint8_t stackIndex;                  // Stack number (0-5)
    float vmax;                          // Max cell voltage
    float vmin;                          // Min cell voltage
    float vavg;                          // Average cell voltage
    float total_voltage;                 // Total stack voltage
    float current;                       // Current (A)
    float temps[2];                      // Temperature sensors [0,1]
    float dieTemp;                       // Die temperature
    uint8_t statusFault;                 // Fault status byte
    uint32_t timestamp;                  // Timestamp (ms)
};

// Global shared variables (protected by mutexes in FreeRTOS)
extern volatile State currentState;                                 // Current system state
extern Stack_data extremaList[MAX_STACKS];                         // Per-stack statistics
extern uint8_t statusFault;                                        // Global fault status
extern float Vref[MAX_STACKS];                                     // Reference voltage for balancing
extern bool ready_balance;                                         // Balancing ready flag
extern bool balancing_complete;                                    // Balancing complete flag
extern bool balance_flag[MAX_STACKS];                              // Per-stack balance flag
extern uint8_t currentlyBalancingCellsCount[MAX_STACKS];          // Count of cells balancing
extern size_t currentlyBalancingCells[MAX_STACKS][MAX_CELLS];     // Cells currently balancing

// Configuration globals (applied from BMSConfigManager)
extern float tempMAX;                    // Max temperature limit (C)
extern float tempMIN;                    // Min temperature limit (C)
extern float Vmaxlimit;                  // Max cell voltage limit (V)
extern float Vminlimit;                  // Min cell voltage limit (V)
extern float Vmaxlimit_pack;             // Max pack voltage limit (V)
extern float Vminlimit_pack;             // Min pack voltage limit (V)
extern float current_max;                // Max current limit (A)
extern float VoltDiffBalance;            // Voltage diff threshold for balancing (V)

// BMS objects (global access)
extern BQ79600 *bms;                     // BMS driver object (pointer for re-init)
extern BQ79600config config;             // BMS configuration struct

#endif // SHARED_DATA_H