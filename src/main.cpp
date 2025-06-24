#include "BQ79600.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <SPI.h>
#include <mcp2515.h>

#define LED1 21
#define LED2 22
#define Relay 13
#define Nfault 4
enum State
{
    Main,
    Fault,
};
volatile State currentState = Main;
// Structure Definitions
struct StackVoltageExtrema {
    float vmin;
    float vmax;
    float averageVcell; 
    std::vector<size_t> vminCells;
    std::vector<size_t> vmaxCells;
};
int Auto = 1; 
int Manual = 0;
struct BmsDataFrame {
    float voltages[6][10];
    float gpioTemps[6][2];
    float dieTemps[6];
};


// Globals
BQ79600config bqConfig;
std::vector<std::vector<size_t>> currentlyBalancingCells;  // [stack] = list of balancing cell index
std::vector<StackVoltageExtrema> allStackExtrema;
SemaphoreHandle_t balanceMutex;  
SemaphoreHandle_t dataMutex;
BmsDataFrame dataBuffer;
BQ79600config config = {
    10,      // num_cells_series
    2,       // num_thermistors
    1,       // num_segments
    0.001f   // shunt_resistance
};
const float VoltDiffBalance = 0.03; // Threshold for balancing
const float VoltDiff_StopBalance = 0.01; // Threshold for stop balancing

HardwareSerial mySerial(1);
BQ79600 bms(mySerial, 1000000, 17, config);
MCP2515 mcp2515(5); // CS pin

EventGroupHandle_t bmsEventGroup;
#define BIT_BALANCE_REQUIRED (1 << 0)

uint8_t voltageToHex(float Vmin);
std::vector<StackVoltageExtrema> AnalyzePerStackStats(const std::vector<StackData>&);
// ----- interrupt --------------//
volatile bool interruptTriggered = false;
void IRAM_ATTR handleInterrupt() {
    interruptTriggered = true;
}

void FSMTask(void* pv) {
    for (;;) {
    switch (currentState) {
        case Main: checkStatus(); break;
        case Fault: handleFault(); break;
      //case Sleep: handleSleep(); break;
    }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// Task: Read BQ79600 Data
void GetdataTask(void *pv) {
    for (;;) {
        bms.ReadVoltCellandTemp();
        auto extremaList = AnalyzePerStackStats(bms.batteryData_pack);
        bool cheakbalance ;

            // Critical section: write to shared memory
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            allStackExtrema = extremaList;   // ✅ ปลอดภัยเพราะอยู่ใน mutex

            for (int s = 0; s < bms.NumSegments; s++) {
                if (s >= bms.batteryData_pack.size()) break;
                StackData& stack = bms.batteryData_pack[s];

                for (int c = 0; c < bms.NumCellsSeries; c++) {
                    if (c >= stack.cells.size()) break;
                        dataBuffer.voltages[s][c] = stack.cells[c].voltage;
                    } 
                    dataBuffer.dieTemps[s] = stack.dieTemp;

                    if (stack.gpioTemps.size() > 0)
                    dataBuffer.gpioTemps[s][0] = stack.gpioTemps[0];
                    if (stack.gpioTemps.size() > 1)
                    dataBuffer.gpioTemps[s][1] = stack.gpioTemps[1];
            }

            xSemaphoreGive(dataMutex);
        }
        cheakbalance = bms.cheakBalance();
        if(!cheakbalance) {
            
         // Set balancing event flags
        for (size_t i = 0; i < extremaList.size(); ++i) {
            
            float Vdiff = extremaList[i].vmax - extremaList[i].vmin;
            Serial.printf("Stack %d Voltage Diff: %.2f V\n", (int)i, Vdiff);
            if (Vdiff >= VoltDiffBalance) {
                
                xEventGroupSetBits(bmsEventGroup, BIT_BALANCE_REQUIRED | (i << 1));
                
            }
            else{
                Serial.printf("No balancing required for stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                (int)i, extremaList[i].vmin, extremaList[i].vmax, Vdiff);
            }
        } 
    }else{

        if (xSemaphoreTake(balanceMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        for (int s = 0; s < currentlyBalancingCells.size(); ++s) {
                            for (int idx : currentlyBalancingCells[s]) {
                                float v = bms.batteryData_pack[s].cells[idx].voltage;
                                Serial.printf("[Realtime] Balancing: Stack %d, Cell %d, Voltage: %.3f V\n", s, idx, v);
                            }
                        }
                        xSemaphoreGive(balanceMutex);
                }
    } 

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void SendCanTask(void *pv) {
    const uint8_t fragLens[3] = {8, 8, 7};
    for (;;) {
        Serial.println("Sending CAN data...");
        BmsDataFrame localBuffer;

        // Step 1: Critical section – copy data
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(&localBuffer, &dataBuffer, sizeof(BmsDataFrame));
            xSemaphoreGive(dataMutex);
        } else {
            Serial.println("Failed to take dataMutex!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Step 2: ใช้ localBuffer ปลอดภัย
        for (int stackIndex = 0; stackIndex < config.num_segments; stackIndex++) {
            uint8_t raw[23];
            int idx = 0;

            for (int i = 0; i < config.num_cells_series; i++) {
                uint16_t v = (uint16_t)(localBuffer.voltages[stackIndex][i] * 1000.0f);
                raw[idx++] = (v >> 8) & 0xFF;
                raw[idx++] = v & 0xFF;
            }

            raw[idx++] = (uint8_t)((int8_t)localBuffer.dieTemps[stackIndex]);
            raw[idx++] = (uint8_t)((int8_t)localBuffer.gpioTemps[stackIndex][0]);
            raw[idx++] = (uint8_t)((int8_t)localBuffer.gpioTemps[stackIndex][1]);

            int offset = 0;
            for (uint8_t frag = 0; frag < 3; frag++) {
                struct can_frame frame;
                frame.can_id  = 0x300 + stackIndex * 4 + frag;
                frame.can_dlc = fragLens[frag];
                memcpy(frame.data, &raw[offset], fragLens[frag]);
                mcp2515.sendMessage(&frame);
                Serial.printf("Sent stack %d frag %d (%d bytes)\n", stackIndex, frag, fragLens[frag]);
                offset += fragLens[frag];
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


void balanceTask(void *pv) {
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(bmsEventGroup, BIT_BALANCE_REQUIRED, pdTRUE, pdFALSE, portMAX_DELAY);
        if(currentState == Main) {
        if (bits & BIT_BALANCE_REQUIRED) {
            for (size_t i = 0; i < bms.batteryData_pack.size(); ++i) {
                const StackData& stack = bms.batteryData_pack[i];
                const StackVoltageExtrema& extrema = allStackExtrema[i];

                float diff = extrema.vmax - extrema.vmin;

                if (diff >= VoltDiffBalance) {
                    float overVoltageThreshold = extrema.vmin + VoltDiffBalance;

                    Serial.printf("Stack %d: Cell Voltages:\n", (int)i);
                    for (size_t j = 0; j < stack.cells.size(); ++j) {
                            Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", (int)(j+1), (int)j, stack.cells[j].voltage);
                    }
                    // === ค้นหา cell ที่แรงดันเกิน threshold นี้ ===
                    std::vector<size_t> overVoltageCells;
                    for (size_t j = 0; j < stack.cells.size(); ++j) {
                        float cellVoltage = stack.cells[j].voltage;
                        if (cellVoltage >= overVoltageThreshold) {
                            overVoltageCells.push_back(j);
                        }
                    }

                    // === เรียงจากแรงดันสูงสุดไปต่ำสุด ===
                    std::sort(overVoltageCells.begin(), overVoltageCells.end(), [&](size_t a, size_t b) {
                        return stack.cells[a].voltage > stack.cells[b].voltage;
                    });

                    // === กรอง: ห้ามเลือก cell ที่ติดกัน (index ห่าง ≤ 1) ===
                    std::vector<size_t> filteredCells;
                    for (size_t j = 0; j < overVoltageCells.size(); ++j) {
                        size_t current = overVoltageCells[j];
                        bool isAdjacent = false;

                        for (size_t k = 0; k < filteredCells.size(); ++k) {
                            if (std::abs((int)current - (int)filteredCells[k]) <= 1) {
                                isAdjacent = true;
                                break;
                            }
                        }

                        if (!isAdjacent) {
                            filteredCells.push_back(current);
                        }
                    }

                    // === แสดงผล cell ที่จะถูกบาลานซ์ ===
                    Serial.println("------------------------------------------------");
                    Serial.printf("Balancing Stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                        (int)i, extrema.vmin, extrema.vmax, diff);
                    Serial.print("Selected Cells for Balancing: ");
                    for (size_t idx : filteredCells) {
                        Serial.printf("%d (%.3f V)  ", (int)idx, stack.cells[idx].voltage);
                    }
                    
                    Serial.println();
                    Serial.println("------------------------------------------------");

                    // === เรียกฟังก์ชัน Balance ===
                    if (!filteredCells.empty()) {
                        uint8_t hexVal = voltageToHex(extrema.vmin+ VoltDiff_StopBalance);
                        if (xSemaphoreTake(balanceMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                            currentlyBalancingCells[i] = filteredCells;  // set สำหรับ stack i
                            xSemaphoreGive(balanceMutex);
                        }

                        bms.BalanceCells(Manual, i, filteredCells, extrema.vminCells[0], hexVal);
                    }
                }
            }
        }
    }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void setup() {
    Serial.begin(115200);
    while (!Serial);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(Relay,OUTPUT);
    pinMode(Nfault,INPUT);

    mySerial.begin(1000000, SERIAL_8N1, 16, 17);
    bms.initialize();

    dataMutex = xSemaphoreCreateMutex();
    bmsEventGroup = xEventGroupCreate();

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    bool start = false ;
    while (!start){
        if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "start") {
            start = true;
            Serial.println("Starting BMS...");
            delay(1000);
        }
    }
}
    
    xTaskCreate(GetdataTask, "ReadData", 4096, NULL, 1, NULL);
   // xTaskCreate(SendCanTask, "SendCAN", 8192, NULL, 1, NULL);
    xTaskCreate(balanceTask,  "Balance", 2048, NULL, 2, NULL);
    xTaskCreate(FSMTask, "FSM", 4096, NULL, 2, NULL);

    balanceMutex = xSemaphoreCreateMutex();
    currentlyBalancingCells.resize(config.num_segments);  // ถ้ามี 6  ใส่ 6
    attachInterrupt(digitalPinToInterrupt(Nfault), handleInterrupt, FALLING);

}

void loop() {
    
}

std::vector<StackVoltageExtrema> AnalyzePerStackStats(const std::vector<StackData>& batteryData_pack) {
    std::vector<StackVoltageExtrema> result;

    for (size_t stackIndex = 0; stackIndex < batteryData_pack.size(); ++stackIndex) {
        const StackData& stack = batteryData_pack[stackIndex];

        StackVoltageExtrema extrema;
        extrema.vmin = std::numeric_limits<float>::max();
        extrema.vmax = std::numeric_limits<float>::lowest();
        float totalVoltage = 0.0f;
        Serial.printf("=== Stack %d ===\n", (int)stackIndex);
        Serial.println("Cell Voltages:");

        for (size_t i = 0; i < stack.cells.size(); ++i) {
            float v = stack.cells[i].voltage;
            totalVoltage += v;
            Serial.printf("  Cell %2d: %.3f V\n", (int)i, v);

            if (v < extrema.vmin) {
                extrema.vmin = v;
                extrema.vminCells.clear();
                extrema.vminCells.push_back(i);
            } else if (v == extrema.vmin) {
                extrema.vminCells.push_back(i);
            }

            if (v > extrema.vmax) {
                extrema.vmax = v;
                extrema.vmaxCells.clear();
                extrema.vmaxCells.push_back(i);
            } else if (v == extrema.vmax) {
                extrema.vmaxCells.push_back(i);
            }
        }

        // พิมพ์ค่าเฉลี่ย
        float averageVcell = totalVoltage / stack.cells.size();
        extrema.averageVcell = averageVcell;
        Serial.printf("Average Vcell: %.3f V\n", averageVcell);

        // แสดงค่า Max / Min
        Serial.printf("Vmin: %.3f V [cells:", extrema.vmin);
        for (size_t idx : extrema.vminCells) {
            Serial.printf(" %d", (int)idx);
        }
        Serial.print(" ] | ");

        Serial.printf("Vmax: %.3f V [cells:", extrema.vmax);
        for (size_t idx : extrema.vmaxCells) {
            Serial.printf(" %d", (int)idx);
        }
        Serial.println(" ]\n");

        result.push_back(extrema);
        

        //----------------------- show CellTemp -----------------------------//
        //Serial.printf("gpioTemps size = %d\n", stack.gpioTemps.size());
        Serial.println(" ---------------- Temp ----------------- ");
        for(int CellTemp = 0 ; CellTemp < stack.gpioTemps.size(); CellTemp++) {
            if (CellTemp >= 2) break; // แสดงแค่ 2 ค่าแรก
            Serial.printf("GPIO Temp %d: %.3f C\n", CellTemp, stack.gpioTemps[CellTemp]);
        }
        Serial.printf("Die Temp %d: %.3f C\n",stackIndex ,stack.dieTemp);
        
        Serial.println(" ------------------------------------ ");
        
    }

    return result;
}

uint8_t voltageToHex(float Vmin) {
    if (Vmin < 2.45f) Vmin = 2.45f;
    if (Vmin > 4.00f) Vmin = 4.00f;
    int step = round((Vmin - 2.45f) / 0.025f) + 1;
    if (step < 1)  step = 1;
    if (step > 63) step = 63;
    return (uint8_t)step;
}

void checkStatus() {
    if(interruptTriggered) {
        currentState = Fault; 
        interruptTriggered = false;   
    }
}

void handleFault(){
    Serial.println("Entering Fault Handler...");

    while (!clearAllFaults()) {
        Serial.println("Waiting for fault to clear...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // รอ 1 วินาที
    }

    Serial.println("All faults cleared.");
    currentState = Main;
}

bool clearAllFaults() {
    bool stillFault = false;

    if (bms.checkFaultBase()) {
        bms.clearFault();
        Serial.println("Cleared base fault");
        stillFault = true;
    }

    if (bms.checkFaultBrigh()) {
        bms.clearFault();
        Serial.println("Cleared brigh fault");
        stillFault = true;
    }

    return !stillFault;  // ถ้าไม่มี fault เลย -> true
}
