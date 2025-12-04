#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "BQ79600.h"
#include "BMSConfig.h"
#include "shared_data.h"
#include "rtos_config.h"

// ========== Global Objects ==========
BQ79600 *bms = nullptr;
HardwareSerial mySerial(1);
MCP2515 mcp2515(5);        // CS pin
BMSConfigManager configManager;
BQ79600config config;

// ========== Shared Variables (Protected by Mutex) ==========
volatile State currentState = Active;
volatile State wasState = Active;
Stack_data extremaList[MAX_STACKS];
uint8_t statusFault = 0x00;
float Vref[MAX_STACKS];
bool ready_balance = true;
bool balancing_complete = true;
bool balance_flag[MAX_STACKS];
uint8_t currentlyBalancingCellsCount[MAX_STACKS];
size_t currentlyBalancingCells[MAX_STACKS][MAX_CELLS];

// Configuration globals
float tempMAX;
float tempMIN;
float Vmaxlimit;
float Vminlimit;
float Vmaxlimit_pack;
float Vminlimit_pack;
float current_max;
float VoltDiffBalance;

// ========== FreeRTOS Handles ==========
// handles for control tasks
TaskHandle_t xHandle_CAN_Task = NULL;
TaskHandle_t xHandle_Config_Task = NULL;
TaskHandle_t xHandle_BMS_Task = NULL;
TaskHandle_t xHandle_Safety_Task = NULL;
TaskHandle_t xHandle_Balance_Task = NULL;

// handles for control data between tasks
QueueHandle_t xQueue_BMS_to_CAN = NULL;
QueueHandle_t xQueue_CAN_to_Config = NULL;

// handles for mutex data between tasks
SemaphoreHandle_t xMutex_BMS_Data = NULL;
SemaphoreHandle_t xMutex_Config = NULL;
SemaphoreHandle_t xMutex_SPI = NULL;
SemaphoreHandle_t xMutex_Serial = NULL;
SemaphoreHandle_t xMutex_BMS_UART = NULL;  //  Protect BMS UART access

// ========== Function Prototypes ==========
void initializeBMS();
void applyConfigToGlobals();
void AnalyzePerStackStats(Stack_data result[], uint8_t numStacks);
bool checkFaults();
bool checkHardwareFaults();
void bms_balancing(uint8_t numStacks);
bool status_balancing(uint8_t numStacks);
uint8_t voltageToHex(float Vmin);

// Interrupt handler
volatile bool interruptTriggered = false;
void IRAM_ATTR handleInterrupt() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Set interrupt flag
    interruptTriggered = true;
    
    // Notify Safety Task immediately
    vTaskNotifyGiveFromISR(xHandle_Safety_Task, &xHigherPriorityTaskWoken);
    
    // Yield if higher priority task woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ========================================================================
// ========================= CORE 0 TASKS (Communication) =================
// ========================================================================

// ========== Task 1: CAN Communication (Core 0, Priority 3) ==========
void Task_CAN(void *pvParameters) {
    struct can_frame frame;
    BMS_Data_Packet packet;
    
    // Print task start message (with mutex protection)
    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("[CAN] Task started on Core 0");
        xSemaphoreGive(xMutex_Serial);
    }
    
    // Timing variables
    TickType_t xLastCANCheck = xTaskGetTickCount();
    const TickType_t xCANCheckInterval = pdMS_TO_TICKS(50);  // Check every 50 ms
    
    while (1) {
        // ===== CAN RX: Check for incoming messages =====
        if (xSemaphoreTake(xMutex_SPI, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Read CAN message (non-blocking)
            if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
                // Check if config message (ID: 0x700)
                if (frame.can_id == 0x700) {
                    // Send to Config Task via queue
                    xQueueSend(xQueue_CAN_to_Config, &frame, 0);  // Don't block
                    
                    // Debug print (with mutex)
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.printf("[CAN] Config command received (ID: 0x%03X)\n", frame.can_id);
                        xSemaphoreGive(xMutex_Serial);
                    }
                }
            }
            xSemaphoreGive(xMutex_SPI);
        }
        
        // ===== CAN TX: Send BMS data =====
        // Check if data available in queue (non-blocking)
        if (xQueueReceive(xQueue_BMS_to_CAN, &packet, 0) == pdTRUE) {
            // Take SPI mutex for CAN transmission
            if (xSemaphoreTake(xMutex_SPI, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Prepare summary frame (high priority)
                struct can_frame txFrame;
                txFrame.can_id = 0x400 + (packet.stackIndex << 4);  // Base ID + stack offset
                txFrame.can_dlc = 8;
                
                // Pack data: [totalV(2), current(2), vmin(2), vmax(2)]
                uint16_t totalV = (uint16_t)(packet.total_voltage * 10.0f);   // 0.1V per LSB
                int16_t curr = (int16_t)(packet.current * 10.0f);             // 0.1A per LSB
                uint16_t vmin = (uint16_t)(packet.vmin * 1000.0f);            // mV
                uint16_t vmax = (uint16_t)(packet.vmax * 1000.0f);            // mV
                
                txFrame.data[0] = (totalV >> 8) & 0xFF;  // Total voltage high byte
                txFrame.data[1] = totalV & 0xFF;         // Total voltage low byte
                txFrame.data[2] = (curr >> 8) & 0xFF;    // Current high byte (signed)
                txFrame.data[3] = curr & 0xFF;           // Current low byte
                txFrame.data[4] = (vmin >> 8) & 0xFF;    // Vmin high byte
                txFrame.data[5] = vmin & 0xFF;           // Vmin low byte
                txFrame.data[6] = (vmax >> 8) & 0xFF;    // Vmax high byte
                txFrame.data[7] = vmax & 0xFF;           // Vmax low byte
                
                // Send CAN frame
                mcp2515.sendMessage(&txFrame);
                
                // Release SPI mutex
                xSemaphoreGive(xMutex_SPI);
                
                // Debug print (with mutex)
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.printf("[CAN] Sent data for Stack %d (V: %.1f, I: %.1f)\n",
                                  packet.stackIndex, packet.total_voltage, packet.current);
                    xSemaphoreGive(xMutex_Serial);
                }
            }
        }
        
        // Delay until next check interval (precise timing)
        vTaskDelayUntil(&xLastCANCheck, xCANCheckInterval);
    }
}

// ========== Task 2: Config Manager (Core 0, Priority 2) ==========
void Task_Config(void *pvParameters) {
    struct can_frame frame;
    
    // Print task start message
    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("[Config] Task started on Core 0");
        xSemaphoreGive(xMutex_Serial);
    }
    
    while (1) {
        // Wait for config command from CAN Task (blocking)
        if (xQueueReceive(xQueue_CAN_to_Config, &frame, portMAX_DELAY) == pdTRUE) {
            // Take config mutex
            if (xSemaphoreTake(xMutex_Config, pdMS_TO_TICKS(200)) == pdTRUE) {
                // Process CAN command
                bool ok = configManager.processCANCommand(
                    frame.can_id,
                    frame.data,
                    frame.can_dlc
                );
                
                if (ok) {
                    // Debug print
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("[Config] Config updated successfully");
                        xSemaphoreGive(xMutex_Serial);
                    }
                    
                    // Apply new config to global variables
                    applyConfigToGlobals();
                    
                    // Check if hardware config changed (need BMS re-init)
                    if (configManager.needsRestart()) {
                        // Notify BMS Task to re-initialize
                        xTaskNotifyGive(xHandle_BMS_Task);
                        
                        // Debug print
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("[Config] Hardware config changed, notifying BMS Task");
                            xSemaphoreGive(xMutex_Serial);
                        }
                    }
                } else {
                    // Debug print error
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("[Config] Failed to process command");
                        xSemaphoreGive(xMutex_Serial);
                    }
                }
                
                // Release config mutex
                xSemaphoreGive(xMutex_Config);
            }
        }
    }
}

// ========================================================================
// ========================= CORE 1 TASKS (BMS Control) ===================
// ========================================================================

// ========== Task 3: BMS Data Read (Core 1, Priority 5 - Critical) ==========
void Task_BMS(void *pvParameters) {

    // trigger variables for precise timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    
    // Print task start message
    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("[BMS] Task started on Core 1");
        xSemaphoreGive(xMutex_Serial);
    }
    
    while (1) {
        // Check for re-initialization request
        if (ulTaskNotifyTake(pdFALSE, 0) > 0) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) { 
                Serial.println("[BMS] Re-initializing BMS...");
                xSemaphoreGive(xMutex_Serial);
            }
            
            if (xSemaphoreTake(xMutex_Config, pdMS_TO_TICKS(200)) == pdTRUE) {
                initializeBMS();
                configManager.clearRestartFlag();
                xSemaphoreGive(xMutex_Config);
                
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
                    Serial.println("[BMS] Re-initialization complete");
                    xSemaphoreGive(xMutex_Serial);
                }
            }
        }
        
        // Take BMS data mutex
        if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(100)) == pdTRUE) {
        
            // ===== ✅ UART Mutex: SHORT duration (เฉพาะ get_data) =====
            if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(100)) == pdTRUE) {
                
                // Read BMS data (UART communication - ~10ms)
                unsigned long startTime = millis();
                bms->get_data();
                unsigned long duration = millis() - startTime;
                if (duration > 50) {
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.printf("[BMS] ⚠️ WARNING: bms->get_data() took %lu ms (expected: ~10ms)\n", 
                                    duration);
                        xSemaphoreGive(xMutex_Serial);
                    }
                }
                // ✅ ปลดล็อก UART ทันที (ไม่รอ AnalyzePerStackStats)
                xSemaphoreGive(xMutex_BMS_UART);
                
            } else {
                // Timeout: UART locked
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[BMS] Warning: UART mutex timeout");
                    xSemaphoreGive(xMutex_Serial);
                }
                xSemaphoreGive(xMutex_BMS_Data);
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            
            // ===== ✅ ออกจาก UART Mutex → ทำการวิเคราะห์ (ไม่ล็อก UART) =====
            // Analyze per-stack statistics (ไม่ต้องล็อก UART!)
            AnalyzePerStackStats(extremaList, bms->NumSegments);
            
            // ✅ Notify Safety Task: ข้อมูล BMS พร้อม (UART ว่างแล้ว)
            xTaskNotifyGive(xHandle_Safety_Task);
            
            // Send data to CAN Task (Core 0)
            for (uint8_t i = 0; i < bms->NumSegments; i++) {
                BMS_Data_Packet packet;
                packet.stackIndex = i;
                packet.vmax = extremaList[i].vmax;
                packet.vmin = extremaList[i].vmin;
                packet.vavg = extremaList[i].averageVcell;
                packet.total_voltage = extremaList[i].total_voltage;
                packet.current = extremaList[i].current;
                packet.temps[0] = bms->batteryData_pack[i].gpioTemps[0];
                packet.temps[1] = bms->batteryData_pack[i].gpioTemps[1];
                packet.dieTemp = bms->batteryData_pack[i].dieTemp;
                packet.statusFault = statusFault;
                packet.timestamp = millis();
                
                xQueueSend(xQueue_BMS_to_CAN, &packet, 0);
            }
            
            // Release BMS data mutex
            xSemaphoreGive(xMutex_BMS_Data);
        
        }
        
        // Precise delay until next read cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ========== Task 4: Safety Monitor (Core 1, Priority 5 - Critical) ==========

void Task_Safety(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(200));

    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("[Safety] Task started on Core 1");
        Serial.println("[Safety] Ready for fault monitoring");
        xSemaphoreGive(xMutex_Serial);
    }
    
    while (1) {
        uint32_t notifyValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
        
        // ===== Check Hardware Interrupt (immediate) =====
        if (interruptTriggered) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.println("[Safety] Hardware interrupt detected!");
                xSemaphoreGive(xMutex_Serial);
            }
            
            // ✅ บันทึก State ก่อน Fault
            wasState = currentState;
            
            // Immediate safety action
            digitalWrite(Relay, LOW);
            digitalWrite(Active_led, LOW);
            digitalWrite(Fault_led, HIGH);
            currentState = Fault;
            interruptTriggered = false;
            
            if (xHandle_Balance_Task != NULL) {
                vTaskSuspend(xHandle_Balance_Task);
            }
        }
        
        // ===== Check based on current state =====
        if (currentState == Fault) {
            if (xHandle_Balance_Task != NULL) {
                vTaskSuspend(xHandle_Balance_Task);
            }

            if (notifyValue > 0) {
                if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(50)) == pdTRUE) {
                    bool softwareFault = checkFaults();
                    bool hardwareFault = false;

                    hardwareFault = checkHardwareFaults();
                        
                    
                    xSemaphoreGive(xMutex_BMS_Data);
                    
                    // ✅ ถ้า Fault หาย → กลับไป State เดิม (wasState)
                    if (!softwareFault && !hardwareFault) {
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.printf("[Safety] Fault cleared! Returning to %s state\n",
                                          wasState == Balance ? "Balance" : "Active");
                            xSemaphoreGive(xMutex_Serial);
                        }
                        
                        // Clear fault in BQ79600
                        if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(100)) == pdTRUE) {
                            bms->clearFault();
                            xSemaphoreGive(xMutex_BMS_UART);
                        }
                        
                        // Recovery actions
                        digitalWrite(Relay, HIGH);
                        digitalWrite(Active_led, HIGH);
                        digitalWrite(Fault_led, LOW);
                        
                        // ✅ กลับไป State เดิม (wasState)
                        currentState = wasState;
                        statusFault = 0x00;
                        
                        // Resume Balancing Task
                        if (xHandle_Balance_Task != NULL) {
                            vTaskResume(xHandle_Balance_Task);
                        }
                    } else {
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("[Safety] Fault still present");
                            xSemaphoreGive(xMutex_Serial);
                        }
                    }
                }
            }
        } else {
            if (notifyValue > 0) {
                if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(200)) == pdTRUE) {
                    
                    bool softwareFault = checkFaults();
                    bool hardwareFault = false;
                    hardwareFault = checkHardwareFaults();
                    
                    
                    // ===== เช็คว่าต้อง Balance หรือไม่ =====
                    if (!softwareFault && !hardwareFault && currentState == Active) {
                        bool needBalance = false;
                        for (uint8_t i = 0; i < bms->NumSegments; i++) {
                            if (extremaList[i].Vdiff > VoltDiffBalance) {
                                balance_flag[i] = true;
                                needBalance = true;
                                Vref[i] = extremaList[i].vmin;
                                
                                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                                    Serial.printf("[Safety] Stack %d needs balancing (Vdiff: %.3f V > %.3f V)\n",
                                                  i, extremaList[i].Vdiff, VoltDiffBalance);
                                    Serial.printf("[Safety] Vref: %.3f V (Vmin: %.3f V)\n",
                                                  Vref[i], extremaList[i].vmin);
                                    xSemaphoreGive(xMutex_Serial);
                                }
                            } else {
                                balance_flag[i] = false;
                            }
                        }
                        
                        if (needBalance) {
                            if (!ready_balance) {

                                currentState = Balance;
                                ready_balance = true;
                            
                                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                                    Serial.println("[Safety] Entering Balance state");
                                    xSemaphoreGive(xMutex_Serial);
                                }
                            } else {// ready_balance ยัง = true (คำสั่งเก่ายังไม่ถูกประมวลผล)
                                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                                    Serial.println("[Safety] Balance command already pending, waiting...");
                                    xSemaphoreGive(xMutex_Serial);
                                }
                            }
                        }else{
                            if (ready_balance) {
                                ready_balance = false;
                                
                                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                                    Serial.println("[Safety] Vdiff normal, clearing balance request");
                                    xSemaphoreGive(xMutex_Serial);
                                }
                            }
                        }
                    }
                    
                    xSemaphoreGive(xMutex_BMS_Data);
                    
                    // ===== Safety Action (if any fault detected) =====
                    if (softwareFault || hardwareFault) {
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("[Safety] Fault detected! Entering Fault state");
                            xSemaphoreGive(xMutex_Serial);
                        }
                        
                        // ✅ บันทึก State ก่อน Fault
                        wasState = currentState;
                        
                        // Safety action
                        digitalWrite(Relay, LOW);
                        digitalWrite(Active_led, LOW);
                        digitalWrite(Fault_led, HIGH);
                        currentState = Fault;
                        
                        if (xHandle_Balance_Task != NULL) {
                            vTaskSuspend(xHandle_Balance_Task);
                        }
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    } 
}

// ========== Task 5: Cell Balancing (Core 1, Priority 4) ==========
// แทนที่ฟังก์ชัน Task_Balance ทั้งหมด (บรรทัด 477-649)
void Task_Balance(void *pvParameters) {
    TickType_t xLastBalanceCheck = xTaskGetTickCount();
    const TickType_t xBalanceInterval = pdMS_TO_TICKS(500);
    
    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("[Balance] Task started on Core 1");
        xSemaphoreGive(xMutex_Serial);
    }
    
    // ตัวแปรเก็บสถานะ
    bool isBalancing = false;
    bool wasBalancing = false;
    
    while (1) {
        // รอจนกว่า state จะเป็น Balance
        while (currentState != Balance) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // ✅ เช็คอีกครั้งหลัง delay (ป้องกัน race condition)
        if (currentState != Balance) {
            vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
            continue;
        }
        
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Balance] Entering Balance state");
            xSemaphoreGive(xMutex_Serial);
        }
        
        // Blink LED
        digitalWrite(Active_led, !digitalRead(Active_led));
        
        // อ่าน NumSegments
        uint8_t numStacks = 0;
        if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(50)) == pdTRUE) {
            numStacks = bms->NumSegments;
            xSemaphoreGive(xMutex_BMS_Data);
        }
        
        // ✅ เช็คก่อนดำเนินการต่อ
        if (currentState != Balance) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.println("[Balance] State changed after reading NumSegments, aborting");
                xSemaphoreGive(xMutex_Serial);
            }
            vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
            continue;
        }
        
        // ===== ขั้นตอนที่ 1: เช็คว่า IC กำลัง Balance อยู่หรือไม่ (Resume after Fault) =====
        if (wasBalancing && !isBalancing) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.println("[Balance] Checking if IC is still balancing (after fault)...");
                xSemaphoreGive(xMutex_Serial);
            }
            
            bool icStillBalancing = false;
            
            if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (uint8_t stackIndex = 0; stackIndex < numStacks; stackIndex++) {
                    if (!balance_flag[stackIndex]) continue;
                    
                    uint8_t addressBQ = stackIndex + 1;
                    int status = bms->checkBalance(addressBQ);
                    
                    if (status != 0x00 && status != 0x01) {
                        icStillBalancing = true;
                        
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.printf("[Balance] IC Stack %d still balancing (status: 0x%02X)\n", 
                                          stackIndex, status);
                            xSemaphoreGive(xMutex_Serial);
                        }
                    }
                }
                
                xSemaphoreGive(xMutex_BMS_UART);
            }
            
            // ✅ เช็คหลังปลดล็อก UART
            if (currentState != Balance) {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] State changed during IC status check, aborting");
                    xSemaphoreGive(xMutex_Serial);
                }
                vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
                continue;
            }
            
            if (icStillBalancing) {
                isBalancing = true;
                
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] IC resumed balancing after fault, waiting...");
                    xSemaphoreGive(xMutex_Serial);
                }
            } else {
                wasBalancing = false;
                
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] IC completed balancing during fault");
                    xSemaphoreGive(xMutex_Serial);
                }
            }
        }
        
        // ===== ขั้นตอนที่ 2: ถ้ากำลัง Balance → เช็คสถานะ =====
        if (isBalancing) {
            bool isComplete = false;
            
            if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(100)) == pdTRUE) {
                isComplete = status_balancing(numStacks);
                xSemaphoreGive(xMutex_BMS_UART);
            } else {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] Warning: UART mutex timeout (status check)");
                    xSemaphoreGive(xMutex_Serial);
                }
            }
            
            // ✅ เช็คหลังปลดล็อก UART
            if (currentState != Balance) {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] State changed during status check, aborting");
                    xSemaphoreGive(xMutex_Serial);
                }
                vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
                continue;
            }
            
            // ===== ถ้า Balance เสร็จ → กลับ Active =====
            if (isComplete) {
                if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("[Balance] Balancing completed, returning to Active");
                        xSemaphoreGive(xMutex_Serial);
                    }
                    
                    // Clear flags
                    for (size_t i = 0; i < bms->NumSegments; i++) {
                        balance_flag[i] = false;
                    }
                    
                    ready_balance = false;
                    isBalancing = false;
                    wasBalancing = false;
                    currentState = Active;
                    
                    xSemaphoreGive(xMutex_BMS_Data);
                }
            } else {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] Balancing in progress...");
                    xSemaphoreGive(xMutex_Serial);
                }
            }
        } 
        // ===== ขั้นตอนที่ 3: ถ้ายังไม่มีการ Balance → เช็คว่าต้องสั่งหรือไม่ =====
        else {
            bool shouldBalance = false;
            
            if (xSemaphoreTake(xMutex_BMS_Data, pdMS_TO_TICKS(100)) == pdTRUE) {
                shouldBalance = ready_balance;
                if (shouldBalance) {
                    ready_balance = false;
                }
                xSemaphoreGive(xMutex_BMS_Data);
            }
            
            // ✅ เช็คก่อนสั่ง Balance
            if (currentState != Balance) {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] State changed before sending command, aborting");
                    xSemaphoreGive(xMutex_Serial);
                }
                vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
                continue;
            }
            
            if (shouldBalance) {
                if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // ✅ เช็คอีกครั้งภายใน UART Mutex (critical section)
                    if (currentState == Balance) {
                        bms_balancing(numStacks);
                        isBalancing = true;
                        wasBalancing = true;
                        
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("[Balance] Balancing command sent");
                            xSemaphoreGive(xMutex_Serial);
                        }
                    } else {
                        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                            Serial.println("[Balance] State changed during UART lock, aborting");
                            xSemaphoreGive(xMutex_Serial);
                        }
                    }
                    
                    xSemaphoreGive(xMutex_BMS_UART);
                } else {
                    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                        Serial.println("[Balance] Warning: UART mutex timeout (balancing)");
                        xSemaphoreGive(xMutex_Serial);
                    }
                }
            } else {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.println("[Balance] Warning: In Balance state but no command");
                    xSemaphoreGive(xMutex_Serial);
                }
            }
        }
        
        vTaskDelayUntil(&xLastBalanceCheck, xBalanceInterval);
    }
}

// ========================================================================
// ============================= SETUP ====================================
// ========================================================================

void setup() {
    // Initialize serial (blocking until ready)
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n========================================");
    Serial.println("BMS Dual-Core FreeRTOS System Starting");
    Serial.println("========================================\n");
    
    // Configure GPIO pins
    pinMode(Fault_led, OUTPUT);
    pinMode(Active_led, OUTPUT);
    pinMode(Relay, OUTPUT);
    pinMode(Nfault, INPUT_PULLUP);
    
    // Initial GPIO states (safe defaults)
    digitalWrite(Relay, LOW);       // Relay off (safety)
    digitalWrite(Fault_led, LOW);   // Fault LED off
    digitalWrite(Active_led, LOW);  // Active LED off
    
    // Initialize MCP2515 CAN controller
    Serial.println("[Init] Configuring MCP2515...");
    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println("[Init] MCP2515 ready");
    
    // Initialize BQ79600 UART
    Serial.println("[Init] Configuring BQ79600 UART...");
    mySerial.begin(1000000, SERIAL_8N1, 16, 17);  // 1 Mbps, RX=16, TX=17
    Serial.println("[Init] UART ready");
    
    // Initialize config manager
    Serial.println("[Init] Loading configuration from NVS...");
    bool config_ok = configManager.initialize();
    if (!config_ok) {
        Serial.println("[Init] Config load failed, using defaults");
    } else {
        Serial.println("[Init] Config loaded successfully");
    }
    
    // Apply config to global variables
    applyConfigToGlobals();
    
    // Initialize BMS object
    initializeBMS();
    
    // Attach hardware interrupt (Nfault pin)
    Serial.println("[Init] Attaching hardware interrupt...");
    attachInterrupt(digitalPinToInterrupt(Nfault), handleInterrupt, FALLING);
    Serial.println("[Init] Interrupt attached");
    
    // ========== Create FreeRTOS Resources ==========
    Serial.println("\n[RTOS] Creating mutexes...");
    xMutex_BMS_Data = xSemaphoreCreateMutex();
    xMutex_Config = xSemaphoreCreateMutex();
    xMutex_SPI = xSemaphoreCreateMutex();
    xMutex_Serial = xSemaphoreCreateMutex();
    xMutex_BMS_UART = xSemaphoreCreateMutex();
    Serial.println("[RTOS] Mutexes created");
    
    Serial.println("[RTOS] Creating queues...");
    xQueue_BMS_to_CAN = xQueueCreate(10, sizeof(BMS_Data_Packet));     // 10 packets buffer FIFO
    xQueue_CAN_to_Config = xQueueCreate(5, sizeof(struct can_frame));  // 5 frames buffer FIFO
    Serial.println("[RTOS] Queues created");
    
    // ========== Create Tasks (Core 0: Communication) ==========
    Serial.println("\n[RTOS] Creating Core 0 tasks (Communication)...");
    
    xTaskCreatePinnedToCore(
        Task_CAN,                    // Task function
        "CAN_Task",                  // Task name
        4096,                        // Stack size (bytes)
        NULL,                        // Parameters
        3,                           // Priority (0-5, higher = more important)
        &xHandle_CAN_Task,           // Task handle
        0                            // Core 0
    );
    Serial.println("[RTOS]   - CAN_Task created (Core 0, Priority 3)");
    
    xTaskCreatePinnedToCore(
        Task_Config,
        "Config_Task",
        4096,
        NULL,
        2,
        &xHandle_Config_Task,
        0                            // Core 0
    );
    Serial.println("[RTOS]   - Config_Task created (Core 0, Priority 2)");
    
    // ========== Create Tasks (Core 1: BMS Control) ==========
    Serial.println("\n[RTOS] Creating Core 1 tasks (BMS Control)...");
    
    xTaskCreatePinnedToCore(
        Task_BMS,
        "BMS_Task",
        8192,                        // Larger stack (BQ79600 has lots of data)
        NULL,
        5,                           // Critical priority
        &xHandle_BMS_Task,
        1                            // Core 1
    );
    Serial.println("[RTOS]   - BMS_Task created (Core 1, Priority 5)");
    
    xTaskCreatePinnedToCore(
        Task_Safety,
        "Safety_Task",
        4096,
        NULL,
        5,                           // Critical priority
        &xHandle_Safety_Task,
        1                            // Core 1
    );
    Serial.println("[RTOS]   - Safety_Task created (Core 1, Priority 5)");
    
    xTaskCreatePinnedToCore(
        Task_Balance,
        "Balance_Task",
        4096,
        NULL,
        4,
        &xHandle_Balance_Task,
        1                            // Core 1
    );
    Serial.println("[RTOS]   - Balance_Task created (Core 1, Priority 4)");
    
    Serial.println("\n========================================");
    Serial.println("System Initialization Complete");
    Serial.println("FreeRTOS Scheduler Starting...");
    Serial.println("========================================\n");
    
    // Enable relay (system ready)
    digitalWrite(Relay, HIGH);
    digitalWrite(Active_led, HIGH);
}

void loop() {
    // Arduino loop() is no longer used!
    // FreeRTOS scheduler handles all tasks
    // Delete loop task to free resources
    vTaskDelete(NULL);
}

// ========================================================================
// ======================== HELPER FUNCTIONS ==============================
// ========================================================================

void initializeBMS() {
    Serial.println("[BMS] Creating BMS object...");
    
    // Delete old BMS object if exists
    if (bms != nullptr) {
        delete bms;
        bms = nullptr;
    }
    
    // Create new BMS object with current config
    bms = new BQ79600(mySerial, 1000000, 17, config);
    
    bool start = false;
    
    // Retry initialization until success
    while (!start) {
        delay(1000); 
        digitalWrite(Fault_led, HIGH);   // Blink LEDs during init
        digitalWrite(Active_led, HIGH);
        
        bool init = bms->initialize();
        
        if (init) {
            Serial.println("[BMS] BQ79600 initialized successfully");
            start = true;
        }
        
        digitalWrite(Fault_led, LOW);
        digitalWrite(Active_led, LOW);
        
    }
    
    delay(50);
    bms->clearFault();  // Clear any lingering faults
    Serial.println("[BMS] BMS ready\n");
}

void applyConfigToGlobals() {
    Serial.println("\n--- Applying Configuration ---");
    
    // Get config from manager
    const HardwareConfig& hwConfig = configManager.getHardwareConfig();
    const ProtectionLimits& protLimits = configManager.getProtectionLimits();
    
    // Apply hardware config
    config.num_segments = hwConfig.num_segments;
    config.num_cells_series = hwConfig.num_cells_series;
    config.num_thermistors = hwConfig.num_thermistors;
    config.shunt_resistance = hwConfig.shunt_resistance;
    
    Serial.println("Hardware Config:");
    Serial.printf("  Segments      : %d\n", config.num_segments);
    Serial.printf("  Cells/Series  : %d\n", config.num_cells_series);
    Serial.printf("  Thermistors   : %d\n", config.num_thermistors);
    Serial.printf("  Shunt R       : %.3f Ohm\n", config.shunt_resistance);
    
    // Apply protection limits
    tempMAX = protLimits.temp_max;
    tempMIN = protLimits.temp_min;
    Vmaxlimit = protLimits.cell_v_max;
    Vminlimit = protLimits.cell_v_min;
    Vmaxlimit_pack = protLimits.pack_v_max;
    Vminlimit_pack = protLimits.pack_v_min;
    current_max = protLimits.current_max;
    VoltDiffBalance = protLimits.volt_diff_balance;
    
    Serial.println("\nProtection Limits:");
    Serial.printf("  Temp Max      : %.1f C\n", tempMAX);
    Serial.printf("  Temp Min      : %.1f C\n", tempMIN);
    Serial.printf("  Cell V Max    : %.3f V\n", Vmaxlimit);
    Serial.printf("  Cell V Min    : %.3f V\n", Vminlimit);
    Serial.printf("  Pack V Max    : %.1f V\n", Vmaxlimit_pack);
    Serial.printf("  Pack V Min    : %.1f V\n", Vminlimit_pack);
    Serial.printf("  Current Max   : %.1f A\n", current_max);
    Serial.printf("  Balance Diff  : %.3f V\n", VoltDiffBalance);
    Serial.println("------------------------------\n");
}

void AnalyzePerStackStats(Stack_data result[], uint8_t numStacks) {
    // Loop through all stacks
    for (uint8_t stackIndex = 0; stackIndex < numStacks; stackIndex++) {
        const StackData& stack = bms->batteryData_pack[stackIndex];
        Stack_data& extrema = result[stackIndex];
        
        // Reset values for this stack
        extrema.vmin = 999.0f;
        extrema.vmax = 0.0f;
        extrema.vminCellsCount = 0;
        extrema.vmaxCellsCount = 0;
        
        float totalVoltage = 0.0f;
        
        // Find vmin, vmax for all cells in this stack
        for (uint8_t i = 0; i < stack.numCells; i++) {
            float v = stack.cells[i].voltage;
            totalVoltage += v;
            
            // Check for new minimum voltage
            if (v < extrema.vmin) {
                extrema.vmin = v;                           // Update vmin
                extrema.vminCellsCount = 0;                 // Reset count
                extrema.vminCells[extrema.vminCellsCount++] = i;  // Store cell index
            } 
            // Check if equal to current vmin (multiple cells)
            else if (v == extrema.vmin && extrema.vminCellsCount < MAX_CELLS) {
                extrema.vminCells[extrema.vminCellsCount++] = i;
            }
            
            // Check for new maximum voltage
            if (v > extrema.vmax) {
                extrema.vmax = v;
                extrema.vmaxCellsCount = 0;
                extrema.vmaxCells[extrema.vmaxCellsCount++] = i;
            }
            // Check if equal to current vmax
            else if (v == extrema.vmax && extrema.vmaxCellsCount < MAX_CELLS) {
                extrema.vmaxCells[extrema.vmaxCellsCount++] = i;
            }
        }
        
        // Calculate stack statistics
        extrema.total_voltage = totalVoltage;
        extrema.averageVcell = totalVoltage / stack.numCells;
        extrema.Vdiff = extrema.vmax - extrema.vmin;
        extrema.current = 0.0f;  // Will be calculated below
        
        // Debug print (with mutex protection)
                
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            
            // ✅ หัวข้อ Stack
            Serial.printf("\n=== Stack %d ===\n", stackIndex);
            Serial.printf("Vmin: %.3f V [cells:", extrema.vmin);
            for (uint8_t idx = 0; idx < extrema.vminCellsCount; idx++) {
                Serial.printf(" %d", (int)extrema.vminCells[idx]);
            }
            Serial.printf("] | Vmax: %.3f V [cells:", extrema.vmax);
            for (uint8_t idx = 0; idx < extrema.vmaxCellsCount; idx++) {
                Serial.printf(" %d", (int)extrema.vmaxCells[idx]);
            }
            Serial.printf("]\n");
            
            // ✅ เพิ่มใหม่: ปริ้น Cell Voltage ทั้งหมด
            Serial.println("--- Cell Voltages ---");
            for (uint8_t cellIdx = 0; cellIdx < stack.numCells; cellIdx++) {
                Serial.printf("  Cell %2d: %.3f V", cellIdx, stack.cells[cellIdx].voltage);
                
                // ✅ ทำเครื่องหมาย Min/Max
                if (cellIdx == extrema.vminCells[0]) {
                    Serial.print(" ← MIN");
                }
                if (cellIdx == extrema.vmaxCells[0]) {
                    Serial.print(" ← MAX");
                }
                
                Serial.println();
            }
            Serial.println("--- temperatures ---");
            for (uint8_t ntc = 0; ntc < stack.numTemps; ntc++) {
                Serial.printf("  NTC %2d: %.3f °C", ntc, stack.gpioTemps[ntc]);
                Serial.println();
            }
            
            // ✅ สรุป Stack Statistics
            Serial.printf("Total Voltage: %.3f V\n", extrema.total_voltage);
            Serial.printf("Avg Cell: %.3f V\n", extrema.averageVcell);
            Serial.printf("Vdiff: %.3f V\n", extrema.Vdiff);
            Serial.println("====================\n");
            
            xSemaphoreGive(xMutex_Serial);
        }
    }
    
    // Calculate current from last stack's busbar voltage
    if (numStacks > 0) {
        uint8_t lastStackIndex = 0;
        float busbarVolt = bms->batteryData_pack[lastStackIndex].busbarVolt;  // mV
        
        // Current = Voltage / Resistance (I = V / R)
        // busbarVolt: mV, shunt_resistance: mOhm -> Current: A
        float current = busbarVolt / config.shunt_resistance;
        
        // Copy current to all stacks
        for (uint8_t i = 0; i < numStacks; i++) {
            result[i].current = current;
        }
        
        // Debug print current calculation
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("\n=== Current Calculation ===");
            Serial.printf("Shunt R: %.6f mOhm\n", config.shunt_resistance);
            Serial.printf("Busbar Voltage (Stack %d): %.3f mV\n", lastStackIndex, busbarVolt);
            Serial.printf("Current: %.3f A\n", current);
            Serial.println("===========================\n");
            xSemaphoreGive(xMutex_Serial);
        }
    }
}

bool checkFaults() {
    // Check interrupt flag first
    /* if (interruptTriggered) {
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Fault] Interrupt triggered!");
            xSemaphoreGive(xMutex_Serial);
        }
        interruptTriggered = false;
        return true;
    } */
    
    // Check all stacks for faults
    for (uint8_t i = 0; i < bms->NumSegments; i++) {
        // Overcurrent check
        if (extremaList[i].current >= current_max) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[Fault] Stack %d: Overcurrent (%.1f A >= %.1f A)\n",
                              i, extremaList[i].current, current_max);
                xSemaphoreGive(xMutex_Serial);
            }
            return true;
        }
        
        // Cell overvoltage check
        if (extremaList[i].vmax > Vmaxlimit) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[Fault] Stack %d: Cell OV (%.3f V > %.3f V)\n",
                              i, extremaList[i].vmax, Vmaxlimit);
                xSemaphoreGive(xMutex_Serial);
            }
            return true;
        }
        
        // Cell undervoltage check
        if (extremaList[i].vmin < Vminlimit) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[Fault] Stack %d: Cell UV (%.3f V < %.3f V)\n",
                              i, extremaList[i].vmin, Vminlimit);
                xSemaphoreGive(xMutex_Serial);
            }
            return true;
        }
        
        // Pack overvoltage check
        if (extremaList[i].total_voltage > Vmaxlimit_pack) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[Fault] Stack %d: Pack OV (%.1f V > %.1f V)\n",
                              i, extremaList[i].total_voltage, Vmaxlimit_pack);
                xSemaphoreGive(xMutex_Serial);
            }
            return true;
        }
        
        // Pack undervoltage check
        if (extremaList[i].total_voltage < Vminlimit_pack) {
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[Fault] Stack %d: Pack UV (%.1f V < %.1f V)\n",
                              i, extremaList[i].total_voltage, Vminlimit_pack);
                xSemaphoreGive(xMutex_Serial);
            }
            return true;
        }
        
        // Temperature checks (all thermistors)
        for (uint8_t t = 0; t < bms->NumThermistors; t++) {
            float temp = bms->batteryData_pack[i].gpioTemps[t];
            
            // Overtemperature check
            if (temp >= tempMAX) {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.printf("[Fault] Stack %d, Sensor %d: Over temp (%.1f C >= %.1f C)\n",
                                  i, t, temp, tempMAX);
                    xSemaphoreGive(xMutex_Serial);
                }
                return true;
            }
            
            // Undertemperature check
            if (temp <= tempMIN) {
                if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                    Serial.printf("[Fault] Stack %d, Sensor %d: Under temp (%.1f C <= %.1f C)\n",
                                  i, t, temp, tempMIN);
                    xSemaphoreGive(xMutex_Serial);
                }
                return true;
            }
        }
    }
    
    return false;  // No faults detected
}

bool checkHardwareFaults() {
    if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println("\n=== Checking Hardware Faults ===");
        xSemaphoreGive(xMutex_Serial);
    }
    
    uint8_t stackFaults[MAX_STACKS];
    bool anyHardwareFault = false;
    uint8_t hardwareStatusFault = 0x00;
    uint8_t bridgeFault = 0x00;
    
    // ===== ✅ ล็อก UART สั้นๆ (เฉพาะการอ่าน) =====
    if (xSemaphoreTake(xMutex_BMS_UART, pdMS_TO_TICKS(300)) == pdTRUE) {
        
        // ✅ Debug: เข้า UART Mutex สำเร็จ
        /* if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Safety] ✅ UART Mutex acquired successfully");
            xSemaphoreGive(xMutex_Serial);
        } */
        
        // อ่าน Fault จาก Base Devices (BQ79612) - ~6ms
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Safety] Calling bms->Fault_Summary()...");
            xSemaphoreGive(xMutex_Serial);
        }
        
        bms->Fault_Summary(stackFaults);
        
        /* if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Safety] ✅ bms->Fault_Summary() completed");
            xSemaphoreGive(xMutex_Serial);
        } */
        
        // อ่าน Fault จาก Bridge Device (BQ79600) - ~2ms
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Safety] Calling bms->FaultM_Summary()...");
            xSemaphoreGive(xMutex_Serial);
        }
        
        bridgeFault = bms->FaultM_Summary();
        
        // ✅ Debug: แสดง bridgeFault value
        /* if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.printf("[Safety] ✅ bms->FaultM_Summary() returned: 0x%02X\n", bridgeFault);
            Serial.println("[Safety] checkHardwareFaults completed");
            xSemaphoreGive(xMutex_Serial);
        } */
        
        // ← ปลดล็อก UART (รวม ~8ms)
        xSemaphoreGive(xMutex_BMS_UART);
        
    } else {
        // ✅ Timeout handling: อย่าให้ System crash
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("[Safety] ⚠️ WARNING: UART mutex timeout (checkHardwareFaults)");
            Serial.println("[Safety]   → Task_BMS is holding UART mutex");
            Serial.println("[Safety]   ⏭️ Skipping hardware fault check this cycle");
            Serial.printf("[Safety]   → Timeout was waiting for: 300 ms\n");
            xSemaphoreGive(xMutex_Serial);
        }
        
        // ✅ Return false (ถือว่าไม่มี fault - ปลอดภัย)
        return false;
    }
    
    // ===== ✅ วิเคราะห์ผลลัพธ์ (นอก UART Mutex) =====
    for (uint8_t physicalIdx = 0; physicalIdx < bms->NumSegments; physicalIdx++) {
        uint8_t logicalIdx = (bms->NumSegments - 1) - physicalIdx;
        
        if (stackFaults[physicalIdx] != 0x00) {
            anyHardwareFault = true;
            
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("[HW Fault] Stack %d: 0x%02X\n", logicalIdx, stackFaults[physicalIdx]);
                
                if (stackFaults[physicalIdx] & 0x80) Serial.println("  - FAULT_PROT");
                if (stackFaults[physicalIdx] & 0x40) Serial.println("  - FAULT_COMP_ADC");
                if (stackFaults[physicalIdx] & 0x20) Serial.println("  - FAULT_OTP");
                if (stackFaults[physicalIdx] & 0x10) Serial.println("  - FAULT_COMM");
                if (stackFaults[physicalIdx] & 0x08) Serial.println("  - FAULT_OTUT");
                if (stackFaults[physicalIdx] & 0x04) Serial.println("  - FAULT_OVUV");
                if (stackFaults[physicalIdx] & 0x02) Serial.println("  - FAULT_SYS");
                if (stackFaults[physicalIdx] & 0x01) Serial.println("  - FAULT_PWR");
                
                xSemaphoreGive(xMutex_Serial);
            }
            
            hardwareStatusFault |= 0x80;
        }
    }
    
    // Check Bridge Device Fault
    if (bridgeFault != 0x00 && bridgeFault != 0xFF) {
        anyHardwareFault = true;
        
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.printf("[HW Fault] Bridge: 0x%02X\n", bridgeFault);
            
            if (bridgeFault & 0x08) Serial.println("  - FAULT_COMM");
            if (bridgeFault & 0x04) Serial.println("  - FAULT_REG");
            if (bridgeFault & 0x02) Serial.println("  - FAULT_SYS");
            if (bridgeFault & 0x01) Serial.println("  - FAULT_PWR");
            
            xSemaphoreGive(xMutex_Serial);
        }
        
        hardwareStatusFault |= 0x80;
    }
    
    if (anyHardwareFault) {
        statusFault = hardwareStatusFault;
        
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.printf("Hardware statusFault: 0x%02X\n", statusFault);
            Serial.println("===================================\n");
            xSemaphoreGive(xMutex_Serial);
        }
    }
    
    return anyHardwareFault;
}

bool status_balancing(uint8_t numStacks) {
    int result[MAX_STACKS];
    bool allCompleted = true;
    
    // Check balancing status for all stacks
    for (uint8_t stackIndex = 0; stackIndex < numStacks; ++stackIndex) {
        uint8_t addressBQ = stackIndex + 1;
        result[stackIndex] = bms->checkBalance(addressBQ);
        
        // Decode status
        if (result[stackIndex] == 0x01 || result[stackIndex] == 0x00) {
            // Completed or idle
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("Stack %d: Balancing completed (0x%02X)\n", 
                              stackIndex, result[stackIndex]);
                xSemaphoreGive(xMutex_Serial);
            }
        } else {
            // Still in progress or error
            allCompleted = false;
            
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("Stack %d: Status 0x%02X - ", stackIndex, result[stackIndex]);
                
                // Decode status bits
                if (result[stackIndex] & 0x80) Serial.print("Invalid CB setting ");
                if (result[stackIndex] & 0x40) Serial.print("Temp warning ");
                if (result[stackIndex] & 0x20) Serial.print("Paused ");
                if (result[stackIndex] & 0x10) Serial.print("Module balancing ");
                if (result[stackIndex] & 0x08) Serial.print("Active balancing ");
                if (result[stackIndex] & 0x04) Serial.print("Fault ");
                if (result[stackIndex] & 0x02) Serial.print("Module complete ");
                
                Serial.println();
                xSemaphoreGive(xMutex_Serial);
            }
        }
    }
    
    return allCompleted;
}

uint8_t voltageToHex(float Vmin) {
    // Clamp to valid range [2.45V - 4.00V]
    if (Vmin < 2.45f) Vmin = 2.45f;
    if (Vmin > 4.00f) Vmin = 4.00f;
    
    // Calculate step (0.025V per step, starting from 2.45V)
    int step = round((Vmin - 2.45f) / 0.025f) + 1;
    
    // Clamp step [1-63]
    if (step < 1) step = 1;
    if (step > 63) step = 63;
    
    // Convert back to voltage
    float voltage_out = 2.45f + (step - 1) * 0.025f;
    
    return (uint8_t)step;
}
void bms_balancing(uint8_t numStacks) {
    // Loop through all stacks
    for (uint8_t stackIndex = 0; stackIndex < numStacks; stackIndex++) {
        // Skip if this stack doesn't need balancing
        if (!balance_flag[stackIndex]) continue;
        
        const StackData& stack = bms->batteryData_pack[stackIndex];
        
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.println("===============================");
            Serial.printf("Balancing Stack %d (Address: %d)\n", stackIndex, stackIndex + 1);
            Serial.printf("Vmin: %.3f V | Vmax: %.3f V | Vdiff: %.3f V\n",
                          extremaList[stackIndex].vmin, extremaList[stackIndex].vmax, 
                          extremaList[stackIndex].Vdiff);
            xSemaphoreGive(xMutex_Serial);
        }
        
        // Find cells that need balancing (voltage >= Vref)
        size_t overVoltageCells[MAX_CELLS];
        uint8_t overVoltageCellsCount = 0;
        
        for (uint8_t j = 0; j < stack.numCells; j++) {
            if (stack.cells[j].voltage >= Vref[stackIndex]) {
                overVoltageCells[overVoltageCellsCount++] = j;
            }
        }
        
        // Bubble sort (descending order: highest voltage first)
        for (uint8_t a = 0; a < overVoltageCellsCount - 1; a++) {
            for (uint8_t b = 0; b < overVoltageCellsCount - a - 1; b++) {
                if (stack.cells[overVoltageCells[b]].voltage < 
                    stack.cells[overVoltageCells[b + 1]].voltage) {
                    // Swap
                    size_t temp = overVoltageCells[b];
                    overVoltageCells[b] = overVoltageCells[b + 1];
                    overVoltageCells[b + 1] = temp;
                }
            }
        }
        
        // Filter out adjacent cells (can't balance adjacent cells simultaneously)
        size_t filteredCells[MAX_CELLS];
        uint8_t filteredCount = 0;
        
        for (uint8_t j = 0; j < overVoltageCellsCount; j++) {
            size_t current_cell = overVoltageCells[j];
            bool isAdjacent = false;
            
            // Check if adjacent to already selected cell
            for (uint8_t k = 0; k < filteredCount; k++) {
                int diff = (int)current_cell - (int)filteredCells[k];
                if (diff >= -1 && diff <= 1) {  // Adjacent (±1)
                    isAdjacent = true;
                    break;
                }
            }
            
            // Add if not adjacent
            if (!isAdjacent && filteredCount < MAX_CELLS) {
                filteredCells[filteredCount++] = current_cell;
            }
        }
        
        // Debug print selected cells
        if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.print("Selected Cells: ");
            for (uint8_t idx = 0; idx < filteredCount; idx++) {
                Serial.printf("%d (%.3fV), ", filteredCells[idx], 
                              stack.cells[filteredCells[idx]].voltage);
            }
            Serial.println();
            Serial.println("------------------------------------------------");
            xSemaphoreGive(xMutex_Serial);
        }
        
        // Execute balancing if cells selected
        if (filteredCount > 0) {
            // Convert Vref to hex threshold
            uint8_t hexThreshold = voltageToHex(Vref[stackIndex]);
            uint8_t addressBQ = stackIndex + 1;
            
            // Store balancing cells
            for (uint8_t idx = 0; idx < filteredCount; idx++) {
                currentlyBalancingCells[stackIndex][idx] = filteredCells[idx];
            }
            currentlyBalancingCellsCount[stackIndex] = filteredCount;
            
            if (xSemaphoreTake(xMutex_Serial, pdMS_TO_TICKS(50)) == pdTRUE) {
                Serial.printf("Sending to BQ79600 Address: %d\n", addressBQ);
                Serial.printf("Vref: %.3f V -> Hex: 0x%02X\n", Vref[stackIndex], hexThreshold);
                xSemaphoreGive(xMutex_Serial);
            }
            
            // Call BMS balancing function
            bms->BalanceCells(1,  // Auto mode
                             addressBQ, 
                             currentlyBalancingCells[stackIndex],
                             currentlyBalancingCellsCount[stackIndex],
                             hexThreshold);
        }
    }
}
