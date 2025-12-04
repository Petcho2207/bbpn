#ifndef RTOS_CONFIG_H
#define RTOS_CONFIG_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ========== Task Handles ==========
// Core 0 Tasks (Communication)
extern TaskHandle_t xHandle_CAN_Task;        // CAN RX/TX task
extern TaskHandle_t xHandle_Config_Task;     // Config manager task

// Core 1 Tasks (BMS Control)
extern TaskHandle_t xHandle_BMS_Task;        // BMS data read task
extern TaskHandle_t xHandle_Safety_Task;     // Safety monitor task
extern TaskHandle_t xHandle_Balance_Task;    // Cell balancing task

// ========== Queue Handles ==========
extern QueueHandle_t xQueue_BMS_to_CAN;      // BMS data -> CAN TX (Core 1 -> Core 0)
extern QueueHandle_t xQueue_CAN_to_Config;   // CAN commands -> Config manager (Core 0 -> Core 0)

// ========== Mutex Handles ==========
extern SemaphoreHandle_t xMutex_BMS_Data;    // Protect: extremaList, statusFault, balance flags
extern SemaphoreHandle_t xMutex_Config;      // Protect: config, protection limits
extern SemaphoreHandle_t xMutex_SPI;         // Protect: MCP2515 SPI access
extern SemaphoreHandle_t xMutex_Serial;      // Protect: Serial.print() calls

#endif // RTOS_CONFIG_H