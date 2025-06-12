#include "BQ79600.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <SPI.h>
#include <mcp2515.h>

#define LED1 21
#define LED2 22
#define Relay 13
struct VoltageExtrema {
    float vmin;
    float vmax;
    size_t vminStack, vminCell;
    size_t vmaxStack, vmaxCell;
};
struct BmsDataFrame {
    float voltages[6][10];
    float gpioTemps[6][2];
    float dieTemps[6];
};
BmsDataFrame dataBuffer;
SemaphoreHandle_t dataMutex; 
BQ79600config config = {
    10,      // num_cells_series
    2,     // num_thermistors
    1,      // num_segments
    0.001f  // shunt_resistance
};
const float VoltDiffBalance = 0.3; // Threshold สำหรับ Balance
HardwareSerial mySerial(1);  // UART1 ของ ESP32
BQ79600 bms(mySerial, 1000000, 17, config);
MCP2515 mcp2515(5); // Pin CS ของ MCP2515
// ประกาศ EventGroup และ Bit Flag
// ------------------------
EventGroupHandle_t bmsEventGroup;
#define BIT_BALANCE_REQUIRED (1 << 0)  // Flag สำหรับแจ้งว่า "ควร Balance"

// ------------------------
// ประกาศ Queue
// ------------------------
QueueHandle_t voltQueue;  // ใช้ส่งค่าแรงดันระหว่าง Task

// ------------------------ read voltage and teperature from BQ79600 ------------------------//
void GetdataTask(void *pv) {

    for (;;) {
        
        bms.ReadVoltCellandTemp();                                                                  // get voltage and temperature from BQ79600
        VoltageExtrema extrema = AnalyzeVoltageStats(bms.batteryData_pack);                         // Analyze max,min voltage 
        //xQueueSend(voltQueue, &voltage, portMAX_DELAY);                                           // sent voltage to Queue
        //xQueueSend(voltQueue, &voltage, portMAX_DELAY);                                           // sent temperature to Queue
        
        float voltDiff = extrema.vmax - extrema.vmin;       // คำนวณความต่างของแรงดัน
        Serial.printf("Voltage Diff: %.2f V\n", voltDiff);  // แสดงผลความต่างแรงดัน
        // 👇 --- เช็คว่าเกิน Threshold แล้ว Set Event Flag ---
        if (voltDiff >= VoltDiffBalance) {
            xEventGroupSetBits(bmsEventGroup, BIT_BALANCE_REQUIRED);       // ⚠️ Trigger Task Balance
        }   

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int s = 0; s < 6; s++) {
                for (int c = 0; c < 10; c++) {
                    dataBuffer.voltages[s][c] =bms.batteryData_pack[s].cells[c].voltage;
                }
                dataBuffer.gpioTemps[s][0] = bms.batteryData_pack[s].gpioTemps[0];
                dataBuffer.gpioTemps[s][1] = bms.batteryData_pack[s].gpioTemps[1];
                dataBuffer.dieTemps[s]     =  bms.batteryData_pack[s].dieTemp;
            }
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100ms
    }
}

// ------------------------
// Task 2: รับค่าแรงดันจาก Queue แล้วส่งผ่าน CAN
// ------------------------
void SendCanTask(void *pv) {
    struct can_frame frame;
    frame.can_dlc = 23;    // ขนาดข้อมูล 23 bytes (CAN FD)

    for (;;) {
        for (int stackIndex = 0; stackIndex < 6; stackIndex++) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // เตรียมข้อมูลแรงดัน cell 10 ก้อนใน stack เป็น uint16_t (mV)
                for (int i = 0; i < 10; i++) {
                    uint16_t v = (uint16_t)(dataBuffer.voltages[stackIndex][i] * 1000.0);
                    frame.data[2*i] = (v >> 8) & 0xFF;      // high byte
                    frame.data[2*i + 1] = v & 0xFF;         // low byte
                }

                // dieTemp ใส่ byte 20
                frame.data[20] = (int8_t)(dataBuffer.dieTemps[stackIndex]);

                // TempGPIO1,2 ใส่ byte 21-22
                frame.data[21] = (int8_t)(dataBuffer.gpioTemps[stackIndex][0]);
                frame.data[22] = (int8_t)(dataBuffer.gpioTemps[stackIndex][1]);

                xSemaphoreGive(dataMutex);
            }

            // กำหนด CAN ID ตาม stack เพื่อแยกแยะข้อมูล
            frame.can_id = 0x300 + stackIndex;

            mcp2515.sendMessage(&frame);

            Serial.printf("Sent stack %d data: voltages 10 cells + dieTemp + GPIO temps\n", stackIndex);

            // ถ้าต้องการ delay เล็กน้อยระหว่างส่งแต่ละ stack ให้ใส่ตรงนี้
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // ส่งครบ 6 stack แล้ว delay 1 วิ ก่อนส่งรอบถัดไป
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ------------------------
// Task 3: ตรวจสอบ Event Flag และทำการ Balance
// ------------------------

void balanceTasdk(void *pv) {
    for (;;) {
        // รอจนกว่าจะมี Event Flag BIT_BALANCE_REQUIRED ถูกตั้งค่า
        EventBits_t bits = xEventGroupWaitBits(bmsEventGroup, BIT_BALANCE_REQUIRED, pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (bits & BIT_BALANCE_REQUIRED) {
            Serial.println("Balance required! Starting balance process...");

            // ทำการ Balance ที่นี่
            bms.BalanceCells(1);  // สมมุติว่ามีฟังก์ชัน BalanceCells ใน BQ79600

            Serial.println("Balance process completed.");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100ms
    }
}
uint8_t voltageToHex(float Vmin) {
    // Clamp input range
    if (Vmin < 2.45) Vmin = 2.45;
    if (Vmin > 4.00) Vmin = 4.00;

    // Calculate step value
    uint8_t step = round((Vmin - 2.45) / 0.025) + 1;

    // Clamp output to valid range: 0x01 to 0x3F (1 to 63)
    if (step < 1) step = 1;
    if (step > 63) step = 63;

    return (uint8_t)step;
}

void setup() {
    // ---------------- config Pin Mode ----------------- //
    pinMode(LED1, OUTPUT);  
    pinMode(LED2, OUTPUT);   
    pinMode(Relay, OUTPUT);

    // ---------------- config Serial ------------------ //
    Serial.begin(115200);
    mySerial.begin(1000000, SERIAL_8N1, 16, 17);
    delay(100);

    // ------------ config BQ79600 & BQ79612 ---------- //
    bms.initialize();

    // ----------------- config FreeRTOS ----------------- //
    bmsEventGroup = xEventGroupCreate();           // create  EventGroup
    voltQueue = xQueueCreate(10, sizeof(float));   // create  Queue , size of float 10
    xTaskCreate(GetdataTask, "ReadVolt", 2048, NULL, 1, NULL);  // create Task Read data
    xTaskCreate(SendCanTask, "SendCAN", 2048, NULL, 1, NULL);    // create Task Send data to CAN
    xTaskCreate(BalanceTask, "Balance", 2048, NULL, 2, NULL);

    // ----------------- config MCP2515 CAN ----------------- //
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void loop() {
    




















































    /*digitalWrite(Relay, LOW) ; // เปิดรีเลย์
    digitalWrite(LED1, LOW) ;
    digitalWrite(LED2, HIGH) ;
    delay(1000);
    digitalWrite(LED1, HIGH) ;
    digitalWrite(LED2, LOW) ;
    digitalWrite(Relay, HIGH) ; // ปิดรีเลย์
    delay(1000);*/
    /*if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "wake") {
        
            bms.wakeUp();

        } else if (command == "initialize") {
            Serial.println("Sending initialize command...");
            bms.initialize();
        } else if (command == "autoADDRESS") {
            Serial.println("Sending autoADDRESS command...");
            bms.AutoAddressing();
        }else if (command == "off") {
            Serial.println("Sending OFF command...");
            bms.offcommand();
        }else if (command == "cheakstatus") {
            Serial.println("Sending cheakstatus command...");
            bms.cheakstatus();
        }else if (command == "Test") {
            Serial.println("Sending Test command...");
            bms.Test();
        }else if (command == "ON") {
            Serial.println("Sending ON GPIO command...");
            bms.IronManON();
        }else if (command == "OFF") {
            Serial.println("Sending OFF GPIO command...");
            bms.IronManOFF();
        }else if (command == "cheakstatus1") {
            Serial.println("Sending cheakstatus1 command...");
            bms.cheakstatus1();
        }else if (command == "ManualAddress") {
            Serial.println("Sending Manual Address command...");
                bool addrOk = bms.SetManualAddress(0x01, 100);
                    if (!addrOk) {
                        Serial.println("Error: ตั้งค่า Address ให้ BQ79612 ไม่สำเร็จ!");
                    } else {
                        Serial.println("Manual Address set successfully.");
                    }
        }else if (command == "ReadVoltCell"){
            Serial.println("Sending ReadVoltCell command...");
            //while (1)
            //{
                //bms.ReadVoltCell();
                for (size_t stack = 0; stack < bms.cellDataMatrix.size(); ++stack) {
                    for (size_t cell = 0; cell < bms.cellDataMatrix[stack].size(); ++cell) {
                        Serial.print("Stack ");
                        Serial.print(stack);
                        Serial.print(" Cell ");
                        Serial.print(cell);
                        Serial.print(" Voltage: ");
                        Serial.println(bms.cellDataMatrix[stack][cell].voltage, 4); // ปริ้น 4 ตำแหน่งทศนิยม
                }
            }
                //delay(1000); // หน่วงเวลา 1 วินาที
            //}
            
            
        }else {
            Serial.println("Unknown command: " + command);
            }
    }*/

                //delay(1000); // หน่วงเวลา 1 วินาที
}

VoltageExtrema AnalyzeVoltageStats(const std::vector<StackData>& batteryData_pack) {
    VoltageExtrema result;
    result.vmin = std::numeric_limits<float>::max();
    result.vmax = std::numeric_limits<float>::lowest();
    result.vminStack = result.vmaxStack = 0;
    result.vminCell  = result.vmaxCell  = 0;

    for (size_t stackIdx = 0; stackIdx < batteryData_pack.size(); ++stackIdx) {
        const StackData& stack = batteryData_pack[stackIdx];
        if (stack.cells.empty()) continue;

        for (size_t i = 0; i < stack.cells.size(); ++i) {
            float v = stack.cells[i].voltage;

            if (v < result.vmin) {
                result.vmin = v;
                result.vminStack = stackIdx;
                result.vminCell = i;
            }
            if (v > result.vmax) {
                result.vmax = v;
                result.vmaxStack = stackIdx;
                result.vmaxCell = i;
            }
        }
    }

    // Optional: แสดงผล
    Serial.println("========== PACK SUMMARY ==========");
    Serial.printf("Overall Vmin: %.4f V at Stack %d Cell %d\n", result.vmin, result.vminStack, result.vminCell);
    Serial.printf("Overall Vmax: %.4f V at Stack %d Cell %d\n", result.vmax, result.vmaxStack, result.vmaxCell);

    return result;
}