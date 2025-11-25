#include "BQ79600.h"
#include "BMSConfig.h"
#include <SPI.h>
#include <mcp2515.h>
#define MAX_CELLS 16            // maximum number of cells per stack
#define MAX_STACKS 6            // maximum number of stacks  
#define Fault_led 21            // fault
#define Active_led 22           // active
#define Relay 13                // relay control
#define Nfault 4                // fault input pin

// ===================== BMS config ===================================================== //

BQ79600config config ;

float tempMAX ;                               // Maximum temperature limit
float tempMIN ;                               // Minimum temperature limit
float Vmaxlimit ;                             // Maximum cell voltage limit
float Vminlimit ;                             // Minimum cell voltage limit
float Vmaxlimit_pack ;                        // Maximum pack voltage limit
float Vminlimit_pack ;                        // Minimum pack voltage limit
float current_max ;                           // Maximum allowable current
float VoltDiffBalance ;                       // Threshold for balancing


// ==================== BMS State ===================================================== //

enum State
{
    Active,
    Sleep,
    Balance,
    Fault
};
volatile State currentState = Active;

// ================== Parameters milli ===================================================== //
//  send balance 
unsigned long lastcanBalanceTime = 0;
const unsigned long canBalanceInterval = 100;  // 100 ms
//  เพิ่มตัวแปรสำหรับ LED blink 
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 100;  // 100 ms
bool ledState = false;

unsigned long startTime;                            // เวลาเริ่มต้น
const unsigned long detailInterval = 0;             // ms, detail frame frequency ~2.5 Hz
unsigned long lastDetailTime;

unsigned long lastCANCheck = 0;  // ✅ เพิ่มบรรทัดนี้ (ตอนนี้ไม่มี)
const unsigned long CAN_CHECK_INTERVAL = 50;  // ✅ เพิ่มบรรทัดนี้

// =================== Parameters CAN ===================================================== //

uint8_t  statusFault = 0x00 ;                       // Fault status though CAN

bool loggingEnabled = true;

const uint16_t CAN_ID_SUMMARY_BASE  = 0x400;        // Summary high priority
const uint16_t CAN_ID_DETAIL_BASE   = 0x500;        // Detail low priority
const uint16_t CAN_ID_CONFIG        = 0x700;        // Config messages
// ================= Parameter Data ===================================================== //
struct Stack_data {
    float vmin;
    float vmax;
    float Vdiff ;
    float averageVcell; 
    float total_voltage;
    float current;
    size_t vminCells[MAX_CELLS];     //  array number of cells with vmin
    uint8_t vminCellsCount;          //  count number of cells with vmin
    size_t vmaxCells[MAX_CELLS];     //  array number of cells with vmax
    uint8_t vmaxCellsCount;          //  count number of cells with vmax
};

Stack_data extremaList[MAX_STACKS];       

// ================= Parameter Balance ===================================================== //

uint8_t Auto = 1; 
uint8_t Manual = 0; 
uint8_t currentlyBalancingCellsCount[MAX_STACKS];       // จำนวน cell ที่ balance ในแต่ละ stack
float Vref[MAX_STACKS];         
bool ready_balance = true;
bool balancing_complete = true;
bool balance_flag[MAX_STACKS]; 
size_t currentlyBalancingCells[MAX_STACKS][MAX_CELLS];  // [stack][cell_index]


// ================= Another Parameter ===================================================== //
const int MAX_RETRIES = 3;
const int RETRY_DELAY_MS = 5;

// ====================== Objects config =================================================== //

BQ79600 *bms = nullptr;
HardwareSerial mySerial(1);
MCP2515 mcp2515(5); // CS pin
BMSConfigManager configManager ; 

// ========================== Function  =================================================== //
void initializeBMS();
void handleCANMessages();
void applyConfigToGlobals();  
void bms_balancing();
void sendCAN ();
void print_data_balance();
void print_data_active();
void AnalyzePerStackStats(Stack_data result[],uint8_t numStacks);
bool status_balancing(); 
bool checkFaults();
bool checkHardwareFaults();
uint8_t voltageToHex(float Vmin);


// interrupt hardware fault
volatile bool interruptTriggered = false;
void IRAM_ATTR handleInterrupt() {
    digitalWrite(Relay, LOW);                           //  Relay close (safety)
    digitalWrite(Active_led, LOW);
    interruptTriggered = true;
    currentState = Fault;
    Serial.println("55555555555555555555555555555555555555555555----------!");
}
// ====================================================================================== //

void setup() {

    Serial.begin(115200);
    while (!Serial);
    pinMode(Fault_led, OUTPUT);                         // status output config (fault led)
    pinMode(Active_led, OUTPUT);                        // status output config (active led)
    pinMode(Relay,OUTPUT);                              // relay output config
    pinMode(Nfault,INPUT);                              // nfault input config

    digitalWrite(Relay, LOW);
    digitalWrite(Fault_led, LOW);
    digitalWrite(Active_led, LOW);

    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    mySerial.begin(1000000, SERIAL_8N1, 16, 17);        // BQ79600 Serial config

    bool config_ok = configManager.initialize();
    
    if (!config_ok) {
        Serial.println("⚠️ Config initialization failed, using defaults");
    }
    // Apply config
    applyConfigToGlobals();
    initializeBMS();
    
    attachInterrupt(digitalPinToInterrupt(Nfault), handleInterrupt, FALLING);
    digitalWrite(Relay, HIGH);
    
}
void loop() {
    
    delayMicroseconds(500);
    unsigned long currentMillis = millis();
    if (currentMillis - lastCANCheck >= CAN_CHECK_INTERVAL) {
        handleCANMessages();
        lastCANCheck = currentMillis;
    }
    
    // ✅ 2. Check Hardware Config Change (need re-init)
    if (configManager.needsRestart()) {
        Serial.println("\n🔄 Hardware config changed! Re-initializing BMS...");
        
        applyConfigToGlobals();  // ✅ Update config struct
        initializeBMS();         // ✅ Re-create BMS object
        configManager.clearRestartFlag();
        
        Serial.println("✅ BMS re-initialized\n");
    }

    switch (currentState){

        case Active :{
            
            digitalWrite(Active_led, HIGH);
            bms->get_data();
            AnalyzePerStackStats(extremaList, bms->NumSegments);
            //sendCAN ();
            print_data_active(); 
            ///------------------ fault software check ------------------/// 
            if (checkFaults()) {
                currentState = Fault;
                digitalWrite(Relay, LOW);  // ปิด Relay ทันที (ปลอดภัย)
                digitalWrite(Active_led, LOW);
                Serial.println("6666666666666666666666666----------!");
                break;  // ← ออกจาก case Active ทันที
            }
            //------------------ find Vmin each stack ------------------//
            if(balancing_complete == true){
                // เก็บ Vref สำหรับแต่ละ stack
                for (size_t i = 0; i < bms->NumSegments; ++i) {
                    Vref[i] = extremaList[i].vmin;
                    Serial.printf("Stack %d: Vref = %.3f V (from vmin)\n", i, Vref[i]);
                }
                balancing_complete = false;
                Serial.println("=====================================\n");
            }

            //------------------ Check each stack for balancing need  ------------------//    
            for (size_t i = 0; i < bms->NumSegments; ++i) {
            
                extremaList[i].Vdiff = extremaList[i].vmax - extremaList[i].vmin;
                Serial.printf("Stack %d Voltage Diff: %.3f V\n", (int)i, extremaList[i].Vdiff);
                float Vconddiff = extremaList[i].vmax - Vref[i]; // conditioned balance each stack for set flag
                if (Vconddiff > VoltDiffBalance) {
                        balance_flag[i] = true;
                        ready_balance = true;
                }else{
                        balance_flag[i] = false;
                }
            }
            //------------------ Check each stack for change state ------------------//
            if(ready_balance){
                currentState = Balance ;
            }else{
                Serial.printf("No balancing required for else stack");
                if (balancing_complete == false){
                    startTime = millis(); // บันทึกเวลาเริ่มต้น
                    balancing_complete = true;
                }
            }
            if (balancing_complete){
                unsigned long currentMillis = millis();
                if (currentMillis - startTime >= 60000*5) { // 5 นาที
                        
                    Serial.println("Go to Sleep mode");
                    currentState = Sleep;  
                    break;  // ออกจาก loop
                        
                }
            } 
            
            
            break;
        }
        

        case Balance :{
            unsigned long currentMillis = millis();
            if (currentMillis - lastBlinkTime >= blinkInterval) {
                lastBlinkTime = currentMillis;
                ledState = !ledState;
                digitalWrite(Active_led, ledState ? HIGH : LOW);
            }

            if (currentMillis - lastcanBalanceTime >= canBalanceInterval) {
                lastcanBalanceTime = currentMillis;
                for (size_t i = 0; i < bms->NumSegments ; ++i) {  
                    if (balance_flag[i]) {
                        bms->pauseBalanceCells(i);
                    }
                }
                //sendCAN ();
                for (size_t i = 0; i < bms->NumSegments ; ++i) {  
                    if (balance_flag[i]) {
                        bms->unpauseBalanceCells(i);
                    }
                }
            }

            if (ready_balance) {
                ready_balance = false;
                bms_balancing();
            }

            bms->get_data();
            AnalyzePerStackStats(extremaList, bms->NumSegments);
            //sendCAN ();
            //print_data_balance();
            if (status_balancing()) {
                for (size_t i = 0; i < bms->NumSegments ; ++i) {  
                    balance_flag[i] = false;
                }
                digitalWrite(Active_led, LOW);
                ledState = false;
                currentState = Active ;
            }

            
            break;
        }    
            

        case Fault: {
            Serial.println("\n===  Entering Fault State ===");
            Serial.println(" Relay: OFF (SAFETY MODE)");
            Serial.println("Waiting for all faults to clear...\n");
            unsigned long faultStateEntryTime = micros();  // เวลาเข้าสู่สถานะ Fault
            digitalWrite(Fault_led, HIGH);
            
            const unsigned long FAULT_CHECK_INTERVAL = 5 ;  // เช็คทุก  5 ms
            const unsigned long FAULT_CLEAR_DURATION = 1000;  // ต้องปกติต่อเนื่อง 1 วินาที
    
            unsigned long faultClearTime = 0;
            bool faultCleared = false;
    
            while (true) {  
                
                digitalWrite(Fault_led, HIGH);
                
                unsigned long currentMillis = millis();
                uint8_t newStatusFault = 0x00;
                bool anyFault = false;

                //  เช็ค Interrupt Flag (ถ้ามี interrupt เกิดขึ้นระหว่าง loop)
                if (interruptTriggered) {
                    Serial.println(" Interrupt triggered during fault check!");
                    interruptTriggered = false;
                }
                // เช็ค Hardware Fault จาก IC ก่อน
                bool hardwareFaultDetected = checkHardwareFaults();
                // เคลียร์ statusFault ก่อนเช็คใหม่

                if (hardwareFaultDetected) {
                    Serial.println(" Hardware fault detected! Clearing...");
                    bms->clearFault();  // พยายามเคลียร์ fault
                    faultCleared = false;
                    bms->get_data();
                    AnalyzePerStackStats(extremaList, bms->NumSegments);
                    //print_data_active();
                    continue;  // ข้ามไปรอบถัดไป
                }     

                // === เช็คทุก Stack === //
                for (size_t i = 0; i < bms->NumSegments; ++i) {
            
                    // 1. Pack Overvoltage (bit 4)
                    if (extremaList[i].total_voltage > Vmaxlimit_pack) {
                        newStatusFault |= 0x10;
                        anyFault = true;
                        Serial.printf(" Stack %d: Pack OV (%.1f V > %.1f V)\n", i, extremaList[i].total_voltage, Vmaxlimit_pack);
                    }
            
                    // 2. Pack Undervoltage (bit 5)
                    if (extremaList[i].total_voltage < Vminlimit_pack) {
                        newStatusFault |= 0x20;
                        anyFault = true;
                        Serial.printf(" Stack %d: Pack UV (%.1f V < %.1f V)\n", i, extremaList[i].total_voltage, Vminlimit_pack);
                    }
            
                    // 3. Cell Overvoltage (bit 0)
                    if (extremaList[i].vmax >= Vmaxlimit) {
                        newStatusFault |= 0x01;
                        anyFault = true;
                        Serial.printf(" Stack %d: Cell OV (%.3f V >= %.1f V)\n", i, extremaList[i].vmax, Vmaxlimit);
                    }
            
                    // 4. Cell Undervoltage (bit 1)
                    if (extremaList[i].vmin <= Vminlimit) {
                        newStatusFault |= 0x02;
                        anyFault = true;
                        Serial.printf(" Stack %d: Cell UV (%.3f V <= %.1f V)\n", i, extremaList[i].vmin, Vminlimit);
                    }
            
                    // 5. Overcurrent (bit 6)
                    if (extremaList[i].current >= current_max) {
                        newStatusFault |= 0x40;
                        anyFault = true;
                        Serial.printf(" Stack %d: Overcurrent (%.1f A >= %.1f A)\n", i, extremaList[i].current, current_max);
                    } 
            
                    // 6. Temperature checks (all sensors)
                    for (size_t t = 0; t <  bms->NumThermistors ; ++t) {
                        float temp = bms->batteryData_pack[i].gpioTemps[t];
                
                        // Overtemperature (bit 2)
                        if (temp >= tempMAX) {
                            newStatusFault |= 0x04;
                            anyFault = true;
                            Serial.printf(" Stack %d, Sensor %d: Over temp (%.1f°C >= %.1f°C)\n", i, t, temp, tempMAX);
                        }
                
                        // Undertemperature (bit 3)
                        if (temp <= tempMIN) {
                            newStatusFault |= 0x08;
                            anyFault = true;
                            Serial.printf(" Stack %d, Sensor %d: Under temp (%.1f°C <= %.1f°C)\n", i, t, temp, tempMIN);
                        }
                    }
                }
                /* bms->get_data();
                extremaList = AnalyzePerStackStats(bms->batteryData_pack); */
            
                // อัปเดต statusFault
                statusFault = newStatusFault;
        
                // แสดง statusFault
                if (anyFault) {
                    Serial.printf(" statusFault: 0x%02X (", statusFault);
                    if (statusFault & 0x01) Serial.print("CellOV ");
                    if (statusFault & 0x02) Serial.print("CellUV ");
                    if (statusFault & 0x04) Serial.print("OverTemp ");
                    if (statusFault & 0x08) Serial.print("UnderTemp ");
                    if (statusFault & 0x10) Serial.print("PackOV ");
                    if (statusFault & 0x20) Serial.print("PackUV ");
                    if (statusFault & 0x40) Serial.print("OverCurrent ");
                    Serial.println(")");
                }
        
                // ส่งข้อมูลผ่าน CAN (รวม statusFault)
                //sendCAN();
        
                // === ตรวจสอบว่า Fault หายหรือยัง === //
                if (!anyFault) {
                    // ถ้าเพิ่งหาย → บันทึกเวลา
                    if (!faultCleared) {
                        faultClearTime = currentMillis;
                        faultCleared = true;
                        Serial.println(" All faults cleared! Waiting for stability...");
                    }
            
                    // ตรวจสอบว่าปกติต่อเนื่องนานพอหรือยัง
                    if (currentMillis - faultClearTime >= FAULT_CLEAR_DURATION) {
                        Serial.println("\n===  Fault Recovery Successful ===");
                        Serial.println("All conditions normal for 5 seconds");
                        Serial.println("Returning to Active state");
                        Serial.println(" Relay: ON");
                
                        currentState = Active;
                        statusFault = 0x00;
                        digitalWrite(Fault_led, LOW);
                        digitalWrite(Relay, HIGH);  // เปิด Relay เมื่อปลอดภัยแล้ว
                        break;  // ออกจาก Fault state
                    }
            
                    // แสดงเวลาที่เหลือ
                    unsigned long elapsed = currentMillis - faultClearTime;
                    Serial.printf("  Stable: %.1f s / 5.0 s\n", elapsed / 1000.0f);
            
                } else {
                    // ยังมี Fault → รีเซ็ตตัวนับ
                    if (faultCleared) {
                        Serial.println("Fault reappeared! Resetting stability timer");
                        faultCleared = false;
                    }
                }
                
                digitalWrite(Fault_led, HIGH);
                delay(FAULT_CHECK_INTERVAL);
            }
    
            break;
        }
        
    }
}

void AnalyzePerStackStats(Stack_data result[], uint8_t numStacks) {
    
    for (uint8_t stackIndex = 0; stackIndex < numStacks; stackIndex++) {
        const StackData& stack = bms->batteryData_pack[stackIndex];

        Stack_data& extrema = result[stackIndex];
        
        // Reset values
        extrema.vmin = 999.0f;
        extrema.vmax = 0.0f;
        extrema.vminCellsCount = 0;
        extrema.vmaxCellsCount = 0;
        
        float totalVoltage = 0.0f;

        // หา vmin, vmax
        for (uint8_t i = 0; i < stack.numCells; i++) {
            float v = stack.cells[i].voltage;
            totalVoltage += v;

            // Check vmin
            if (v < extrema.vmin) {
                extrema.vmin = v;
                extrema.vminCellsCount = 0;
                extrema.vminCells[extrema.vminCellsCount++] = i;
            } else if (v == extrema.vmin && extrema.vminCellsCount < MAX_CELLS) {
                extrema.vminCells[extrema.vminCellsCount++] = i;
            }

            // Check vmax
            if (v > extrema.vmax) {
                extrema.vmax = v;
                extrema.vmaxCellsCount = 0;
                extrema.vmaxCells[extrema.vmaxCellsCount++] = i;
            } else if (v == extrema.vmax && extrema.vmaxCellsCount < MAX_CELLS) {
                extrema.vmaxCells[extrema.vmaxCellsCount++] = i;
            }
        }
        
        // คำนวณค่าพื้นฐาน
        extrema.total_voltage = totalVoltage;
        extrema.averageVcell = totalVoltage / stack.numCells;
        extrema.Vdiff = extrema.vmax - extrema.vmin;
        extrema.current = 0.0f;  // ตั้งค่าเริ่มต้น
        
        // แสดงผล vmin, vmax
        Serial.printf("Stack %d - Vmin: %.3f V [cells:", stackIndex, extrema.vmin);
        for (uint8_t idx = 0; idx < extrema.vminCellsCount; idx++) {
            Serial.printf(" %d", (int)extrema.vminCells[idx]);
        }
        Serial.printf(" ] | Vmax: %.3f V [cells:", extrema.vmax);
        for (uint8_t idx = 0; idx < extrema.vmaxCellsCount; idx++) {
            Serial.printf(" %d", (int)extrema.vmaxCells[idx]);
        }
        Serial.println(" ]");
    }

    // คำนวณ current จาก stack สุดท้าย แล้วคัดลอกไปทุก stack
    if (numStacks > 0) {
        uint8_t lastStackIndex = numStacks - 1;
        
        // อ่าน busbar voltage จาก stack สุดท้าย
        float busbarVolt = bms->batteryData_pack[lastStackIndex].busbarVolt;
        
        // คำนวณ current (busbarVolt เป็น mV, shunt_resistance เป็น mΩ)
        float current = busbarVolt / config.shunt_resistance;
        
        Serial.println("\n=== Current Calculation ===");
        Serial.printf("r_shunt: %.6f mΩ\n", config.shunt_resistance);
        Serial.printf("busbarVolt (Stack %d): %.3f mV\n", lastStackIndex, busbarVolt);
        Serial.printf("Calculated Current: %.3f A\n", current);
        Serial.println("===========================\n");
        
        // คัดลอก current ไปทุก stack
        for (uint8_t i = 0; i < numStacks; i++) {
            result[i].current = current;
        }
    }
}

bool status_balancing(){
    int result[bms->NumSegments] ;
    bool allCompleted = true;
    uint8_t addressBQ = 0 ;
    for(uint8_t stackIndex = 0; stackIndex < bms->NumSegments; ++stackIndex) {
        addressBQ = stackIndex + 1;
        result[stackIndex] = bms->checkBalance(addressBQ);
        //----------------------------------------------------------------//
        if (result[stackIndex] == 0x80){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("Invalid CB setting");
            allCompleted = false;
        }else if (result[stackIndex] == 0x40){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("NTC thermistor measurement is greater than OTCB_THR OR CBFET temp  is greater than CB TWARN"); 
            allCompleted = false;  
        }else if (result[stackIndex] == 0x20){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("cell balancing pause status");
            allCompleted = false;
        }else if (result[stackIndex] == 0x10){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("Module balancing");
            allCompleted = false;
        }else if (result[stackIndex] == 0x08){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("At least 1 cell is in active cell balancing");
            allCompleted = false;
        }else if (result[stackIndex] == 0x04){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println(" fault detect");
            allCompleted = false;
            currentState = Fault;
        }else if (result[stackIndex] == 0x02){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("Module balancing completed");   
            allCompleted = false;
        }else if (result[stackIndex] == 0x01 ){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println(" All cell balancing is completed");   
        }else if (result[stackIndex] == 0x00){
            Serial.printf("=== Stack %d Balance Status (0x%02X) ===\n", stackIndex, result[stackIndex]);
            Serial.println("✅ No balancing (idle/ready)");
        //----------------------------------------------------------------//
        }
    }
    if (allCompleted) {
        Serial.println("=== All stacks balancing completed (all 0x00 or 0x01) ===");
    } else {
        Serial.println("=== ⏳ Balancing still in progress or other status detected ===");
    }
    
    return allCompleted;
}

uint8_t voltageToHex(float Vmin) {

    if (Vmin < 2.45f) Vmin = 2.45f;
    if (Vmin > 4.00f) Vmin = 4.00f;

    int step = round((Vmin - 2.45f) / 0.025f) + 1;
    if (step < 1)  step = 1;
    if (step > 63) step = 63;

    float voltage_out = 2.45f + (step - 1) * 0.025f;
    return voltage_out;
}

void sendFrameWithRetry(struct can_frame& frame) {
    int result = mcp2515.sendMessage(&frame);
    int retries = 0;
    while (result != 0 && retries < MAX_RETRIES) {
        delay(RETRY_DELAY_MS);
        result = mcp2515.sendMessage(&frame);
        retries++;
    }
    if (result != 0) {
        Serial.printf("Frame 0x%03X failed after %d retries\n", frame.can_id, retries);
    }
}

// ----- Send Summary (high priority) -----
void sendSummaryCAN(int stackIndex, const Stack_data& ext, const StackData& stack) {
    // Frame 1: totalV, current, vmin/vmax
    struct can_frame frame1;
    frame1.can_id  = CAN_ID_SUMMARY_BASE + (stackIndex << 4);
    frame1.can_dlc = 8;

    uint16_t totalV = (uint16_t)(ext.total_voltage * 10.0f);  // 0.1V per LSB
    int16_t curr    = (int16_t)(ext.current * 10.0f);          // 0.1A per LSB
    uint16_t vmin   = (uint16_t)(ext.vmin * 1000.0f);          // mV
    uint16_t vmax   = (uint16_t)(ext.vmax * 1000.0f);

    frame1.data[0] = (totalV >> 8) & 0xFF;
    frame1.data[1] = totalV & 0xFF;
    frame1.data[2] = (curr >> 8) & 0xFF;
    frame1.data[3] = curr & 0xFF;
    frame1.data[4] = (vmin >> 8) & 0xFF;
    frame1.data[5] = vmin & 0xFF;
    frame1.data[6] = (vmax >> 8) & 0xFF;
    frame1.data[7] = vmax & 0xFF;

    sendFrameWithRetry(frame1);

    // Frame 2: Temps, averageVcell
    struct can_frame frame2;
    frame2.can_id  = CAN_ID_SUMMARY_BASE + (stackIndex << 4) + 1;
    frame2.can_dlc = 8;

    uint16_t avgV = (uint16_t)(ext.averageVcell * 1000.0f); // mV

    frame2.data[0] = (int8_t)stack.dieTemp;
    frame2.data[1] = (int8_t)stack.gpioTemps[0];
    frame2.data[2] = (int8_t)stack.gpioTemps[1];
    frame2.data[3] = 0; // reserved
    frame2.data[4] = (avgV >> 8) & 0xFF;
    frame2.data[5] = avgV & 0xFF;
    frame2.data[6] = 0; // reserved
    frame2.data[7] = 0;

    sendFrameWithRetry(frame2);
}

// ----- Send Detail / Cell-level (low priority) -----
void sendDetailCAN(int stackIndex, const StackData &stack) {
    uint8_t raw[20];
    int idx = 0;
    
    for (uint8_t i = 0; i < bms->NumCellsSeries; i++) {
        uint16_t v = (uint16_t)(stack.cells[i].voltage * 1000.0f);
        raw[idx++] = (v >> 8) & 0xFF;
        raw[idx++] = v & 0xFF;
    }

    // สร้าง bitmask
    uint16_t balanceMask = 0;
    if (stackIndex < MAX_STACKS) {
        for (uint8_t i = 0; i < currentlyBalancingCellsCount[stackIndex]; i++) {
            size_t cellIndex = currentlyBalancingCells[stackIndex][i];
            if (cellIndex < bms->NumCellsSeries) {
                balanceMask |= (1 << cellIndex);
            }
        }
    }

    const uint8_t fragLens[3] = {8, 8, 7};
    int offset = 0;
    
    for (int frag = 0; frag < 3; frag++) {
        struct can_frame frame;
        frame.can_id  = CAN_ID_DETAIL_BASE + (stackIndex << 4) + frag;
        frame.can_dlc = fragLens[frag];
        memcpy(frame.data, &raw[offset], fragLens[frag]);

        if (frag == 2) {
            frame.data[4] = (balanceMask >> 8) & 0xFF;
            frame.data[5] = balanceMask & 0xFF;
            frame.data[6] = statusFault;
        }

        sendFrameWithRetry(frame);
        offset += fragLens[frag];
    }
}

// ----- Main sendCAN function -----
void sendCAN() {
    unsigned long currentMillis = millis();

    for (int i = 0; i < bms->NumSegments; i++) {
        // Summary: high priority, send every loop
        sendSummaryCAN(i, extremaList[i], bms->batteryData_pack[i]);

        // Detail: low priority, send slower
        if (loggingEnabled && (currentMillis - lastDetailTime >= detailInterval)) {
            sendDetailCAN(i, bms->batteryData_pack[i]);
        }
    }

    // Update lastDetailTime if we sent detail
    if (loggingEnabled && (currentMillis - lastDetailTime >= detailInterval)) {
        lastDetailTime = currentMillis;
    }
}
void bms_balancing(){
    for (uint8_t stackIndex = 0; stackIndex < bms->NumSegments; stackIndex++) {
        
        if (!balance_flag[stackIndex]) continue;
        
        const StackData& stack = bms->batteryData_pack[stackIndex];
        
        Serial.println("===============================");
        Serial.printf("Stack %d (Address: %d): Cell Voltages:\n", 
                      stackIndex, stackIndex + 1);
        
        for (uint8_t j = 0; j < stack.numCells; j++) {
            Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", 
                          j + 1, j, stack.cells[j].voltage);
        }
        
        Serial.println("===============================");
        Serial.printf("Balancing Stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                      stackIndex, extremaList[stackIndex].vmin, 
                      extremaList[stackIndex].vmax, extremaList[stackIndex].Vdiff);
        
        // === ค้นหา cells ที่ต้อง balance ===
        size_t overVoltageCells[MAX_CELLS];
        uint8_t overVoltageCellsCount = 0;
        
        for (uint8_t j = 0; j < stack.numCells; j++) {
            if (stack.cells[j].voltage >= Vref[stackIndex]) {
                overVoltageCells[overVoltageCellsCount++] = j;
            }
        }
        
        // === Bubble sort (จากสูงไปต่ำ) ===
        for (uint8_t a = 0; a < overVoltageCellsCount - 1; a++) {
            for (uint8_t b = 0; b < overVoltageCellsCount - a - 1; b++) {
                if (stack.cells[overVoltageCells[b]].voltage < 
                    stack.cells[overVoltageCells[b + 1]].voltage) {
                    
                    size_t temp = overVoltageCells[b];
                    overVoltageCells[b] = overVoltageCells[b + 1];
                    overVoltageCells[b + 1] = temp;
                }
            }
        }
        
        // === กรอง cells ที่ติดกัน ===
        size_t filteredCells[MAX_CELLS];
        uint8_t filteredCount = 0;
        
        for (uint8_t j = 0; j < overVoltageCellsCount; j++) {
            size_t current_cell = overVoltageCells[j];
            bool isAdjacent = false;
            
            for (uint8_t k = 0; k < filteredCount; k++) {
                int diff = (int)current_cell - (int)filteredCells[k];
                if (diff >= -1 && diff <= 1) {
                    isAdjacent = true;
                    break;
                }
            }
            
            if (!isAdjacent && filteredCount < MAX_CELLS) {
                filteredCells[filteredCount++] = current_cell;
            }
        }
        
        // === แสดงผล ===
        Serial.println("------------------------------------------------");
        Serial.printf("Balancing Stack %d (Address: %d)\n", 
                      stackIndex, stackIndex + 1);
        Serial.print("Selected Cells for Balancing: ");
        for (uint8_t idx = 0; idx < filteredCount; idx++) {
            Serial.printf("%d (%.3fV), ", 
                          filteredCells[idx], stack.cells[filteredCells[idx]].voltage);
        }
        Serial.println();
        Serial.println("------------------------------------------------");
        
        // === เรียก BalanceCells ===
        if (filteredCount > 0) {
            //  แปลง Vref → hex threshold
            uint8_t hexThreshold = voltageToHex(Vref[stackIndex]);
            uint8_t addressBQ = stackIndex + 1;
            
            // เก็บ cells ที่ balance
            for (uint8_t idx = 0; idx < filteredCount; idx++) {
                currentlyBalancingCells[stackIndex][idx] = filteredCells[idx];
            }
            currentlyBalancingCellsCount[stackIndex] = filteredCount;
            
            Serial.printf("  → Sending to BQ79600 Address: %d\n", addressBQ);
            Serial.printf("  → V_ref: %.3f V → Hex: 0x%02X\n", 
                          Vref[stackIndex], hexThreshold);
            
            //  เรียก BalanceCells (5 parameters)
            bms->BalanceCells(Auto, addressBQ, 
                             currentlyBalancingCells[stackIndex],
                             currentlyBalancingCellsCount[stackIndex],
                             hexThreshold);
        }
    }
}
void print_data_balance(){
    for (uint8_t stack = 0; stack < bms->NumSegments; stack++) {
        const StackData& kts = bms->batteryData_pack[stack];
        
        Serial.println("===============================");
        Serial.printf("Stack %d (Address: %d): Cell Voltages:\n", stack, stack + 1);
        
        for (uint8_t cell = 0; cell < kts.numCells; cell++) {
            Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", 
                          cell + 1, cell, kts.cells[cell].voltage);
        }
        
        Serial.println("===============================");
        Serial.printf("volt stack: %.2f V\n", extremaList[stack].total_voltage);
        Serial.printf("volt AVG: %.3f V\n", extremaList[stack].averageVcell);
        Serial.printf("volt diff: %.3f V\n", extremaList[stack].Vdiff);
        Serial.printf("V_ref: %.3f V\n", Vref[stack]);
        Serial.printf("Temp1: %.1f °C\n", kts.gpioTemps[0]);
        Serial.printf("Temp2: %.1f °C\n", kts.gpioTemps[1]);
        Serial.printf("DieTemp: %.1f °C\n", kts.dieTemp);
        Serial.println("===============================");
        
        Serial.print("Balancing Cells: ");
        if (currentlyBalancingCellsCount[stack] > 0) {
            for (uint8_t idx = 0; idx < currentlyBalancingCellsCount[stack]; idx++) {
                size_t cellIdx = currentlyBalancingCells[stack][idx];
                Serial.printf("%d (%.3fV), ", cellIdx, kts.cells[cellIdx].voltage);
            }
        } else {
            Serial.print("None");
        }
        Serial.println();
    }
    Serial.println();
}

void print_data_active(){
    for (uint8_t stack = 0; stack < bms->NumSegments; stack++) {
        const StackData& kts = bms->batteryData_pack[stack];
        
        Serial.println("===============================");
        Serial.printf("Stack %d (Address: %d): Cell Voltages:\n", stack, stack + 1);
        
        for (uint8_t cell = 0; cell < kts.numCells; cell++) {
            Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", 
                          cell + 1, cell, kts.cells[cell].voltage);
        }
        
        Serial.println("===============================");
        Serial.printf("volt stack: %.2f V\n", extremaList[stack].total_voltage);
        Serial.printf("volt AVG: %.3f V\n", extremaList[stack].averageVcell);
        Serial.printf("V_ref: %.3f V\n", Vref[stack]);
        Serial.printf("Temp1: %.1f °C\n", kts.gpioTemps[0]);
        Serial.printf("Temp2: %.1f °C\n", kts.gpioTemps[1]);
        Serial.printf("DieTemp: %.1f °C\n", kts.dieTemp);
        Serial.println("===============================");
    }
    Serial.println();
}

bool checkFaults() {

    if(interruptTriggered) {
        Serial.println("⚠️ Interrupt triggered! Checking faults...");
        interruptTriggered = false;  
        return true; 
    }
    
    for(uint8_t i = 0; i < bms->NumSegments; i++) {
        
        // Current check
        if (extremaList[i].current >= current_max) {
            Serial.printf("⚠️ Stack %d: Over current (%.1f A)\n", i, extremaList[i].current);
            return true;
        }
        
        // Cell voltage checks
        if (extremaList[i].vmax > Vmaxlimit) {
            Serial.printf("⚠️ Stack %d: Over voltage (%.3f V)\n", i, extremaList[i].vmax);
            return true;
        }
        
        if (extremaList[i].vmin < Vminlimit) {
            Serial.printf("⚠️ Stack %d: Under voltage (%.3f V)\n", i, extremaList[i].vmin);
            return true;
        }
        
        // Pack voltage checks
        if (extremaList[i].total_voltage > Vmaxlimit_pack) {
            Serial.printf("⚠️ Stack %d: Pack over voltage (%.1f V)\n", i, extremaList[i].total_voltage);
            return true;
        }
        
        if (extremaList[i].total_voltage < Vminlimit_pack) {
            Serial.printf("⚠️ Stack %d: Pack under voltage (%.1f V)\n", i, extremaList[i].total_voltage);
            return true;
        }
        
        // Temperature checks (all sensors)
        for (uint8_t t = 0; t < bms->NumThermistors; t++) {
            float temp = bms->batteryData_pack[i].gpioTemps[t];
            if (temp >= tempMAX || temp <= tempMIN) {
                Serial.printf("⚠️ Stack %d, Sensor %d: Temp fault (%.1f°C, Limit: %.1f-%.1f°C)\n",
                              i, t, temp, tempMIN, tempMAX);
                return true;
            }
        }
    }
    
    return false;
}

bool checkHardwareFaults() {
    Serial.println("\n=== Checking Hardware Faults from IC ===");
    
    uint8_t stackFaults[MAX_STACKS];
    uint8_t otpFaults[MAX_STACKS];
    
    bool anyHardwareFault = false;
    uint8_t hardwareStatusFault = 0x00;
    
    // เช็ค Base Device Faults
    Serial.println("\n--- Checking Base Device Faults (BQ79612) ---");
    bms->Fault_Summary(stackFaults);
    
    for (uint8_t physicalIdx = 0; physicalIdx < bms->NumSegments; physicalIdx++) {
        uint8_t logicalIdx = (bms->NumSegments - 1) - physicalIdx;
        
        if (stackFaults[physicalIdx] != 0x00) {
            anyHardwareFault = true;
            
            Serial.printf("⚠️ Stack %d (Address: %d) Base Fault: 0x%02X\n",
                          logicalIdx, logicalIdx + 1, stackFaults[physicalIdx]);
            
            if (stackFaults[physicalIdx] & 0x80) {
                Serial.printf("  → Stack %d: FAULT_PROT\n", logicalIdx);
                hardwareStatusFault |= 0x80;
            }
            
            if (stackFaults[physicalIdx] & 0x40) {
                Serial.printf("  → Stack %d: FAULT_COMP_ADC\n", logicalIdx);
                hardwareStatusFault |= 0x80;
            }
            
            if (stackFaults[physicalIdx] & 0x20) {
                Serial.printf("  → Stack %d: FAULT_OTP\n", logicalIdx);
                
                bms->FAULT_OTP(otpFaults);
                
                Serial.print("  Response: ");
                for (uint8_t j = 0; j < bms->NumSegments; j++) {
                    Serial.printf("0x%02X ", otpFaults[j]);
                }
                Serial.println();
                
                hardwareStatusFault |= 0x80;
            }
            
            if (stackFaults[physicalIdx] & 0x10) {
                Serial.printf("  → Stack %d: FAULT_COMM\n", logicalIdx);
                hardwareStatusFault |= 0x80;
            }
            
            if (stackFaults[physicalIdx] & 0x08) {
                Serial.printf("  → Stack %d: FAULT_OTUT\n", logicalIdx);
                hardwareStatusFault |= 0x0C;
            }
            
            if (stackFaults[physicalIdx] & 0x04) {
                Serial.printf("  → Stack %d: FAULT_OVUV\n", logicalIdx);
                hardwareStatusFault |= 0x03;
            }
            
            if (stackFaults[physicalIdx] & 0x02) {
                Serial.printf("  → Stack %d: FAULT_SYS\n", logicalIdx);
                hardwareStatusFault |= 0x80;
            }
            
            if (stackFaults[physicalIdx] & 0x01) {
                Serial.printf("  → Stack %d: FAULT_PWR\n", logicalIdx);
                hardwareStatusFault |= 0x80;
            }
        } else {
            Serial.printf(" Stack %d (Address: %d): No base faults\n", 
                          logicalIdx, logicalIdx + 1);
        }
    }
    
    // Bridge Device Fault
    Serial.println("\n--- Checking Bridge Device Faults (BQ79600) ---");
    uint8_t bridgeFault = bms->FaultM_Summary();
    
    if (bridgeFault != 0x00) {
        anyHardwareFault = true;
        
        Serial.printf(" Bridge Fault: 0x%02X\n", bridgeFault);
        
        if (bridgeFault & 0x08) {
            Serial.println("  → Bridge: FAULT_COMM");
            uint8_t comm1 = bms->FaultM_Comm1();
            uint8_t comm2 = bms->FaultM_Comm2();
            Serial.printf("  Response1: 0x%02X, Response2: 0x%02X\n", comm1, comm2);
            hardwareStatusFault |= 0x80;
        }
        
        if (bridgeFault & 0x04) {
            Serial.println("  → Bridge: FAULT_REG");
            uint8_t reg = bms->FaultM_REG();
            Serial.printf("  Response: 0x%02X\n", reg);
            bms->cheak();
            hardwareStatusFault |= 0x80;
        }
        
        if (bridgeFault & 0x02) {
            Serial.println("  → Bridge: FAULT_SYS");
            hardwareStatusFault |= 0x80;
        }
        
        if (bridgeFault & 0x01) {
            Serial.println("  → Bridge: FAULT_PWR");
            uint8_t pwr = bms->FaultM_PWR();
            Serial.printf("  Response: 0x%02X\n", pwr);
            hardwareStatusFault |= 0x80;
        }
    } else if (bridgeFault == 0xFF) {
        Serial.println(" Bridge: Failed to read fault status");
        anyHardwareFault = true;
        hardwareStatusFault |= 0x80;
    } else {
        Serial.println(" Bridge: No faults");
    }
    
    if (anyHardwareFault) {
        statusFault = hardwareStatusFault;
        
        Serial.printf("\n Hardware statusFault: 0x%02X (", statusFault);
        if (statusFault & 0x80) Serial.print("HW_Fault ");
        if (statusFault & 0x40) Serial.print("OverCurrent ");
        if (statusFault & 0x20) Serial.print("PackUV ");
        if (statusFault & 0x10) Serial.print("PackOV ");
        if (statusFault & 0x08) Serial.print("UnderTemp ");
        if (statusFault & 0x04) Serial.print("OverTemp ");
        if (statusFault & 0x02) Serial.print("CellUV ");
        if (statusFault & 0x01) Serial.print("CellOV ");
        Serial.println(")");
    }
    
    Serial.println("=========================================\n");
    return anyHardwareFault;
}
void applyConfigToGlobals() {
    Serial.println("\n--- Applying Configuration to Globals ---");
    
    const HardwareConfig& hwConfig = configManager.getHardwareConfig();
    const ProtectionLimits& protLimits = configManager.getProtectionLimits();
    
    // Hardware Config
    config.num_segments = hwConfig.num_segments;
    config.num_cells_series = hwConfig.num_cells_series;
    config.num_thermistors = hwConfig.num_thermistors;
    config.shunt_resistance = hwConfig.shunt_resistance;
    
    Serial.println("Hardware Config:");
    Serial.printf("  Segments      : %d\n", config.num_segments);
    Serial.printf("  Cells/Series  : %d\n", config.num_cells_series);
    Serial.printf("  Thermistors   : %d\n", config.num_thermistors);
    Serial.printf("  Shunt R       : %.3f Ω\n", config.shunt_resistance);
    
    // Protection Limits
    tempMAX = protLimits.temp_max;
    tempMIN = protLimits.temp_min;
    Vmaxlimit = protLimits.cell_v_max;
    Vminlimit = protLimits.cell_v_min;
    Vmaxlimit_pack = protLimits.pack_v_max;
    Vminlimit_pack = protLimits.pack_v_min;
    current_max = protLimits.current_max;
    VoltDiffBalance = protLimits.volt_diff_balance;

    Serial.println("\nProtection Limits:");
    Serial.printf("  Temp Max      : %.1f °C\n", tempMAX);
    Serial.printf("  Temp Min      : %.1f °C\n", tempMIN);
    Serial.printf("  Cell V Max    : %.3f V\n", Vmaxlimit);
    Serial.printf("  Cell V Min    : %.3f V\n", Vminlimit);
    Serial.printf("  Pack V Max    : %.1f V\n", Vmaxlimit_pack);
    Serial.printf("  Pack V Min    : %.1f V\n", Vminlimit_pack);
    Serial.printf("  Current Max   : %.1f A\n", current_max);
    Serial.printf("  Balance Diff  : %.3f V\n", VoltDiffBalance);
}
// ========== Initialize BMS ==========
void initializeBMS() {
    Serial.println("Creating BMS object with current config...");
    
    if (bms != nullptr) {
        delete bms;
        bms = nullptr;
    }
    
    bms = new BQ79600(mySerial, 1000000, 17, config);
    
    bool start = false;
    
    while (!start) {
        digitalWrite(Fault_led, HIGH);
        digitalWrite(Active_led, HIGH);
        
        bool init = bms->initialize();
        
        if (init) {
            Serial.println(" BQ79600 initialized successfully");
            start = true;
        } 
        digitalWrite(Fault_led, LOW);
        digitalWrite(Active_led, LOW);
        delay(50);
    }
    
    delay(50);
    bms->clearFault();
    Serial.println(" BMS ready\n");
}
void handleCANMessages() {
    struct can_frame frame;
    
    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        
        if (frame.can_id == CAN_ID_CONFIG) {
            
            Serial.printf("\n Received CAN Config (ID: 0x%03X, Len: %d)\n", frame.can_id, frame.can_dlc);
            
            Serial.print("Data: ");
            for (uint8_t i = 0; i < frame.can_dlc; i++) {
                Serial.printf("%02X ", frame.data[i]);
            }
            Serial.println();
            
            bool ok = configManager.processCANCommand(
                frame.can_id,
                frame.data,
                frame.can_dlc
            );
            
            if (ok) {
                Serial.println(" Config command processed\n");
                
                //  Hot Reload Protection Limits
                applyConfigToGlobals();
                
            } else {
                Serial.println(" Config command failed!\n");
            }
        }
    }
}