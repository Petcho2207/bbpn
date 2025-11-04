#include "BQ79600.h"
#include <SPI.h>
#include <mcp2515.h>
#define LED1 21 // fault
#define LED2 22
#define Relay 13
#define Nfault 4
#define MAX_SIZE 10
extern MCP2515 mcp2515;
int Auto = 1; 
int Manual = 0; 
float Vdiff ;
float Vref ;
bool ready_balance = true;
bool balancing_complete = true;
// ---------------- BMS config ------------------//
float tempMAX = 35.0;
float tempMIN = 23.0;
float Vmaxlimit = 4.3;
float Vmaxlimit_pack = 43.0;
float Vminlimit = 3.1;
float Vminlimit_pack = 32.0;
//-----------------------------------------------//
unsigned long startTime;  // เวลาเริ่มต้น
uint8_t  statusFault = 0 ;

const unsigned long detailInterval = 0; // ms, detail frame frequency ~2.5 Hz
unsigned long lastDetailTime;

//----------------- CAN ID mapping ----------------// 
const uint16_t CAN_ID_SUMMARY_BASE  = 0x400; // Summary high priority
const uint16_t CAN_ID_DETAIL_BASE   = 0x500; // Detail low priority

// Retry mechanism
const int MAX_RETRIES = 3;
const int RETRY_DELAY_MS = 5;
//------------------------------------------------//
bool loggingEnabled = true;
enum State
{
    Active,
    Sleep,
    Balance,
    Fault
};
volatile State currentState = Active;
struct StackVoltageExtrema {
    float vmin;
    float vmax;
    float averageVcell; 
    float total_voltage;
    float current;
    std::vector<size_t> vminCells;
    std::vector<size_t> vmaxCells;
};
struct BmsDataFrame {
    float voltages[6][10];
    float gpioTemps[6][2];
    float dieTemps[6];
};
// Globals
BQ79600config bqConfig;
std::vector<std::vector<size_t>> currentlyBalancingCells;  // [stack] = list of balancing cell index
//std::vector<StackVoltageExtrema> allStackExtrema;
std::vector<StackVoltageExtrema> extremaList;
std::vector<size_t> filteredCells;

BmsDataFrame dataBuffer;
BQ79600config config = {
    10,      // num_cells_series
    2,       // num_thermistors
    1,       // num_segments
    1.5     // shunt_resistance m-ohm
};
const float VoltDiffBalance = 0.005; // Threshold for balancing
const float VoltDiff_StopBalance = 0.00; // Threshold for stop balancing
const float current_max = 100.0; // Maximum allowable current

HardwareSerial mySerial(1);
BQ79600 bms(mySerial, 1000000, 17, config);
MCP2515 mcp2515(5); // CS pin

// ----- interrupt --------------//
volatile bool interruptTriggered = false;
void IRAM_ATTR handleInterrupt() {
    interruptTriggered = true;
}

bool clearAllFaults() ;
std::vector<StackVoltageExtrema> AnalyzePerStackStats(const std::vector<StackData>&);
int Balancing();
float voltageToHex(float Vmin);
void checkStatus();
void handleFault();
void sendCAN ();
std::vector<size_t> waitForInputVector();

void setup() {
    Serial.begin(115200);
    while (!Serial);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(Relay,OUTPUT);
    pinMode(Nfault,INPUT);

    //---------------------------------------
    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    bool start = false;
    mySerial.begin(1000000, SERIAL_8N1, 16, 17);
    while (!start){
        bool init = false;
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        init = bms.initialize();
        if (init){
            
            Serial.println("BQ79600 initialized successfully.");
            start = true;
        }
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        delay(500);
    }
        
    //Serial.println(digitalRead(Nfault));
    currentlyBalancingCells.resize(config.num_segments); 
    clearAllFaults();
    attachInterrupt(digitalPinToInterrupt(Nfault), handleInterrupt, FALLING);
    digitalWrite(Relay, HIGH);
    
}
void loop() {
    
    delay(100);
    
    switch (currentState){

        case Active :{
            bms.get_data();
            extremaList = AnalyzePerStackStats(bms.batteryData_pack);
            //sendCAN ();
            
            if(balancing_complete == true){
                Vref = extremaList[0].vmin  ;
                balancing_complete = false;

            }
            for (int stack = 0; stack < bms.NumSegments; stack++) {
                Serial.println("===============================");
                Serial.printf("Stack %d: Cell Voltages:\n", (int)stack);
                for (size_t cell = 0; cell < bms.batteryData_pack[stack].cells.size(); ++cell) {
                    Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", (int)(cell+1), (int)cell, bms.batteryData_pack[stack].cells[cell].voltage);
                }
                Serial.println("===============================");
                Serial.print("volt stack: ");
                Serial.println(extremaList[stack].total_voltage);
                Serial.print("volt AVG: ");
                Serial.println(extremaList[stack].averageVcell);
                Serial.print("V_ref: ");
                Serial.println(Vref,3);
                Serial.print("Temp1: ");
                Serial.println(bms.batteryData_pack[0].gpioTemps[0]);
                Serial.print("Temp2: ");
                Serial.println(bms.batteryData_pack[0].gpioTemps[1]);
                Serial.print("DieTemp: ");
                Serial.println(bms.batteryData_pack[0].dieTemp);
                Serial.println("===============================");

            }
            Serial.println();  // ปิดท้าย 1 บรรทัด
                        
            for (size_t i = 0; i < extremaList.size(); ++i) {
            
                Vdiff = extremaList[i].vmax - extremaList[i].vmin;
                float Vconddiff = extremaList[i].vmax - Vref;
                Serial.printf("Stack %d Voltage Diff: %.2f V\n", (int)i, Vdiff);
                if (Vconddiff > VoltDiffBalance) {
                    if(extremaList[i].current <= 1){
                        currentState = Balance ;
                        ready_balance = true;
                        Serial.println("Balancing required ");
                    } 
                }
                else{
                    Serial.printf("No balancing required for stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                    (int)i, extremaList[i].vmin, extremaList[i].vmax, Vdiff);
                    if (balancing_complete == false){
                        startTime = millis(); // บันทึกเวลาเริ่มต้น
                        balancing_complete = true;
                    }
                }
                
                if (balancing_complete){
                    unsigned long currentMillis = millis();
                    if (currentMillis - startTime >= 60000*5) { // 5 นาที
                        
                        Serial.println("Go to Sleep mode");
                        
                    }
                }
                if (extremaList[i].current >= current_max){
                        currentState = Fault ;
                    }
                
                if (extremaList[i].vmax > Vmaxlimit  ){
                        currentState = Fault ;
                        break;
                    }
                if (extremaList[i].vmin < Vminlimit  ){
                        currentState = Fault ;
                        break;
                    }
                if( extremaList[i].total_voltage > Vmaxlimit_pack){
                    currentState = Fault ;
                    break;
                }
                if( extremaList[i].total_voltage < Vminlimit_pack){
                    currentState = Fault ;
                    break;
                }
                
                /* if( bms.batteryData_pack[i].gpioTemps[1] >= tempMAX){
                    currentState = Fault ;
                    break;
                }
                if( bms.batteryData_pack[i].gpioTemps[1] <= tempMIN){
                    currentState = Fault ;
                    break;
                } */
            }
            checkStatus();
            
            
            //delay(1000);
        }
        break;

        case Balance :{
            std::vector<size_t> filteredCells;
            
            if (ready_balance) {
                ready_balance = false;
                for (size_t i = 0; i < bms.batteryData_pack.size(); ++i) {
                const StackData& stack = bms.batteryData_pack[i];
                
                Serial.println("===============================");
                Serial.printf("Stack %d: Cell Voltages:\n", (int)i);
                for (size_t j = 0; j < stack.cells.size(); ++j) {
                    Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", (int)(j+1), (int)j, stack.cells[j].voltage);
                }
                Serial.println("===============================");
                
                Serial.printf("Balancing Stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                                (int)i, extremaList[i].vmin, extremaList[i].vmax, Vdiff);
                
                    // === ค้นหา cell ที่แรงดันเกิน threshold นี้ ===
                    std::vector<size_t> overVoltageCells;
                    for (size_t j = 0; j < stack.cells.size(); ++j) {
                        float cellVoltage = stack.cells[j].voltage;
                        if (cellVoltage >= Vref) {
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
                        size_t current_cell = overVoltageCells[j];
                        bool isAdjacent = false;

                        for (size_t k = 0; k < filteredCells.size(); ++k) {
                            if (std::abs((int)current_cell - (int)filteredCells[k]) <= 1) {
                                isAdjacent = true;
                                break;
                            }
                        }

                        if (!isAdjacent) {
                            filteredCells.push_back(current_cell);
                        }
                    } 
                    
                    // === แสดงผล cell ที่จะถูกบาลานซ์ ===
                    Serial.println("------------------------------------------------");
                    Serial.printf("Balancing Stack %d (Vmin=%.3f V | Vmax=%.3f V | Vdiff=%.3f V)\n",
                        (int)i, extremaList[i].vmin, extremaList[i].vmax, Vdiff);
                    Serial.print("Selected Cells for Balancing: ");
                    for (size_t idx : filteredCells) {
                        Serial.printf("%d (%.3f V),:", (int)idx, stack.cells[idx].voltage);
                    }
                    
                    Serial.println();
                    Serial.println("------------------------------------------------");  

                    // === เรียกฟังก์ชัน Balance ===
                    if (!filteredCells.empty()) {
                        float hexVal = voltageToHex(Vref);
                        currentlyBalancingCells[i] = filteredCells;  // set สำหรับ stack i
                        
                        bms.BalanceCells(Manual, i, currentlyBalancingCells[i] , extremaList[i].vmin, hexVal);
                    }
                    /* float hexVal = voltageToHex(Vref);
                    Serial.println("wait input filteredCells  ");
                    std::vector<size_t> filteredCells = waitForInputVector();
                    Serial.println("== input filteredCells ==");
                    for (size_t i = 0; i < filteredCells.size(); i++) {
                        Serial.print("filteredCells[");
                        Serial.print(i);
                        Serial.print("] = ");
                        Serial.println(filteredCells[i]);
                    }
                    Serial.println("===============================");
                    bms.BalanceCells(Manual, i, filteredCells , extremaList[i].vmin, hexVal); */

                }
                
                //delay(1000); //
                
                //currentState = Balance;

            }
            bms.get_data();
            extremaList = AnalyzePerStackStats(bms.batteryData_pack);
            //sendCAN ();
            for (int stack = 0; stack < bms.NumSegments; stack++) {
                const StackData& kts = bms.batteryData_pack[stack];
                Serial.println("===============================");
                Serial.printf("Stack %d: Cell Voltages:\n", (int)stack);
                for (size_t cell = 0; cell < bms.batteryData_pack[stack].cells.size(); ++cell) {
                    Serial.printf("  Cell %2d (Index %2d) = %.3f V\n", (int)(cell+1), (int)cell, bms.batteryData_pack[stack].cells[cell].voltage);
                }
                //Vdiff = extremaList[stack].vmax - extremaList[stack].vmin;
                Serial.println("===============================");
                Serial.print("volt stack: ");
                Serial.println(extremaList[stack].total_voltage);
                Serial.print("volt AVG: ");
                Serial.println(extremaList[stack].averageVcell);
                Serial.print("volt diff: ");
                Serial.println(Vdiff,3);
                Serial.print("V_ref: ");
                Serial.println(Vref,3);
                Serial.print("Temp1: ");
                Serial.println(bms.batteryData_pack[0].gpioTemps[0]);
                Serial.print("Temp2: ");
                Serial.println(bms.batteryData_pack[0].gpioTemps[1]);
                Serial.print("DieTemp: ");
                Serial.println(bms.batteryData_pack[0].dieTemp);
                Serial.println("===============================");
                
                    for (size_t idx : filteredCells) {
                        Serial.printf("%d (%.3f V),:", (int)idx, kts.cells[idx].voltage);
                    }

            }
            Serial.println();  // ปิดท้าย 1 บรรทัด
            int status_balance = Balancing();
            if (status_balance == 0x01 ||status_balance == 0x00) {
                currentState = Active ;
                
                }  
        }    
            

        break;

        case Fault : {
            digitalWrite(Relay, LOW);
            handleFault();
            
            while ( extremaList[0].current >= current_max ||
                    extremaList[0].total_voltage > Vmaxlimit_pack||
                    extremaList[0].total_voltage < Vminlimit_pack|| 
                    extremaList[0].vmax >= Vmaxlimit || 
                    extremaList[0].vmin <= Vminlimit ||
                    bms.batteryData_pack[0].gpioTemps[1] >= tempMAX ||
                    bms.batteryData_pack[0].gpioTemps[1] <= tempMIN) 
            {
                delay(100);
                digitalWrite(LED1, HIGH);
                digitalWrite(LED2, LOW);
                bms.get_data();
                extremaList = AnalyzePerStackStats(bms.batteryData_pack);

                if( extremaList[0].total_voltage > Vmaxlimit_pack){
                    statusFault = statusFault | 0x10;
                    
                }
                if( extremaList[0].total_voltage < Vminlimit_pack){
                    statusFault = statusFault | 0x20;
                    
                }
                
                if (extremaList[0].vmax >= Vmaxlimit  ){

                    statusFault = statusFault | 0x01; // Overvoltage
                }
                if (extremaList[0].vmin <= Vminlimit  ){

                    statusFault = statusFault | 0x02; // Undervoltage
                }

                /* if( bms.batteryData_pack[0].gpioTemps[1] >= tempMAX){

                    statusFault = statusFault | 0x04; // Overtemperature
                }
                if( bms.batteryData_pack[0].gpioTemps[1] <= tempMIN){
                    statusFault = statusFault | 0x08; // Undertemperature
                } */
                Serial.print("statusFault: ");
                Serial.println(statusFault, BIN);
                //sendCAN();
                //--------------------------------------------//
                if (extremaList[0].current < current_max && 
                    extremaList[0].total_voltage < Vmaxlimit_pack&&
                    extremaList[0].total_voltage > Vminlimit_pack&&
                    extremaList[0].vmax < Vmaxlimit && 
                    extremaList[0].vmin > Vminlimit &&  
                    bms.batteryData_pack[0].gpioTemps[1] < tempMAX && 
                    bms.batteryData_pack[0].gpioTemps[1] > tempMIN)
                {
                    currentState = Active ;
                    statusFault = 0x00 ;
                    digitalWrite(LED1, LOW);
                    break;
                }

            }
            
            digitalWrite(Relay, HIGH);
        }
        break;
    }
        
}

std::vector<StackVoltageExtrema> AnalyzePerStackStats(const std::vector<StackData>& batteryData_pack) {
std::vector<StackVoltageExtrema> result;

    for (size_t stackIndex = 0; stackIndex < batteryData_pack.size(); ++stackIndex) {
        const StackData& stack = batteryData_pack[stackIndex];

        StackVoltageExtrema extrema;
        extrema.vmin = std::numeric_limits<float>::max();
        extrema.vmax = std::numeric_limits<float>::lowest();
        float current = 0.0000000f;
        float totalVoltage = 0.0f;
        
        /* Serial.printf("=== Stack %d ===\n", (int)stackIndex);
        Serial.println("Cell Voltages:"); */

        for (size_t i = 0; i < stack.cells.size(); ++i) {
            float v = stack.cells[i].voltage;
            totalVoltage += v ;
            //Serial.printf("  Cell %2d: %.3f V\n", (int)i, v);

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
        // total_voltage
        extrema.total_voltage = totalVoltage;
        // พิมพ์ค่าเฉลี่ย
        float averageVcell = totalVoltage / stack.cells.size();
        extrema.averageVcell = averageVcell;
        //Serial.printf("Average Vcell: %.3f V\n", averageVcell);
        
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
        // current
        if (stackIndex == config.num_segments - 1){
            Serial.print("r_shunt :");
            Serial.println(config.r_shunt);
            Serial.print("busbarVolt :");
            Serial.println(stack.busbarVolt);        
            current = stack.busbarVolt / config.r_shunt;
            Serial.print("current :");
            Serial.println(current);
            extrema.current = current;
            result.push_back(extrema);
        }
        

        //----------------------- show CellTemp -----------------------------//
        //Serial.printf("gpioTemps size = %d\n", stack.gpioTemps.size());
        /* Serial.println(" ---------------- Temp ----------------- ");
        for(int CellTemp = 0 ; CellTemp < stack.gpioTemps.size(); CellTemp++) {
            if (CellTemp >= 2) break; // แสดงแค่ 2 ค่าแรก
            Serial.printf("GPIO Temp %d: %.3f C\n", CellTemp, stack.gpioTemps[CellTemp]);
        }
        Serial.printf("Die Temp %d: %.3f C\n",stackIndex ,stack.dieTemp);
        
        Serial.println(" ------------------------------------ "); */
        
    }

    return result;
}
bool clearAllFaults() {
    bool stillFault = false;

    if (bms.checkFaultBase()) {
        bms.clearFault();
        Serial.println("Cleared base fault");
        stillFault = true;
        delay(100); // ให้เวลา BMS ตรวจสอบสถานะ
    }

    if (bms.checkFaultBrigh()) {
        bms.clearFault();
        Serial.println("Cleared brigh fault");
        stillFault = true;
        delay(100); // ให้เวลา BMS ตรวจสอบสถานะ
    }

    return !stillFault;  // ถ้าไม่มี fault เลย -> true
}
void checkStatus() {
    Serial.println("Checking BMS status...");
    digitalWrite(LED2, HIGH);
    if(interruptTriggered) {
        Serial.println("Interrupt triggered! Checking faults...");
        digitalWrite(LED2, LOW);
        currentState = Fault; 
        interruptTriggered = false;   
    }
    //delay(1000); // ให้เวลา BMS ตรวจสอบสถานะ
}
int Balancing(){
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED2, LOW);
    delay(100);
    int result = bms.cheakBalance();
    if (result == 0x80){
        Serial.println("Invalid CB setting");
        return result;
    }else if (result == 0x40){
        Serial.println("NTC thermistor measurement is greater than OTCB_THR OR CBFET temp  is greater than CB TWARN");
        return result;    
    }else if (result == 0x20){
        Serial.println("cell balancing pause status");
        return result;
    }else if (result == 0x10){
        Serial.println("Module balancing");
        return result;
    }else if (result == 0x08){
        Serial.println("At least 1 cell is in active cell balancing");
        return result;
    }else if (result == 0x04){
        Serial.println(" fault detect");
        currentState = Fault;
        return result;
    }else if (result == 0x02){
        Serial.println("Module balancing completed");
        //currentState = Active;   
        return result; 
    }else if (result == 0x01 ){
        Serial.println(" All cell balancing is completed");   
        //currentState = Active; 
        return result; 
    }

    return result ;
}

float voltageToHex(float Vmin) {
    if (Vmin < 2.45f) Vmin = 2.45f;
    if (Vmin > 4.00f) Vmin = 4.00f;

    int step = round((Vmin - 2.45f) / 0.025f) + 1;
    if (step < 1)  step = 1;
    if (step > 63) step = 63;

    float voltage_out = 2.45f + (step - 1) * 0.025f;
    return voltage_out;
}
// ฟังก์ชันรอ input จาก Serial Monitor และแปลงเป็น int[]
// ฟังก์ชันรอรับ input จาก Serial Monitor และคืนค่าเป็น vector
std::vector<size_t> waitForInputVector() {
    String input = "";

    // วนรอจนกว่าจะมี input
    while (true) {
        if (Serial.available() > 0) {
            input = Serial.readStringUntil('\n');
            input.trim();
        break;
        }
    }

    std::vector<size_t> result;
    int startIndex = 0;

    while (startIndex < input.length()) {
        int commaIndex = input.indexOf(',', startIndex);
        if (commaIndex == -1) {
        // ไม่มี comma อีกแล้ว → อ่านตัวสุดท้าย
        String part = input.substring(startIndex);
        part.trim();
        if (part.length() > 0) {
            result.push_back(part.toInt());
        }
        break;
        } else {
            String part = input.substring(startIndex, commaIndex);
            part.trim();
            if (part.length() > 0) {
                result.push_back(part.toInt());
            }
            startIndex = commaIndex + 1;
        }
    }
    return result;
}
void handleFault(){
    Serial.println("Entering Fault Handler...");
    
    while (!clearAllFaults()) {
        Serial.println("Waiting for fault to clear...");
        digitalWrite(LED1, HIGH);
        delay(500); 
        digitalWrite(LED1, LOW);
        delay(100); 
        
    }
    
    Serial.println("All faults cleared.");
    currentState = Active;
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
void sendSummaryCAN(int stackIndex, const StackVoltageExtrema& ext, const StackData& stack) {
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
    uint8_t raw[20]; // 10 cells × 2 bytes
    int idx = 0;
    for (int i = 0; i < bms.NumCellsSeries; i++) {
        uint16_t v = (uint16_t)(stack.cells[i].voltage * 1000.0f);
        raw[idx++] = (v >> 8) & 0xFF;
        raw[idx++] = v & 0xFF;
    }

    // --- สร้าง bitmask สำหรับ cell balancing ---
    uint16_t balanceMask = 0;
    for (size_t cellIndex : filteredCells) {
        if (cellIndex < bms.NumCellsSeries) {
            balanceMask |= (1 << cellIndex);  // set bit
        }
    }

    const uint8_t fragLens[3] = {8, 8, 7};  // ← เพิ่มเป็น 7 bytes สำหรับ frame3
    int offset = 0;
    for (int frag = 0; frag < 3; frag++) {
        struct can_frame frame;
        frame.can_id  = CAN_ID_DETAIL_BASE + (stackIndex << 4) + frag;
        frame.can_dlc = fragLens[frag];
        memcpy(frame.data, &raw[offset], fragLens[frag]);

        // --- เพิ่ม balanceMask + statusFault ที่ frame3 (CAN ID 0x502) ---
        if (frag == 2) {  
            frame.data[4] = (balanceMask >> 8) & 0xFF; // high byte
            frame.data[5] = balanceMask & 0xFF;        // low byte
            frame.data[6] = statusFault;               // statusFault 1 byte
        }

        sendFrameWithRetry(frame);
        offset += fragLens[frag];
    }
}

// ----- Main sendCAN function -----
void sendCAN() {
    unsigned long currentMillis = millis();

    for (int i = 0; i < bms.NumSegments; i++) {
        // Summary: high priority, send every loop
        sendSummaryCAN(i, extremaList[i], bms.batteryData_pack[i]);

        // Detail: low priority, send slower
        if (loggingEnabled && (currentMillis - lastDetailTime >= detailInterval)) {
            sendDetailCAN(i, bms.batteryData_pack[i]);
        }
    }

    // Update lastDetailTime if we sent detail
    if (loggingEnabled && (currentMillis - lastDetailTime >= detailInterval)) {
        lastDetailTime = currentMillis;
    }
}





