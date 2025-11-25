#include "BQ79600.h"

Crc16 crc;
BQ79600::BQ79600(HardwareSerial &serial, uint32_t baud, int tx_pin, BQ79600config config): uart(&serial), baudRate(baud), tx_pin_(tx_pin)
    {
        NumSegments         =   config.num_segments;
        NumCellsSeries      =   config.num_cells_series;
        NumThermistors      =   config.num_thermistors;
        ShuntResistance     =   config.shunt_resistance;
        // clear buffers
        memset(voltageBuffer, 0, sizeof(voltageBuffer));
        memset(dieTempBuffer, 0, sizeof(dieTempBuffer));
        memset(busbarBuffer, 0, sizeof(busbarBuffer));
        memset(gpioTempBuffer, 0, sizeof(gpioTempBuffer));
        
        // clear indexes
        memset(voltageBufferIndex, 0, sizeof(voltageBufferIndex));
        memset(dieTempBufferIndex, 0, sizeof(dieTempBufferIndex));
        memset(busbarBufferIndex, 0, sizeof(busbarBufferIndex));
        memset(gpioTempBufferIndex, 0, sizeof(gpioTempBufferIndex));
        
        // clear counts
        memset(voltageBufferCount, 0, sizeof(voltageBufferCount));
        memset(dieTempBufferCount, 0, sizeof(dieTempBufferCount));
        memset(busbarBufferCount, 0, sizeof(busbarBufferCount));
        memset(gpioTempBufferCount, 0, sizeof(gpioTempBufferCount));
    }   

bool BQ79600::initialize() {
    bool autoAddressed = false;
    wakeUp();                               // Wake up the BQ79600  and BQ79612
    
    data_arr_[0] = 0x02 ; //  restart
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::CONTROL1 ,data_arr_);
    delayMicroseconds(500);
    
    autoAddressed = AutoAddressing();                                   // Auto address the BQ79600 devices

    /* data_arr_[0] = 0x1C ; //  2 bit stop bits uart
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR, RegisterAddress::DEV_CONF1 ,data_arr_);  
    delay(5);
    data_arr_[0] = 0x5C ; //  2 bit stop bits uart
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::DEV_CONF ,data_arr_);   
    delay(5);
    beginUart2();         //setting Uart mcu  */

    data_arr_[0] = 0x26 ; //  COMM_TIMEOUT config
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR, RegisterAddress::COMM_TIMEOUT ,data_arr_);  
    
    //---------------------------------------------------------------------------------------//
    config_MainADC(NumCellsSeries,NumSegments);         // Configure the main ADC 
    //config_AuxADC() ;                                 // Configure the aux ADC
    config_Fault();                                     // Configure the fault settings
    config_OT_UT();                                     // Configure the Over Temperature and Under Temperature settings
    config_OV_UV();    
    if (autoAddressed){
        return true; // Auto addressing successful
    }else if(!autoAddressed){
        return false; // Auto addressing failed
    }
    
}

void BQ79600::beginUart1()
{
    uart->begin(baudRate, SERIAL_8N1, 16, tx_pin_);
    delay(10); // wait UART stable
    
}
void BQ79600::beginUart2()
{
    uart->begin(baudRate, SERIAL_8N2, 16, tx_pin_);
    delay(10); // wait UART stable
    
}

void BQ79600::wakeUp() {
    wakePing();
    wakePing();
    delay(5);
    beginUart1();             //setting Uart mcu
    data_arr_[0] = 0x20 ;
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR , RegisterAddress::CONTROL1,data_arr_);   // Send WAKE tone up the stack bq79612
    delay(5); // tAFE_SETTLE 4 ms
    
}

void BQ79600::wakePing() {
    uart->end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(2700);        // low 2.75ms 
    digitalWrite(tx_pin_, HIGH);
    delayMicroseconds(3500);        //  delay 3.5ms  
    
}



void BQ79600::sendCommandTo(RequestType req_type, byte data_size, byte dev_addr,RegisterAddress reg_addr, uint8_t data[]) {
    
    uint8_t bq_frame_[50];  
    data_size -= 1;
    
    bool StackOrBroad = (req_type == RequestType::StackRead)      || 
                        (req_type == RequestType::StackWrite)     || 
                        (req_type == RequestType::BroadcastRead)  || 
                        (req_type == RequestType::BroadcastWrite) || 
                        (req_type == RequestType::BroadcastWrite_REV);
    
    size_t frame_len = 6 + data_size + (StackOrBroad ? 0 : 1);
    
    // Build frame
    bq_frame_[0] = static_cast<byte>(req_type) | (data_size & 0b00001111);
    
    if (!StackOrBroad) {
        bq_frame_[1] = dev_addr;
    }
    
    bq_frame_[1 + (!StackOrBroad)] = static_cast<uint16_t>(reg_addr) >> 8;
    bq_frame_[2 + (!StackOrBroad)] = static_cast<uint16_t>(reg_addr) & 0xFF;
    
    for (int i = 0; i <= data_size; i++) {
        bq_frame_[3 + i + (!StackOrBroad)] = data[i];
    }
    
    // Calculate CRC
    uint16_t command_crc = crc.Modbus(bq_frame_, 0, 4 + data_size + (!StackOrBroad));
    
    bq_frame_[4 + data_size + (!StackOrBroad)] = command_crc & 0xFF;
    bq_frame_[5 + data_size + (!StackOrBroad)] = command_crc >> 8;
    
    // Debug output
    Serial.println("Command: ");
    for (int i = 0; i < frame_len; i++) {  // ✅ ใช้ frame_len
        Serial.printf("%02X ", bq_frame_[i]);
    }
    Serial.println();
    
    // Clear RX buffer
    int cleared = 0;
    while (uart->available()) {
        uart->read();
        cleared++;
    }
    if (cleared > 0) {
        Serial.printf("🧹 Pre-cleared %d old bytes\n", cleared);
    }
    
    // 
    uart->write(bq_frame_, frame_len);  // ← ใช้ frame_len
    uart->flush();
    
    // Delay ตาม request type
    if (req_type == RequestType::StackRead || req_type == RequestType::StackWrite) {
        uint16_t stackDelay = (5 + 1) * NumSegments;
        delay(stackDelay);
        Serial.printf("⏱️ Stack delay: %d ms\n", stackDelay);
    } else {
        delay(5);
    }
}

void BQ79600::config_MainADC(uint8_t numcell,uint8_t numStack) {
    if (numcell < 6 || numcell > 16) {
        Serial.println("Invalid numcell: must be between 6 and 16");
        return;
    }
     //----------------------------- reset MAIN ADC --------------------------------------------------///
    Serial.println("");
    Serial.println(" Reset LPF_BB_EN, LPF_VCELL,MAIN_MODE command  ");
    data_arr_[0] = 0x00 ; //  reset LPF_BB_EN, LPF_VCELL ,MAIN_MODE
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL1 ,data_arr_);   // Stop ADC
    delayMicroseconds(500);   
    
    // ---------- configure enable TSREF ---------- //
    Serial.println("");
    Serial.println(" Set configure enable TSREF command ");
    data_arr_[0] = 0x01;
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::CONTROL2 , data_arr_);

    // Set the ADC configuration cells to active //
    Serial.println("");
    Serial.println(" Set all used cells to active command ");
    uint8_t A = numcell - 6; // 0x00 - 0x0A
    data_arr_[0] = A;
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ACTIVE_CELL , data_arr_);  //  Set all used cells to active

    // set GPIO 1,2 
    Serial.println("");
    Serial.println(" Set GPIO 1,2 command ");
    data_arr_[0] = 0x09 ; //  Configures GPIO1 ,GPIO2  0x09
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF1 ,data_arr_);

    // set GPIO 3,4 
    Serial.println("");
    Serial.println(" Set GPIO 3,4 command ");
    data_arr_[0] = 0x05 ; //  Configures GPIO3  as output low 
    data_arr_[0] = data_arr_[0] | (1 << 3)  | (1 << 5) ; //  Configures GPIO4 as output low 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF2 ,data_arr_);

    // set GPIO 5,6
    Serial.println("");
    Serial.println(" Set GPIO 5,6 command "); 
    data_arr_[0] = 0x00 ; //  Configures GPIO5 ,GPIO6  as  disabled, high-Z 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF3 ,data_arr_);

    // set GPIO 7,8
    Serial.println("");
    Serial.println(" Set GPIO 7,8 command ");
    data_arr_[0] = 0x00 ; //  Configures GPIO7 ,GPIO8  as  disabled, high-Z 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF4 ,data_arr_);

    // set delay
    Serial.println("");
    Serial.println(" set delay for start ADC command ");
    uint8_t delayCode = calcADC_DLY(numStack);
    data_arr_[0] = delayCode; //  ADC delay code
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CONF2 ,data_arr_);

    // set  Configures the post ADC low-pass & Configures the post main SAR ADC low-pass filter cut-off frequency for BBP/N
    Serial.println("");
    Serial.println(" Configures low-pass filter cut-off frequency command ");
    data_arr_[0] = 0b00000000 ; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CONF1 ,data_arr_);

        //----------------------------- start MAIN ADC --------------------------------------------------///
    Serial.println("");
    Serial.println(" Set LPF_BB_EN, LPF_VCELL,MAIN_MODE command  ");
    Serial.println("Start ReadMainADC ");
    data_arr_[0] = 0x0E ; //  LPF_BB_EN, LPF_VCELL ,MAIN_MODE
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL1 ,data_arr_);   // START ADC
    delayMicroseconds(3500);    //  wait for ADC to start 
    delayMicroseconds(192*8);   //  wait for ADC finished 
}  
void BQ79600::config_AuxADC() {
    
    // ---------- configure Selects which AUXCELL ---------- //
    data_arr_[0] = 0x00; // Run all active cell channels set by ACTIVE_CELL_CONF register
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL2 ,data_arr_);

    // ---------- configure Selects GPIO AUX  ---------- //
    data_arr_[0] = 0x06; // Run all active GPIO channels set by GPIO_CONF register
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL3 ,data_arr_);
    delayMicroseconds(192*8);   //  wait for ADC finished 
}
void BQ79600::BalanceCells(uint8_t mode, size_t stack, const size_t targetCells[], uint8_t targetCellsCount, uint8_t hexThreshold) {
    
    Serial.printf("\n=== Balancing Stack %d (Mode: %d) ===\n", stack, mode);

    //  1. ตั้งค่า threshold voltage
    data_arr_[0] = hexThreshold;
    sendCommandTo(RequestType::SingleWrite, 1, stack,RegisterAddress::VCB_DONE_THRESH, data_arr_);
    
    
    //  2. เขียนค่าเวลาลง CB_CELLx_CTRL สำหรับทุก cell
    Serial.println("\nConfiguring cell balance time:");
    
    for (int i = 0; i < NumCellsSeries; i++) {
        // คำนวณ register address (CB_CELL1_CTRL - i)
        RegisterAddress reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) - i);
        bool shouldBalance = false;

        if(mode == 0){
            //  เช็คว่า cell นี้อยู่ใน targetCells[] หรือไม่
            for (uint8_t j = 0; j < targetCellsCount; ++j) {
                if (targetCells[j] == i) {
                    shouldBalance = true;
                    break;
                }
            }
        }
        else if(mode == 1){
            // Auto mode: balance all cells above threshold
            shouldBalance = true;
        }
            
        
        // set time to balance
        if (shouldBalance) {
            data_arr_[0] = 0x04;  // 300 seconds (balance ON)
            Serial.printf("   Cell %d → Balancing ON  (Reg 0x%04X, Time: 0x04)\n", i, static_cast<uint16_t>(reg));
        } else {
            data_arr_[0] = 0x00;  // 0 seconds (balance OFF)
            Serial.printf("   Cell %d → Balancing OFF (Reg 0x%04X, Time: 0x00)\n", i, static_cast<uint16_t>(reg));
        }
        
        // send command to IC
        sendCommandTo(RequestType::SingleWrite, 1, stack, reg, data_arr_);
    }
    
    //  3. ตั้งค่า duty cycle (BAL_CTRL1)
    Serial.println("\nConfiguring balance control:");
    data_arr_[0] = 0x04;  // 60 seconds duty cycle
    sendCommandTo(RequestType::SingleWrite, 1, stack,RegisterAddress::BAL_CTRL1, data_arr_);
    Serial.println("   Set BAL_CTRL1 = 0x04 (60s duty cycle)");
    
    //  4. ตั้งค่า CBFET (DEV_CONF) for allow an adjacent CB FET to be turned on in manual CB control
    /* data_arr_[0] = 0x14;
    sendCommandTo(RequestType::SingleWrite, 1, stack,RegisterAddress::DEV_CONF, data_arr_);
    Serial.println("  ✓ Set DEV_CONF = 0x14 (CBFET config)"); */
    
    //  5. ตั้งค่า OV/UV mode (OVUV_CTRL)
    data_arr_[0] = 0x05;  // Run OV and UV round robin
    sendCommandTo(RequestType::SingleWrite, 1, stack, RegisterAddress::OVUV_CTRL, data_arr_);
    Serial.println("   Set OVUV_CTRL = 0x05 (OV/UV round robin)");
    
    //  6. เริ่ม balancing (BAL_CTRL2)
    data_arr_[0] = 0x22;  // Base config: FLTSTOP_EN, BAL_GO, OTCB_EN
    
    if (mode == 1) {
        data_arr_[0] |= (1 << 0);  // เพิ่ม Auto mode
        Serial.println("   Set BAL_CTRL2 = 0x23 (Auto balancing mode)");
    } else {
        Serial.println("   Set BAL_CTRL2 = 0x22 (Manual balancing mode)");
    }
    
    sendCommandTo(RequestType::SingleWrite, 1, stack, RegisterAddress::BAL_CTRL2, data_arr_);
    
    delay(1);
    
    Serial.println("===========================================\n");
}

void BQ79600::pauseBalanceCells(uint8_t stack) {
    
    Serial.printf("\n=== Pause Balancing Stack %d ", stack);
    data_arr_[0] = 0x00 ;
    sendAndReceive(RequestType::SingleRead, 1, stack, RegisterAddress::BAL_CTRL2, data_arr_, response);
    delay(1);

    data_arr_[0] = response[4] | (1 << 6); // Set PAUSE bit
    sendCommandTo(RequestType::SingleWrite, 1, stack, RegisterAddress::BAL_CTRL2, data_arr_);
    delay(1);
    
    Serial.println("===========================================\n");
}

void BQ79600::unpauseBalanceCells(uint8_t stack) {
    
    Serial.printf("\n=== unPause Balancing Stack %d ", stack);
    
    data_arr_[0] = 0x00 ;
    sendAndReceive(RequestType::SingleRead, 1, stack, RegisterAddress::BAL_CTRL2, data_arr_, response);
    delay(1);

    data_arr_[0] = response[4] & ~(1 << 6); // Clear PAUSE bit
    sendCommandTo(RequestType::SingleWrite, 1, stack, RegisterAddress::BAL_CTRL2, data_arr_);
    delay(1);
    
    Serial.println("===========================================\n");
}

void BQ79600::config_OV_UV(){
    //  Configures the overvoltage and undervoltage thresholds 
    data_arr_[0] = 0x23; //  Set the overvoltage threshold 4.2 V
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OV_THRESH , data_arr_);
    
    data_arr_[0] = 0x00; //  Set the undervoltage threshold 3.1V
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::UV_THRESH , data_arr_);
    
     // optional: stop cell voltage Threshold for balancing
    data_arr_[0] = 0x01; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::VCB_DONE_THRESH , data_arr_);
    
    // UV_disable1 CELL9 - CELL16
    data_arr_[0] = 0x00; //  
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::UV_DISABLE1 , data_arr_);

    // UV_disable2 CELL1 - CELL8
    data_arr_[0] = 0x00; //  
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::UV_DISABLE2 , data_arr_);

    // configures mode of OV/UV detection  
    data_arr_[0] = 0x05; // Run the OV and UV round robin and Go
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OVUV_CTRL , data_arr_);
    Serial.println("  ");
    Serial.println(" OV/UV configured ");
    
}
void BQ79600::config_OT_UT(){
    // Configures the overtemperature and undertemperature thresholds
    data_arr_[0] = 0b11100000; //  
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_THRESH , data_arr_); 

    /* // Configures the overtemperature and undertemperature thresholds for run again
    data_arr_[0] = 0x00; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_CTRL , data_arr_); */

    // Configures mode of OT/UT detection
    data_arr_[0] = 0x00; // Run the OT and UT round robin and Go
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_CTRL , data_arr_);
    Serial.println(" OT/UT configured ");

}

void BQ79600::config_Fault(){
    data_arr_[0] = 0x40; 
    sendCommandTo(RequestType::StackWrite, 1, 0x01, RegisterAddress::FAULT_MSK2 , data_arr_);  
    /* data_arr_[0] = 0x01 ;
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR, RegisterAddress::FAULTM_MSK ,data_arr_);  */
    // Configures BQ79600 and BQ79612 enable NFAULT , FCOMM_EN , HB_TX_EN ,FTO_EN
    data_arr_[0] = 0x57; //  Enable NFAULT , FCOMM_EN , HB_TX_EN , FTO_EN 
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::DEV_CONF , data_arr_);
    Serial.println(" Fault configured ");
    
}
    
void BQ79600::get_data() {
    uint8_t response[256];
    
    // ✅ Temporary static arrays สำหรับเก็บข้อมูลแต่ละรอบ
    double voltages[MAX_STACKS][MAX_CELLS_PER_STACK];
    double dieTemps[MAX_STACKS];
    double tsref[MAX_STACKS];
    double gpioTemps[MAX_STACKS][MAX_TEMPS_PER_STACK];
    double busbar_top;
    
    RegisterAddress reg;
    bool ok;
    
    //  Loop เก็บข้อมูลหลายรอบ
    for (int round = 0; round < averageWindow; ++round) {
        
        Serial.printf("=== Reading Round %d/%d ===\n", round + 1, averageWindow);
        
        // ---------- 1. Read Cell Voltages ----------
        Serial.println("--------------------------------");
        Serial.println("Reading Cell Voltages...");
        
        reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::VCELL16_HI) + (16 - NumCellsSeries) * 2);
        //data_arr_[0] = 0x11; // Dummy data for read
        ok = sendAndReceive(RequestType::StackRead, NumCellsSeries * 2, DEV_ADDR, reg, data_arr_, response);
        
        if (ok) {
            size_t response_size = NumSegments * NumCellsSeries * 2;
            parseCellVoltages(response, response_size, voltages, NumSegments, NumCellsSeries);
        }
        delayMicroseconds(500);
        
        // ---------- 2. Read TSREF ----------
        Serial.println("--------------------------------");
        Serial.println("Reading TSREF...");
        
        ok = sendAndReceive(RequestType::StackRead, 2, DEV_ADDR, RegisterAddress::TSREF_HI, data_arr_, response);
        
        if (ok) {
            size_t response_size = NumSegments * 2;
            parseTSREF(response, response_size, tsref, NumSegments);
        }
        delayMicroseconds(500);
        
        // ---------- 3. Read BUSBAR (Top Stack Only) ----------
        Serial.println("--------------------------------");
        Serial.println("Reading BUSBAR Voltage...");
        
        ok = sendAndReceive(RequestType::SingleRead, 2, NumSegments, RegisterAddress::BUSBAR_HI, data_arr_, response);
        
        if (ok) {
            busbar_top = parseBUSBAR_Single(response, sizeof(response));
            Serial.printf("Top Stack BUSBAR: %.2f mV\n", busbar_top);
        } else {
            Serial.println("⚠️ Failed to read BUSBAR from Top Stack!");
            busbar_top = -999.0;
        }
        delayMicroseconds(500);
        
        // ---------- 4. Read GPIO Temp ----------
        Serial.println("--------------------------------");
        Serial.println("Reading GPIO Temp...");
        
        reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::GPIO1_HI));
        ok = sendAndReceive(RequestType::StackRead, NumThermistors * 2, DEV_ADDR, reg, data_arr_, response);
        
        if (ok) {
            size_t response_size = NumSegments * NumThermistors * 2;
            parseTemp(response, response_size, tsref, gpioTemps, NumSegments, NumThermistors);
        }
        delayMicroseconds(500);
        
        // ---------- 5. Read Die Temp ----------
        Serial.println("");
        Serial.println("Reading Die Temp...");
        
        reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::DIETEMP1_HI));
        ok = sendAndReceive(RequestType::StackRead, 2, DEV_ADDR, reg, data_arr_, response);
        
        if (ok) {
            size_t response_size = NumSegments * 2;
            parsedie_Temp(response, response_size, dieTemps, NumSegments);
        }
        delayMicroseconds(500);
        
        // ✅ เก็บข้อมูลลง Circular Buffers
        for (uint8_t stack = 0; stack < NumSegments; ++stack) {
            // Cell Voltages (reverse order: VCELL16 → VCELL1)
            for (uint8_t cell = 0; cell < NumCellsSeries; ++cell) {
                float v = voltages[stack][NumCellsSeries - 1 - cell];
                addVoltageToBuffer(stack, cell, v);
            }
            
            // Die Temperature
            addDieTempToBuffer(stack, dieTemps[stack]);
            
            // Busbar (Top Stack Only)
            if (stack == NumSegments - 1) {
                addBusbarToBuffer(stack, busbar_top);
            }
            
            // GPIO Temperatures
            for (uint8_t t = 0; t < NumThermistors; ++t) {
                addGpioTempToBuffer(stack, t, gpioTemps[stack][t]);
            }
        }
        
        Serial.printf("=== Round %d Complete ===\n\n", round + 1);
    }
    
    //  คำนวณค่าเฉลี่ยและเก็บลง batteryData_pack
    Serial.println("=== Calculating Averages ===");
    
    for (uint8_t stack = 0; stack < NumSegments; ++stack) {
        batteryData_pack[stack].numCells = NumCellsSeries;
        batteryData_pack[stack].numTemps = NumThermistors;
        
        // Cell Voltages
        for (uint8_t cell = 0; cell < NumCellsSeries; ++cell) {
            batteryData_pack[stack].cells[cell].voltage = getAverageVoltage(stack, cell);
            Serial.printf("Stack %d Cell %d: %.3f V (avg of %d samples)\n", stack, cell, batteryData_pack[stack].cells[cell].voltage, voltageBufferCount[stack][cell]);
        }
        
        // Die Temperature
        batteryData_pack[stack].dieTemp = getAverageDieTemp(stack);
        Serial.printf("Stack %d Die Temp: %.2f °C\n", stack, batteryData_pack[stack].dieTemp);
        
        // Busbar (Top Stack Only)
        if (stack == NumSegments - 1) {
            batteryData_pack[stack].busbarVolt = getAverageBusbar(stack);
            Serial.printf("Stack %d Busbar: %.2f mV\n", stack, batteryData_pack[stack].busbarVolt);
        } else {
            batteryData_pack[stack].busbarVolt = 0.0;
        }
        
        // GPIO Temperatures
        for (uint8_t therm = 0; therm < NumThermistors; ++therm) {
            batteryData_pack[stack].gpioTemps[therm] = getAverageGpioTemp(stack, therm);
            Serial.printf("Stack %d GPIO %d: %.2f °C\n", stack, therm, batteryData_pack[stack].gpioTemps[therm]);
        }
        
        Serial.println();
    }
    Serial.println("=== Converting Physical → Logical Order ===");
    
    // Temporary buffer สำหรับเก็บข้อมูล
    StackData tempBuffer[MAX_STACKS];
    
    // คัดลอกข้อมูลจาก batteryData_pack → tempBuffer
    for (uint8_t i = 0; i < NumSegments; i++) {
        memcpy(&tempBuffer[i], &batteryData_pack[i], sizeof(StackData));
    }
    
    // แปลง Physical → Logical แล้วเขียนกลับ batteryData_pack
    for (uint8_t physicalIdx = 0; physicalIdx < NumSegments; physicalIdx++) {
        // คำนวณ logical index (กลับลำดับ)
        uint8_t logicalIdx = (NumSegments - 1) - physicalIdx;
        
        // คัดลอกข้อมูลจาก tempBuffer → batteryData_pack (logical order)
        memcpy(&batteryData_pack[logicalIdx],&tempBuffer[physicalIdx],sizeof(StackData));
        
        Serial.printf("Physical Stack %d → Logical Stack %d (Address: %d)\n",physicalIdx, logicalIdx, logicalIdx + 1);
    }
    
    Serial.println("=== Physical → Logical Conversion Complete ===\n");
    Serial.println("=== get_data() Complete ===\n");
}

void BQ79600::parsedie_Temp(const uint8_t response[], size_t response_size,double dieTemps[], uint8_t numStacks) {
    
    size_t expectedSize = numStacks * 2;
    
    if (response_size < expectedSize) {
        Serial.printf(" parsedie_Temp: Response too short! Got %d, expected %d\n",response_size, expectedSize);
        return;
    }
    
    size_t index = 0;
    
    for (uint8_t stack = 0; stack < numStacks; stack++) {
        uint16_t rawValue = (response[index] << 8) | response[index + 1];
        index += 2;
        dieTemps[stack] = convertTo_dietemp(rawValue);
    }
}

void BQ79600::parseTemp(const uint8_t response[], size_t response_size,const double tsref[],double temps[][MAX_TEMPS_PER_STACK],uint8_t numStacks, uint8_t numNTC) {
    
    size_t expectedSize = numStacks * numNTC * 2;
    
    if (response_size < expectedSize) {
        Serial.printf(" parseTemp: Response too short! Got %d, expected %d\n",response_size, expectedSize);
        return;
    }
    
    size_t index = 0;
    
    for (uint8_t stack = 0; stack < numStacks; stack++) {
        for (uint8_t ntc = 0; ntc < numNTC; ntc++) {
            uint16_t rawValue = (response[index] << 8) | response[index + 1];
            index += 2;
            
            double gpioVolt = convertTo_VoltGPIO(rawValue);
            double VREF = tsref[stack];
            // cheak voltage 
            if (gpioVolt >= VREF || gpioVolt <= 0) {
                temps[stack][ntc] = -273.15;  // Invalid
                Serial.printf("⚠️ Stack %d NTC %d: Invalid voltage (%.3fV >= %.3fV)\n",stack, ntc, gpioVolt, VREF);
                continue;
            }
            const double R1 = 10000.0;  // 10kΩ pull-up resistor
            double Rntc = (gpioVolt * R1) / (VREF - gpioVolt);
            
            // แปลงเป็นอุณหภูมิ
            temps[stack][ntc] = calculateTemperatureFromResistance(Rntc);
        }
    }
}

// Helper function
double BQ79600::calculateTemperatureFromResistance(double resistance) {
    // Steinhart-Hart equation (ปรับค่า A, B, C ตาม NTC ที่ใช้)
    const double A = 0.001129148;
    const double B = 0.000234125;
    const double C = 0.0000000876741;
    /* const double A = 0.0011216666;
    const double B = 0.0002374118;
    const double C = 0.00000006646871;
    const double A = 0.003354016;
    const double B = 0.0002569850;
    const double C = 0.000002620131; */
    
    if (resistance <= 0) return -273.15;  // Invalid
    
    double lnR = log(resistance);
    double temp_K = 1.0 / (A + B * lnR + C * lnR * lnR * lnR);
    return temp_K - 273.15;  // Convert to Celsius
}
void BQ79600::parseTSREF(const uint8_t response[], size_t response_size,double tsref[], uint8_t numStacks) {
    
    size_t expectedSize = numStacks * 2;
    
    if (response_size < expectedSize) {
        Serial.printf(" parseTSREF: Response too short! Got %d, expected %d\n",response_size, expectedSize);
        return;
    }
    
    size_t index = 0;
    
    for (uint8_t stack = 0; stack < numStacks; stack++) {
        uint16_t rawValue = (response[index] << 8) | response[index + 1];
        index += 2;
        tsref[stack] = convertTo_TREFVolt(rawValue);
    }
}
// ✅ ฟังก์ชันใหม่สำหรับ Single Read (Top Stack)
double BQ79600::parseBUSBAR_Single(const uint8_t response[], size_t response_size) {
    
    if (response_size < 6) {  // Header(4) + Data(2)
        Serial.printf(" parseBUSBAR_Single: Response too short! Got %d, expected 6\n",response_size);
        return 0.0;
    }
    
    // Single Read: response[4-5] = data
    uint16_t rawValue = (response[4] << 8) | response[5];
    return convertTo_BUSBARVolt(rawValue) * 1000.0;  // Convert to mV
}

//  Stack Read
/* std::vector<double> BQ79600::parseBUSBAR(const std::vector<byte>& response, uint8_t numStacks) {
    std::vector<double> BUSBAR(numStacks);
    
    const size_t dataSizePerStack = 2;  //  ไม่รวม header/CRC
    
    if (response.size() < dataSizePerStack * numStacks) {
        Serial.printf("⚠️ parseBUSBAR: Response too small (%d bytes, expected %d)\n", 
                      response.size(), dataSizePerStack * numStacks);
        return BUSBAR;
    }

    for (uint8_t stack = 0; stack < numStacks; ++stack) {
        size_t index = stack * dataSizePerStack;
        
        uint16_t raw = (response[index] << 8) | response[index + 1];
        double BUSBAR_volt = convertTo_BUSBARVolt(raw) * 1000.0;  // mV
        
        BUSBAR[stack] = BUSBAR_volt;
        Serial.printf("Stack %d BUSBAR: %.2f mV (raw: 0x%04X)\n", stack, BUSBAR_volt, raw);
    }

    return BUSBAR;
} */


void BQ79600::parseCellVoltages(const uint8_t response[], size_t response_size,double voltages[][MAX_CELLS_PER_STACK], uint8_t numStacks, uint8_t numCells) {
    
    size_t expectedSize = numStacks * numCells * 2;
    
    if (response_size < expectedSize) {
        Serial.printf("⚠️ parseCellVoltages: Response too short! Got %d, expected %d\n",response_size, expectedSize);
        return;
    }
    
    size_t index = 0;
    
    for (uint8_t stack = 0; stack < numStacks; stack++) {
        for (uint8_t cell = 0; cell < numCells; cell++) {
            uint16_t rawValue = (response[index] << 8) | response[index + 1];
            index += 2;
            voltages[stack][cell] = convertTo_VoltagCell(rawValue);
        }
    }
}
//  voltage
void BQ79600::addVoltageToBuffer(uint8_t stack, uint8_t cell, float voltage) {
    if (stack >= MAX_STACKS || cell >= MAX_CELLS_PER_STACK) return;
    
    //  เก็บข้อมูลแบบ circular buffer
    uint8_t idx = voltageBufferIndex[stack][cell];
    voltageBuffer[stack][cell][idx] = voltage;
    
    //  เลื่อน index
    voltageBufferIndex[stack][cell] = (idx + 1) % AVERAGE_WINDOW;
    
    //  เพิ่ม count (สูงสุด AVERAGE_WINDOW)
    if (voltageBufferCount[stack][cell] < AVERAGE_WINDOW) {
        voltageBufferCount[stack][cell]++;
    }
}

//  เพิ่มข้อมูล die temperature
void BQ79600::addDieTempToBuffer(uint8_t stack, float temp) {
    if (stack >= MAX_STACKS) return;
    
    uint8_t idx = dieTempBufferIndex[stack];
    dieTempBuffer[stack][idx] = temp;
    
    dieTempBufferIndex[stack] = (idx + 1) % AVERAGE_WINDOW;
    
    if (dieTempBufferCount[stack] < AVERAGE_WINDOW) {
        dieTempBufferCount[stack]++;
    }
}

//  เพิ่มข้อมูล busbar voltage
void BQ79600::addBusbarToBuffer(uint8_t stack, float voltage) {
    if (stack >= MAX_STACKS) return;
    
    uint8_t idx = busbarBufferIndex[stack];
    busbarBuffer[stack][idx] = voltage;
    
    busbarBufferIndex[stack] = (idx + 1) % AVERAGE_WINDOW;
    
    if (busbarBufferCount[stack] < AVERAGE_WINDOW) {
        busbarBufferCount[stack]++;
    }
}

//  เพิ่มข้อมูล GPIO temperature
void BQ79600::addGpioTempToBuffer(uint8_t stack, uint8_t thermistor, float temp) {
    if (stack >= MAX_STACKS || thermistor >= MAX_TEMPS_PER_STACK) return;
    
    uint8_t idx = gpioTempBufferIndex[stack][thermistor];
    gpioTempBuffer[stack][thermistor][idx] = temp;
    
    gpioTempBufferIndex[stack][thermistor] = (idx + 1) % AVERAGE_WINDOW;
    
    if (gpioTempBufferCount[stack][thermistor] < AVERAGE_WINDOW) {
        gpioTempBufferCount[stack][thermistor]++;
    }
}

//  คำนวณค่าเฉลี่ย voltage
float BQ79600::getAverageVoltage(uint8_t stack, uint8_t cell) {
    if (stack >= MAX_STACKS || cell >= MAX_CELLS_PER_STACK) return 0.0f;
    
    uint8_t count = voltageBufferCount[stack][cell];
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += voltageBuffer[stack][cell][i];
    }
    
    return sum / count;
}

//  คำนวณค่าเฉลี่ย die temperature
float BQ79600::getAverageDieTemp(uint8_t stack) {
    if (stack >= MAX_STACKS) return 0.0f;
    
    uint8_t count = dieTempBufferCount[stack];
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += dieTempBuffer[stack][i];
    }
    
    return sum / count;
}

//  คำนวณค่าเฉลี่ย busbar voltage
float BQ79600::getAverageBusbar(uint8_t stack) {
    if (stack >= MAX_STACKS) return 0.0f;
    
    uint8_t count = busbarBufferCount[stack];
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += busbarBuffer[stack][i];
    }
    
    return sum / count;
}

//  คำนวณค่าเฉลี่ย GPIO temperature
float BQ79600::getAverageGpioTemp(uint8_t stack, uint8_t thermistor) {
    if (stack >= MAX_STACKS || thermistor >= MAX_TEMPS_PER_STACK) return 0.0f;
    
    uint8_t count = gpioTempBufferCount[stack][thermistor];
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += gpioTempBuffer[stack][thermistor][i];
    }
    
    return sum / count;
}


double BQ79600::convertTo_VoltagCell(uint16_t rawValue) {
    int16_t signed_value = static_cast<int16_t>(rawValue);
    return signed_value * 190.73e-6;
    
}

double BQ79600::convertTo_VoltGPIO(uint16_t rawValue) {
    int16_t signed_value = static_cast<int16_t>(rawValue);
    return signed_value *  152.59e-6;
    
}


double BQ79600::convertTo_dietemp(uint16_t rawValue) {
    int16_t signed_value = static_cast<int16_t>(rawValue);
    return signed_value * 0.025;
    
}

double BQ79600::convertTo_TREFVolt(uint16_t rawValue) {
    int16_t signed_value = static_cast<int16_t>(rawValue);
    return signed_value * 169.54e-6;
    
}

double BQ79600::convertTo_BUSBARVolt(uint16_t rawValue) {
    int16_t signed_value = static_cast<int16_t>(rawValue);
    return signed_value * 30.52e-6;
    
}

uint8_t BQ79600::calcADC_DLY(uint8_t numDevices) {
    float afeSettle = 4000.0f ;
    float propDelay    = (numDevices - 1) * 4.0f;     // µs
    float totalDelay   = propDelay + afeSettle ;
    uint8_t code       = uint8_t(totalDelay / 5.0f + 0.5f); // ปัดขึ้น
    return (code <= 40) ? code : 40;  // จำกัด 0–40
}
bool BQ79600::AutoAddressing(){
    bool ok;
    
    Serial.println("\n========================================");
    Serial.println("=== Starting Auto Addressing Process ===");
    Serial.println("========================================\n");
    
    // ----- Step 1: Write 0x00 to OTP_ECC_DATAIN registers (0x343-0x34A) -----
    Serial.println("--- Step 1: Clear OTP_ECC_DATAIN registers ---");
    for (int i = 0; i < 8; i++) {
        RegisterAddress reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::OTP_ECC_DATAIN) + i);
        data_arr_[0] = 0x00;
        sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, reg, data_arr_);
        Serial.printf("  ✓ Cleared register 0x%04X\n", (uint16_t)reg);
        delayMicroseconds(500);
    }

    // ----- Step 2: Broadcast write to enable auto-addressing mode (CONTROL1=0x01) -----
    Serial.println("--- Step 2: Enable Auto-Addressing Mode ---");
    data_arr_[0] = 0x01;
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::CONTROL1, data_arr_);
    Serial.println("----------Step 2: Completed------------------\n");
    delayMicroseconds(500);
    
    // ----- Step 3: Broadcast write consecutively to DIR0_ADDR = 0, 1, 2, 3... -----
    Serial.println("--- Step 3: Assign Device Addresses ---");
    for (int i = 0; i <= NumSegments; i++){
        data_arr_[0] = i;
        sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::DIR0_ADDR, data_arr_);
        Serial.printf("  Set DIR0_ADDR = %d\n", i);
        delayMicroseconds(500);
    }
    
    Serial.println("----------Step 3: Completed------------------\n");

    // ----- Step 4: Broadcast write to set all devices as stack device (COMM_CTRL=0x02) -----
    Serial.println("--- Step 4: Set All Devices as Stack ---");
    data_arr_[0] = 0x02;
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::COMM_CTRL, data_arr_);
    Serial.println("----------Step 4: Completed------------------\n");
    delayMicroseconds(500);

    // ----- Step 5: Single device write to highest device (COMM_CTRL=0x03) -----
    data_arr_[0] = 0x03;
    Serial.println("--- Step 5: Configure Top of Stack ---");
    sendCommandTo(RequestType::SingleWrite, 1, NumSegments, RegisterAddress::COMM_CTRL, data_arr_);
    Serial.printf("----------Step 5: Set device %d as Top of Stack------------------\n\n", NumSegments);
    delayMicroseconds(500);

    // ----- Step 6: Dummy stack read registers OTP_ECC_DATAOUT (0x343-0x34A) -----
    Serial.println("--- Step 6: Dummy Stack Read (Sync Internal DLL) ---");
    data_arr_[0] = 0x00;
    bool allSuccess = true;
    int failedCount = 0;
    uint16_t failedRegisters[8];  //  Static array (max 8 registers)
    
    for (int i = 0; i < 8; i++) {
        
        RegisterAddress reg = static_cast<RegisterAddress>(
            static_cast<uint16_t>(RegisterAddress::OTP_ECC_DATAIN) + i
        );
        
        Serial.printf("  Reading register 0x%04X (DATAIN%d)... ", (uint16_t)reg, i + 1);
        
        ok = this->sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, reg, data_arr_, response);
        
        if (ok) {
            Serial.println("✓ Success");
        } else {
            Serial.println("✗ FAILED");
            allSuccess = false;
            failedRegisters[failedCount++] = static_cast<uint16_t>(reg);  // ✅ เก็บใน array
        }
        delayMicroseconds(500);
    }

    // Display summary results
    Serial.println();
    if (allSuccess) {
        Serial.println("----------Step 6: Completed (All 8 dummy reads successful)------------------\n");
    } else {
        Serial.printf(" Step 6: %d/%d reads FAILED\n", failedCount, 8);
        Serial.println("Failed registers:");
        for (int i = 0; i < failedCount; i++) {  //  failedCount
            Serial.printf("  - 0x%04X\n", failedRegisters[i]);
        }
        Serial.println();
        Serial.println("========================================");
        Serial.println("=== Auto Addressing FAILED ===");
        Serial.println("========================================\n");
        return false;
    }

    // ----- Step 7: Stack read address 0x306 (Verify device addresses) -----
    Serial.println("--- Step 7: Verify Device Addresses ---");
    Serial.print("  Reading DIR0_ADDR (0x306)... ");
    
    ok = this->sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::DIR0_ADDR, data_arr_, response);
    delayMicroseconds(500);
    if (!ok) {
        Serial.println("✗ FAILED");
        Serial.println(" Step 7 Failed: Could not read device addresses from 0x306");
        Serial.println("========================================");
        Serial.println("=== Auto Addressing FAILED ===");
        Serial.println("========================================\n");
        return false;  
    }
    
    Serial.println("✓ Success");
    
    // Display the addresses read
    Serial.print("  Device addresses: ");
    for (uint8_t i = 0; i < NumSegments; ++i) {
        Serial.printf("0x%02X ", response[i]);
    }
    Serial.println();
    
    // Check if addresses are correct (should be 0, 1, 2, ...)
    bool addressValid = true;
    for (size_t i = 0; i < NumSegments; ++i) {
        if (response[i] != (NumSegments - i)) {
            Serial.printf("   Warning: Device %d has address 0x%02X (expected 0x%02X)\n", i, response[i], i);
            addressValid = false;
        }
    }
    
    if (!addressValid) {
        Serial.println(" Step 7 Failed: Device addresses incorrect!");
        Serial.println("========================================");
        Serial.println("=== Auto Addressing FAILED ===");
        Serial.println("========================================\n");
        return false;  
    }
    
    Serial.println("----------Step 7: Completed (All device addresses verified)------------------\n");

    // ----- Step 8: Single device read to BQ79600, verify 0x2001 = 0x14 -----
    Serial.println("--- Step 8: Verify BQ79600 Device Configuration ---");
    Serial.print("  Reading DEV_CONF1 (0x2001)... ");
    ok = this->sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR, RegisterAddress::DEV_CONF1, data_arr_, response);
    delayMicroseconds(500);
    
    if (!ok) {
        Serial.println("✗ FAILED");
        Serial.println(" Step 8 Failed: Could not read DEV_CONF1 from BQ79600");
        Serial.println("========================================");
        Serial.println("=== Auto Addressing FAILED ===");
        Serial.println("========================================\n");
        return false;  
    }
    
    Serial.println("✓ Success");
    
    uint8_t devConf = response[4];
    Serial.printf("  DEV_CONF1 = 0x%02X ", devConf);
    
    if (devConf == 0x14) {
        Serial.println("(Expected: 0x14) ✓");
        Serial.println("----------Step 8: Completed------------------\n");
        
        // success
        Serial.println("========================================");
        Serial.println("=== Auto Addressing SUCCESSFUL ===");
        Serial.println("========================================\n");
        return true;
        
    } else {
        Serial.printf("(Expected: 0x14) ✗\n");
        Serial.println(" Step 8 Failed: DEV_CONF1 value incorrect");
        Serial.println("========================================");
        Serial.println("=== Auto Addressing FAILED ===");
        Serial.println("========================================\n");
        return false;  
    }
}

bool BQ79600::receiveResponse(uint8_t response[], size_t expected_size, unsigned long timeout_ms) {
    
    if (!uart) {
        Serial.println("UART not initialized.");
        return false;
    }
    
    // Counter for number of bytes received
    size_t bytesReceived = 0;
    unsigned long start_time = millis();

    while ((millis() - start_time) < timeout_ms) {
        
        if (uart->available()) {
            byte incoming = uart->read();
            response[bytesReceived] = incoming;  // Store in  array
            bytesReceived++;
            
            // Check if complete
            if (bytesReceived == expected_size) {
                
                // Validate CRC 
                if (!validateCRC(response, expected_size)) {
                    Serial.println(" CRC check failed!");
                    
                    // show raw data
                    Serial.print("Raw response: ");
                    for (size_t i = 0; i < bytesReceived; ++i) {
                        Serial.printf("%02X ", response[i]);
                    }
                    Serial.println();
                    
                    return false;
                }

                // success
                Serial.println(" Received valid response:");
                for (size_t i = 0; i < bytesReceived; ++i) {
                    Serial.printf("%02X ", response[i]);
                }
                Serial.println();
                
                return true;
            }
        }
    }

    //  Timeout
    Serial.printf(" Timeout! Received %d/%d bytes\n", bytesReceived, expected_size);
    
    if (bytesReceived > 0) {
        Serial.print("Partial data: ");
        for (size_t i = 0; i < bytesReceived; ++i) {
            Serial.printf("%02X ", response[i]);
        }
        Serial.println();
    }
    if (bytesReceived < expected_size) {
        delay(200);
        Serial.printf(" Incomplete data! Got %d/%d bytes\n", bytesReceived, expected_size);
        return false;
    }
    
    return false;
}
bool BQ79600::receiveStackResponse(uint8_t response[], size_t expected_size,size_t data_per_device, unsigned long timeout_ms) {
    
    if (!uart) {
        Serial.println("UART not initialized.");
        return false;
    }
    
    // Static buffer สำหรับรับข้อมูล
    uint8_t rawBuffer[200];  
    size_t rawBufferSize = 0;
    const size_t FRAME_SIZE = 4 + data_per_device + 2;
    unsigned long start_time = millis();
    
    Serial.println(" Receiving data...");
    
    while ((millis() - start_time) < timeout_ms && rawBufferSize < expected_size) {
        if (uart->available()) {
            rawBuffer[rawBufferSize++] = uart->read();
        }
    }
    
    Serial.printf(" Received %d bytes (expected %d)\n\n", rawBufferSize, expected_size);
    
    // ตรวจสอบขนาดข้อมูล
    if (rawBufferSize < expected_size) {
        delay(200);
        Serial.printf(" Incomplete data! Got %d/%d bytes\n", rawBufferSize, expected_size);
        return false;
    }
    
    // แสดง Raw Buffer (optional - comment out ถ้าไม่ต้องการ)
    /* Serial.println("Raw Buffer (Hex):");
    for (size_t i = 0; i < rawBufferSize; ++i) {
        Serial.printf("%02X ", rawBuffer[i]);
        if ((i + 1) % 10 == 0) {
            Serial.println();
        }
    }
    Serial.println("\n"); */
    
    // Parse แต่ละ frame
    bool allValid = true;
    size_t responseIndex = 0;
    size_t device_count = NumSegments;
    
    for (size_t dev = 0; dev < device_count; ++dev) {
        size_t frameStart = dev * FRAME_SIZE;
        
        Serial.printf("--- Device %d (Frame at byte %d-%d) ---\n",dev, frameStart, frameStart + FRAME_SIZE - 1);
        
        //  Extract frame (ใช้ static array)
        uint8_t frame[64];
        for (size_t i = 0; i < FRAME_SIZE; ++i) {
            frame[i] = rawBuffer[frameStart + i];
        }
        
        // แสดง raw frame
        Serial.print("Raw Frame: ");
        for (size_t j = 0; j < FRAME_SIZE; ++j) {
            Serial.printf("%02X ", frame[j]);
        }
        Serial.println();
        
        // Parse header
        byte command = frame[0];
        byte devAddr = frame[1];
        uint16_t regAddr = (frame[2] << 8) | frame[3];
        
        Serial.printf("  Command: 0x%02X (Response: %s, Data Size: %d)\n",command, (command & 0x80) ? "No" : "Yes",(command & 0x07) + 1);
        Serial.printf("  Device Address: 0x%02X\n", devAddr);
        Serial.printf("  Register Address: 0x%04X\n", regAddr);
        
        //  Validate CRC 
        if (!validateCRC(frame, FRAME_SIZE)) {
            Serial.printf(" CRC check FAILED for Device %d (Address 0x%02X)\n\n", dev, devAddr);
            allValid = false;
            continue;
        }
        
        Serial.println(" CRC check PASSED");
        
        // Extract data (ข้าม header 4 bytes, ตัด CRC 2 bytes)
        Serial.print("  Extracted Data: ");
        for (size_t i = 4; i < FRAME_SIZE - 2; ++i) {
            response[responseIndex++] = frame[i];
            Serial.printf("%02X ", frame[i]);
        }
        Serial.println("\n");
    }
    
    if (!allValid) {
        Serial.println(" Some frames have CRC errors!");
        return false;
    }
    
    Serial.printf(" Successfully extracted %d bytes from %d devices\n\n",responseIndex, device_count);
    return true;
}

bool BQ79600::sendAndReceive(RequestType req_type, byte data_size, byte dev_addr,RegisterAddress reg_addr, uint8_t data[], uint8_t response[])
{
    bool isSingle = (req_type == RequestType::SingleRead);
    bool isStack  = (req_type == RequestType::StackRead);
    // Response: header 4 bytes + data + CRC 2 bytes
    const size_t HEADER_SIZE = 4;   // Command(1) + DevAddr(1) + RegAddr(2)
    const size_t CRC_SIZE = 2;
    size_t expected_response_len = HEADER_SIZE + data_size + CRC_SIZE;
    /* while (uart->available()) {  // เช็ค RX buffer
        uart->read();            // อ่านและทิ้ง 1 byte จาก RX
    } */
    data_arr_[0] = data_size - 1; // Dummy data for read
    sendCommandTo(req_type, 1 , dev_addr, reg_addr, data_arr_);
    delayMicroseconds(500); // 
        if (isSingle) {
            return receiveResponse(response, expected_response_len, 500);
        }
        if (isStack){
            size_t data_per_device = data_size;
            expected_response_len = expected_response_len * NumSegments;
            return receiveStackResponse(response, expected_response_len,data_per_device, 500);
            
        }
    
    return false;                    
}  

void BQ79600::shutdownPing()
{
    //uart->end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(12500);    // 12.5ms pulse
    digitalWrite(tx_pin_, HIGH);
    delayMicroseconds((5000 + 600)* NumSegments);  //(5ms active to shutdown transition + 600us propogation of wake) * number_of_devices

}

// ------------------------- Balance Check --------------------------------- // 

int BQ79600::checkBalance (uint8_t numstack){

    Serial.println(" data BAL_STAT"); 
    data_arr_[0] = 0x00; 
    bool ok = sendAndReceive(RequestType::SingleRead, 1, numstack, RegisterAddress::BAL_STAT, data_arr_, response);
    if(!ok){
        Serial.println(" MissData");    
    }
    return  response[4] ;
}

// ------------------------- Fault Bright --------------------------------- //
uint8_t BQ79600::FaultM_Summary(){

    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_SUMMARY, data_arr_, response);
    uint8_t response_fault;
    response_fault = response[4];

    return response_fault;
}
    /* if(response[4] == 0){
            Serial.println(" No Faults Detected");
            return false;
    }else
        if (response[4] & 0x08){
            Serial.println("FAULTM_COMM");
            Serial.println("=====================");
            FaultMComm1();
            FaultMComm2();
            Serial.println("=====================");
            return true;
        if (response[4] & 0x04){
            Serial.println("FAULT_REG");
            Serial.println("=====================");
            Fault_REG();
            Serial.println("=====================");
            return true;
        }else if (response[4] & 0x02){
            Serial.println("FAULTN_SYS");
            Serial.println("=====================");
            FaultM_SYS();
            Serial.println("=====================");
            return true;
        }else if(response[4] & 0x01) {
            Serial.println("FAULTM_PWR");
            Serial.println("=====================");
            FaultM_PWR();
            Serial.println("=====================");
            return true;
        }
        
        Serial.println("-----------------------------------");
        Serial.println("");
        return true ; */
    
uint8_t BQ79600::FaultM_Comm1() { 
    
    data_arr_[0] = 0x00;
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR, RegisterAddress::FAULTM_COMM1, data_arr_, response);
    uint8_t response_fault = response[4];

    return response_fault;
}

    /* if (fault == 0x00) {
        Serial.println("FAULT_COMM1: No Faults Detected");
        return 0x00;
    }

    if (fault & 0x80) {
        response_fault |= 0x80;
        Serial.println("FAULT_COMM1: RSVD");
    }
    if (fault & 0x40) {
        response_fault |= 0x40;
        Serial.println("FAULT_COMM1: FCOMM_DET");
    }
    if (fault & 0x20) {
        response_fault |= 0x20;
        Serial.println("FAULT_COMM1: FTONE_DET");
    }
    if (fault & 0x10) {
        response_fault |= 0x10;
        Serial.println("FAULT_COMM1: HB_FAIL");
    }
    if (fault & 0x08) {
        response_fault |= 0x08;
        Serial.println("FAULT_COMM1: HB_FAST"); // ตรวจใน datasheet อีกที
    }
    if (fault & 0x04) {
        response_fault |= 0x04;
        Serial.println("FAULT_COMM1: UART_FRAME");
    }
    if (fault & 0x02) {
        response_fault |= 0x02;
        Serial.println("FAULT_COMM1: COMMCLR_DET");
    }
    if (fault & 0x01) {
        response_fault |= 0x01;
        Serial.println("FAULT_COMM1: STOP_DET");
    }
 */


uint8_t BQ79600::FaultM_Comm2(){
    
    data_arr_[0] = 0x00;
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_COMM2, data_arr_, response);
    uint8_t response_fault = response[4];

    return response_fault;
}
    /* if(fault == 0){
            Serial.println("FaultComm2: No Faults Detected");
            return ;
    }
    
    if (fault & 0x80){
        Serial.println("FAULT_COMM2: RSVD");
    }if (fault & 0x40){
        Serial.println("FAULT_COMM2: RSVD");
    }if (fault & 0x20){
        Serial.println("FAULT_COMM2: SPI_FRAME");
    }if (fault & 0x10){
        Serial.println("FAULT_COMM2: SPI_PHY");
    }if (fault & 0x08){
        Serial.println("FAULT_COMM2: COML_FRAME");
    }if (fault & 0x04){
        Serial.println("FAULT_COMM2: COML_PHY");
    }if (fault & 0x02){
        Serial.println("FAULT_COMM2: COMH_FRAME");
    }if (fault & 0x01) {
        Serial.println("FAULT_COMM2: COMH_PHY");
    } */

uint8_t BQ79600::FaultM_REG(){

    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_REG, data_arr_, response);
    uint8_t response_fault = response[4];

    return response_fault;
}
    /* if(response[4] == 0){
            Serial.println("FAULT_REG No Faults Detected");
            
    }else if (response[4] & 0x04){
            Serial.println("FAULT_CONF_MON_ERR");
    }else if (response[4] & 0x02){
            Serial.println("FAULT_FACTLDERR");
    }else if(response[4] & 0x01) {
            Serial.println("FAULT_FACT_CRC");
    } */


uint8_t BQ79600::FaultM_SYS(){

    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_SYS, data_arr_, response);
    uint8_t response_fault = response[4];

    return response_fault;
}
    /* if(response[4] == 0){
            Serial.println("FAULTM_SYS No Faults Detected");

        }else if (response[4] & 0x80){
            Serial.println("VALIDATE_DET");
        }else if (response[4] & 0x40){
            Serial.println("LFO");
        }else if (response[4] & 0x20){
            Serial.println("SHUTDOWN_REC");
        }else if (response[4] & 0x10){
            Serial.println("DRST");
        }else if (response[4] & 0x08){
            Serial.println("CTL");
        }else if (response[4] & 0x04){
            Serial.println("CTS");
        }else if (response[4] & 0x02){
            Serial.println("TSHUT");
        }else if(response[4] & 0x01) {
            Serial.println("INH");
        } */

uint8_t BQ79600::FaultM_PWR(){

    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_PWR, data_arr_, response);
    uint8_t response_fault = response[4];

    return response_fault;
}
    /* if(response[4] == 0){
            Serial.println("FAULTM_PWR No Faults Detected");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_CVDD_UV_DRST");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_CVDD_OV");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_DVDD_OV");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_AVDDREF_OV");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_AVAO_SW_FAIL");
        } */


// ------------------------- Fault Stack --------------------------------- //
// Fault_Summary
void BQ79600::Fault_Summary(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR,RegisterAddress::FAULT_SUMMARY, data_arr_, response);
    
    if (!ok) {
        Serial.println("Failed to read FAULT_SUMMARY (Stack Read)");
        memset(faults, 0xFF, NumSegments);  // Fill with error indicator
        return;
    }
    
    //  Copy data from response to faults array
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    
    /* if(response[4] == 0){
            Serial.println(" No Faults Detected");
            return false;
    }else
        if(response[4] & 0x80){
            Serial.println(" FAULT_PROT");
            FAULT_PROT1();
            FAULT_PROT2();
            return true;
        }else if (response[4] & 0x40){
            Serial.println("FAULT_COMP_ADC");
            return true;
        }else if (response[4] & 0x20){
            Serial.println("FAULT_OTP");
            return true;
        }else if (response[4] & 0x10){
            Serial.println("FAULT_COMM");
            return true;
        }else if (response[4] & 0x08){
            Serial.println("FAULT_OTUT");
            return true;
        }else if (response[4] & 0x04){
            Serial.println("FAULT_OVUV");
            FAULT_OV1();
            FAULT_OV2();
            FAULT_UV1();
            FAULT_UV2();
            return true;
        }else if (response[4] & 0x02){
            Serial.println("FAULT_SYS");
            return true;
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_PWR");
            return true;
        }
        Serial.println("-----------------------------------");
        Serial.println(""); */
    
// FAULT_PROT1
void BQ79600::FAULT_PROT1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR,RegisterAddress::FAULT_PROT1, data_arr_, response);
    
    if (!ok) {
        Serial.println("Failed to read FAULT_PROT1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_PROT1 No Faults Detected");
            return ;
    }else if (response[4] & 0x02){
            Serial.println("FAULT_PROT1: TPARITY_FAIL");
    }else if(response[4] & 0x01) {
            Serial.println("FAULT_PROT1: VPARITY_FAIL");
        } */
    
// FAULT_PROT2
void BQ79600::FAULT_PROT2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_PROT2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_PROT2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_PROT2 No Faults Detected");
            
    }else if (response[4] & 0x40){
            Serial.println("FAULT_PROT2: BIST_ABORT");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_PROT2: TPATH_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_PROT2: VPATH_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_PROT2: UTCOMP_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_PROT2: OTCOMP_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_PROT2: OVCOMP_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_PROT2: UVCOMP_FAIL");
        } */

//  FAULT_OV1
void BQ79600::FAULT_OV1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_OV1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_OV1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_OV1 No Faults Detected");
            return ;
    }else if (response[4] & 0x80){
            Serial.println("FAULT_OV cell 16");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_OV cell 15");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_OV cell 14");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_OV cell 13");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_OV cell 12");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_OV cell 11");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_OV cell 10");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_OV cell 9");
        } */

// FAULT_OV2
void BQ79600::FAULT_OV2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_OV2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_OV2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_OV2 No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("FAULT_OV cell 8");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_OV cell 7");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_OV cell 6");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_OV cell 5");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_OV cell 4");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_OV cell 3");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_OV cell 2");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_OV cell 1");
        } */

//  FAULT_UV1
void BQ79600::FAULT_UV1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_UV1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_UV1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_UV1 No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("FAULT_UV cell 16");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_UV cell 15");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_UV cell 14");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_UV cell 13");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_UV cell 12");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_UV cell 11");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_UV cell 10");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_UV cell 9");
        } */

// FAULT_UV2
void BQ79600::FAULT_UV2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_UV2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_UV2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_UV2 No Faults Detected");
            
        }else if (response[4] & 0x80){
            Serial.println("FAULT_UV cell 8");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_UV cell 7");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_UV cell 6");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_UV cell 5");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_UV cell 4");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_UV cell 3");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_UV cell 2");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_UV cell 1");
        } */

// FAULT_COMP_VCCB1
void BQ79600::FAULT_COMP_VCCB1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_VCCB1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_VCCB1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_VCCB1 No Faults Detected");

        }else if (response[4] & 0x80){
            Serial.println("CELL16_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CELL15_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CELL14_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CELL13_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CELL12_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CELL11_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CELL10_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CELL9_FAIL");
        }
 */
//  FAULT_COMP_VCCB2
void BQ79600::FAULT_COMP_VCCB2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_VCCB2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_VCCB2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_VCCB2 No Faults Detected");

        }else if (response[4] & 0x80){
            Serial.println("CELL8_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CELL7_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CELL6_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CELL5_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CELL4_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CELL3_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CELL2_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CELL1_FAIL");
        } */

//  FAULT_COMP_CBOW1
void BQ79600::FAULT_COMP_CBOW1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_CBOW1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_CBOW1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_CBOW1 No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("CBOW16_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CBOW15_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CBOW14_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CBOW13_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CBOW12_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CBOW11_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CBOW10_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CBOW9_FAIL");
        } */

//  FAULT_COMP_CBOW2
void BQ79600::FAULT_COMP_CBOW2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_CBOW2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_CBOW2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_CBOW2 No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("CBOW8_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CBOW7_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CBOW6_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CBOW5_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CBOW4_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CBOW3_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CBOW2_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CBOW1_FAIL");
        } */

// FAULT_COMP_CBFET1
void BQ79600::FAULT_COMP_CBFET1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_CBFET1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_CBFET1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_CBFET1 No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("CBFET16_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CBFET15_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CBFET14_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CBFET13_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CBFET12_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CBFET11_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CBFET10_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CBFET9_FAIL");
        } */

void BQ79600::FAULT_COMP_CBFET2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_CBFET2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_CBFET2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_CBFET2 No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("CBFET8_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("CBFET7_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("CBFET6_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("CBFET5_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("CBFET4_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("CBFET3_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("CBFET2_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("CBFET1_FAIL");
        } */

//  FAULT_COMP_GPIO
void BQ79600::FAULT_COMP_GPIO(uint8_t faults[]) {
    data_arr_[0] = 0x00;
    
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_GPIO, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_GPIO (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_COMP_GPIO No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("GPIO8_FAIL");
        }else if (response[4] & 0x40){
            Serial.println("GPIO7_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("GPIO6_FAIL");
        }else if (response[4] & 0x10){
            Serial.println("GPIO5_FAIL");
        }else if (response[4] & 0x08){
            Serial.println("GPIO4_FAIL");
        }else if (response[4] & 0x04){
            Serial.println("GPIO3_FAIL");
        }else if (response[4] & 0x02){
            Serial.println("GPIO2_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("GPIO1_FAIL");
        } */

void BQ79600::FAULT_COMP_MISC(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMP_MISC, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMP_MISC (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}

    /* if(response[4] == 0){
            Serial.println("RSVD");
        }else if (response[4] & 0x40){
            Serial.println("RSVD");
        }else if (response[4] & 0x20){
            Serial.println("RSVD");
        }else if (response[4] & 0x10){
            Serial.println("RSVD");
        }else if (response[4] & 0x08){
            Serial.println("RSVD");
        }else if (response[4] & 0x04){
            Serial.println("RSVD");
        }else if (response[4] & 0x02){
            Serial.println("COMP_ADC_ABORT");
        }else if(response[4] & 0x01) {
            Serial.println("LPF_FAIL");
        } */
//  FAULT_OTP
void BQ79600::FAULT_OTP(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_OTP, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_OTP (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}

    /* if(response[4] == 0){
            Serial.println("FAULT_OTP No Faults Detected");
        }else if (response[4] & 0x80){
            Serial.println("RSVD ");
        }else if (response[4] & 0x40){
            Serial.println("DED_DET");
        }else if (response[4] & 0x20){
            Serial.println("SEC_DET");
        }else if (response[4] & 0x10){
            Serial.println("CUST_CRC");
        }else if (response[4] & 0x08){
            Serial.println("FACT_CRC");
        }else if (response[4] & 0x04){
            Serial.println("CUSTLDERR");
        }else if (response[4] & 0x02){
            Serial.println("FACTLDERR");
        }else if(response[4] & 0x01) {
            Serial.println("GBLOVERR");
        } */

//  Fault_Comm1
void BQ79600::Fault_Comm1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMM1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMM1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FaultComm1: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("FAULT_COMM1: RSVD");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_COMM1: RSVD");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_COMM1: RSVD");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_COMM1: UART_TR");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_COMM1: UART_RR");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_COMM1: UART_RC");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_COMM1: COMMCLR_DET");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_COMM1: STOP_DET");
        } */

//  Fault_Comm2
void BQ79600::Fault_Comm2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMM2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMM2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FaultComm2: No Faults Detected");
            return ;
        }else if (response[4] & 0x20){
            Serial.println("FAULT_COMM2: COML_RC");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_COMM2: COML_BIT");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_COMM2: COMH_TR");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_COMM2: COMH_RR");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_COMM2: COMH_RC");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_COMM2: COMH_BIT");
        } */

//  Fault_Comm3
void BQ79600::Fault_Comm3(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_COMM3, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_COMM3 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FaultComm3: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("FAULT_COMM3: RSVD");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_COMM3: RSVD");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_COMM3: RSVD");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_COMM3: RSVD");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_COMM3: FCOMM_DET");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_COMM3: FTONE_DET");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_COMM3: HB_FAIL");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_COMM3: HB_FAST");
        } */

//  FAULT_OT
void BQ79600::FAULT_OT(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_OT, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_OT (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_OT: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("FAULT_OT: OT8_DET");
        }else if (response[4] & 0x40){
            Serial.println("FAULT_OT: OT7_DET");
        }else if (response[4] & 0x20){
            Serial.println("FAULT_OT: OT6_DET");
        }else if (response[4] & 0x10){
            Serial.println("FAULT_OT: OT5_DET");
        }else if (response[4] & 0x08){
            Serial.println("FAULT_OT: OT4_DET");
        }else if (response[4] & 0x04){
            Serial.println("FAULT_OT: OT3_DET");
        }else if (response[4] & 0x02){
            Serial.println("FAULT_OT: OT2_DET");
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_OT: OT1_DET");
        } */


//  FAULT_UT
void BQ79600::FAULT_UT(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_UT, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_UT (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_UT: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("UT8_DET");
        }else if (response[4] & 0x40){
            Serial.println("UT7_DET");
        }else if (response[4] & 0x20){
            Serial.println("UT6_DET");
        }else if (response[4] & 0x10){
            Serial.println("UT5_DET");
        }else if (response[4] & 0x08){
            Serial.println("UT4_DET");
        }else if (response[4] & 0x04){
            Serial.println("UT3_DET");
        }else if (response[4] & 0x02){
            Serial.println("UT2_DET");
        }else if(response[4] & 0x01) {
            Serial.println("UT1_DET");
        } */

//  FAULT_SYS
void BQ79600::FAULT_SYS(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_SYS, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_SYS (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_SYS: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("LFO");
        }else if (response[4] & 0x40){
            Serial.println("RSVD");
        }else if (response[4] & 0x20){
            Serial.println("GPIO");
        }else if (response[4] & 0x10){
            Serial.println("DRST");
        }else if (response[4] & 0x08){
            Serial.println("CTL");
        }else if (response[4] & 0x04){
            Serial.println("CTS");
        }else if (response[4] & 0x02){
            Serial.println("TSHUT");
        }else if(response[4] & 0x01) {
            Serial.println("TWARN");
        } */

//  FAULT_PWR1
void BQ79600::FAULT_PWR1(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_PWR1, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_PWR1 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_PWR1: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("CVSS_OPEN");
        }else if (response[4] & 0x40){
            Serial.println("DVSS_OPEN");
        }else if (response[4] & 0x20){
            Serial.println("REFHM_OPEN");
        }else if (response[4] & 0x10){
            Serial.println("CVDD_UV");
        }else if (response[4] & 0x08){
            Serial.println("CVDD_OV");
        }else if (response[4] & 0x04){
            Serial.println("DVDD_OV");
        }else if (response[4] & 0x02){
            Serial.println("AVDD_OSC");
        }else if(response[4] & 0x01) {
            Serial.println("AVDD_OV");
        } */

// ✅ 24. FAULT_PWR2
void BQ79600::FAULT_PWR2(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_PWR2, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_PWR2 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_PWR2: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("RSVD");
        }else if (response[4] & 0x40){
            Serial.println("PWRBIST_FAIL");
        }else if (response[4] & 0x20){
            Serial.println("RSVD");
        }else if (response[4] & 0x10){
            Serial.println("REFH_OSC");
        }else if (response[4] & 0x08){
            Serial.println("NEG5V_UV");
        }else if (response[4] & 0x04){
            Serial.println("TSREF_OSC");
        }else if (response[4] & 0x02){
            Serial.println("TSREF_UV");
        }else if(response[4] & 0x01) {
            Serial.println("TSREF_OV");
        } */

//  FAULT_PWR3
void BQ79600::FAULT_PWR3(uint8_t faults[]) {

    data_arr_[0] = 0x00;
    bool ok = sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::FAULT_PWR3, data_arr_, response);
    
    if (!ok) {
        Serial.println("⚠️ Failed to read FAULT_PWR3 (Stack Read)");
        memset(faults, 0xFF, NumSegments);
        return;
    }
    
    for (uint8_t stack = 0; stack < NumSegments; stack++) {
        faults[stack] = response[stack];
    }
}
    /* if(response[4] == 0){
            Serial.println("FAULT_PWR3: No Faults Detected");
            return ;
    }else
        if (response[4] & 0x80){
            Serial.println("RSVD");
        }else if (response[4] & 0x40){
            Serial.println("RSVD");
        }else if (response[4] & 0x20){
            Serial.println("RSVD");
        }else if (response[4] & 0x10){
            Serial.println("RSVD");
        }else if (response[4] & 0x08){
            Serial.println("RSVD");
        }else if (response[4] & 0x04){
            Serial.println("RSVD");
        }else if (response[4] & 0x02){
            Serial.println("RSVD");
        }else if(response[4] & 0x01) {
            Serial.println("AVDDUV_DRST");
        } */

void BQ79600::clearFault() {

    // -------------- Clear all faults of Bq79612 -------------------- //
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::FAULT_RST2 , data_arr_); 
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::FAULT_RST1 , data_arr_);

    // -------------- Clear all faults of Bq79600 -------------------- //
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR , RegisterAddress::FAULT_RST , data_arr_); 

    Serial.println("All Faults Cleared!");
    Serial.println("");
}

bool BQ79600::validateCRC(uint8_t frame[], size_t frame_size) {
    if (frame_size < 3) {
        Serial.println("⚠️ Frame too short for CRC check");
        return false;
    }

    // คำนวณ CRC จาก entire frame
    // ถ้า CRC ถูกต้อง → ผลลัพธ์ต้องเป็น 0x0000
    uint16_t calculatedCRC = crc.Modbus(frame, 0, frame_size);
    
    //  Debug output
    if (calculatedCRC != 0x0000) {
        // show CRC  (2 bytes )
        uint16_t receivedCRC = (frame[frame_size - 1] << 8) | frame[frame_size - 2];
        
        Serial.printf("   CRC mismatch!\n");
        Serial.printf("   Received CRC: 0x%04X\n", receivedCRC);
        Serial.printf("   CRC check result: 0x%04X (expected 0x0000)\n", calculatedCRC);
        
        // แสดง frame ที่ error
        Serial.print("   Frame: ");
        for (size_t i = 0; i < frame_size; ++i) {
            Serial.printf("%02X ", frame[i]);
        }
        Serial.println();
        
        return false;
    }
    
    return true;
}


void BQ79600::cheak() {
    data_arr_[0] = 0x00;
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR , RegisterAddress::DEV_CONF1 , data_arr_,response); 
    Serial.print("DEV_CONF1: ");
    Serial.println(response[4], HEX);

    data_arr_[0] = 0x00;
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR , RegisterAddress::FAULTM_MSK , data_arr_,response); 
    Serial.print("FAULTM_MSK: ");
    Serial.println(response[4], HEX);
    
}