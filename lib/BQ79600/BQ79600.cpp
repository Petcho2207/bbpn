#include "BQ79600.h"

Crc16 crc;
BQ79600::BQ79600(HardwareSerial &serial, uint32_t baud, int tx_pin, BQ79600config config)
    : uart(&serial), baudRate(baud), tx_pin_(tx_pin),
    NumSegments(config.num_segments),
    NumCellsSeries(config.num_cells_series),
    NumThermistors(config.num_thermistors),
    ShuntResistance(config.shunt_resistance)
{
    data_arr_.resize(20);
}

void BQ79600::initialize() {
    wakeUp();                                           // Wake up the BQ79600  and BQ79612
    AutoAddressing();                                   // Auto address the BQ79600 devices
    
    config_MainADC(NumCellsSeries,NumSegments);         // Configure the main ADC 
    //config_AuxADC() ;                                    // Configure the aux ADC
    config_Fault();                                     // Configure the fault settings
    config_OT_UT();                                     // Configure the Over Temperature and Under Temperature settings
    config_OV_UV();                                     // Configure the Over Voltage and Under Voltage settings
    

}

void BQ79600::beginUart()
{
    uart->begin(baudRate, SERIAL_8N1, 16, tx_pin_); // SERIAL_8N1_HALF_DUPLEX);  // BQ79656 uart interface is half duplex

} 

void BQ79600::wakeUp() {
    wakePing();
    wakePing();
    delay(5);
    beginUart();
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR , RegisterAddress::CONTROL1,{0x20});   // Send WAKE tone up the stack bq79612
    delay(5);
}

void BQ79600::wakePing() {
    uart->end();
    pinMode(tx_pin_, OUTPUT);
    digitalWrite(tx_pin_, LOW);
    delayMicroseconds(2700);        // low 2.75ms 
    digitalWrite(tx_pin_, HIGH);
    delayMicroseconds(3500);        //  delay 3.5ms  
    
}



void BQ79600::sendCommandTo(RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, std::vector<byte> data){
    data_size -= 1;  // 0 means 1 byte
    bool StackOrBroad   =   (req_type == RequestType::StackRead)      || 
                            (req_type == RequestType::StackWrite)     || 
                            (req_type == RequestType::BroadcastRead)  || 
                            (req_type == RequestType::BroadcastWrite) || 
                            (req_type == RequestType::BroadcastWrite_REV) ;
    size_t frame_len = 6 + data_size + (StackOrBroad ? 0 : 1);
    bq_frame_.resize(frame_len); // หรือขนาดที่ต้องใช้จริง
    bq_frame_[0] = static_cast<byte>(req_type) |(data_size & 0b00000111);   // command type | data_size
    if (!StackOrBroad)
        {
            bq_frame_[1] = dev_addr ;  // device address
        }
    bq_frame_[1 + (!StackOrBroad)] = static_cast<uint16_t>(reg_addr) >> 8;       // register address high byte
    bq_frame_[2 + (!StackOrBroad)] = static_cast<uint16_t>(reg_addr) & 0xFF;     // register address low byte
    for (int i = 0; i <= data_size; i++)
    {
        bq_frame_[3 + i + (!StackOrBroad)] = data[i];
    }
    uint16_t command_crc = crc.Modbus(bq_frame_.data(), 0, 4 + data_size + (!StackOrBroad));  // calculates the CRC, but the bytes are backwards
    bq_frame_[4 + data_size + (!StackOrBroad)] = command_crc & 0xFF;
    bq_frame_[5 + data_size + (!StackOrBroad)] = command_crc >> 8;
    
    /*  Serial.println("Command: ");
        for (int i = 0; i <= 5 + data_size + (!StackOrBroad); i++)
        {
            Serial.print(bq_frame_[i], HEX);
            Serial.print(" "); 
        }
        Serial.println(); */
    
    uart->write(bq_frame_.data(), 6 + data_size + (!StackOrBroad));
    delay(5);
}

void BQ79600::config_MainADC(uint8_t numcell,uint8_t numStack) {
    if (numcell < 6 || numcell > 16) {
        Serial.println("Invalid numcell: must be between 6 and 16");
        return;
    }
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
    data_arr_[0] = 0x01 ; //  Configures GPIO1 ,GPIO2  
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

    //----------------------------- reset MAIN ADC --------------------------------------------------///
    Serial.println("");
    Serial.println(" Reset LPF_BB_EN, LPF_VCELL,MAIN_MODE command  ");
    data_arr_[0] = 0x00 ; //  reset LPF_BB_EN, LPF_VCELL ,MAIN_MODE
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL1 ,data_arr_);   // START ADC
    delayMicroseconds(3500);    //  wait for ADC to start 

    //----------------------------- start MAIN ADC --------------------------------------------------///
    Serial.println("");
    Serial.println(" Set LPF_BB_EN, LPF_VCELL,MAIN_MODE command  ");
    Serial.println("Start ReadMainADC ");
    data_arr_[0] = 0x05 ; //  LPF_BB_EN, LPF_VCELL ,MAIN_MODE
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL1 ,data_arr_);   // START ADC
    //delayMicroseconds(3500);    //  wait for ADC to start 
    delayMicroseconds(192*8);   //  wait for ADC finished 
}  
void BQ79600::config_AuxADC() {
    
    // ---------- configure Selects which AUXCELL ---------- //
    data_arr_[0] = 0x00; // Run all active cell channels set by ACTIVE_CELL_CONF register
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL2 ,data_arr_);

    // ---------- configure Selects GPIO AUX  ---------- //
    data_arr_[0] = 0x06; // Run all active GPIO channels set by GPIO_CONF register
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL3 ,data_arr_);
    
    //delayMicroseconds(3500);    //  wait for ADC to start 
    delayMicroseconds(192*8);   //  wait for ADC finished 
}
void BQ79600::BalanceCells(uint8_t mode, size_t stack, const std::vector<size_t>& targetCells, size_t preservedCell, uint8_t hexThreshold){
    bool ok;

     // optional: stop cell voltage Threshold for balancing
    data_arr_[0] = hexThreshold; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::VCB_DONE_THRESH , data_arr_);
    data_arr_[0] = 0x02; // Run the OV and UV round robin and Go
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OVUV_CTRL , data_arr_);

    // determine the balaneing channels
    data_arr_[0] = 0x02; //  seting time of balancing  10 s

    for (int i = 0; i < NumCellsSeries; i++) {
    RegisterAddress reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::CB_CELL1_CTRL) - i );

        if (std::find(targetCells.begin(), targetCells.end(), i) != targetCells.end() && i != preservedCell) {
            data_arr_[0] = 0x04;  // 
            //Serial.printf(" → Balancing ON  | Cell Index %2d (Reg 0x%04X)\n", i, reg);
        } else {
            data_arr_[0] = 0x00;  // cell นี้ไม่ต้องทำ balancing
            //Serial.printf(" → Balancing OFF | Cell Index %2d (Reg 0x%04X)\n", i, reg);
        }

        sendCommandTo(RequestType::StackWrite, 1, 0x01, reg, data_arr_);
    }

    // setting duty cycle if auto balancing is used
    data_arr_[0] = 0x01;  // seting time of balancing  10 s
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::BAL_CTRL1 , data_arr_);

    /* // setting for on one or two CBFED  
    data_arr_[0] = 0x14; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::DEV_CONF , data_arr_); */
    // configures mode of OV/UV detection  
    
    data_arr_[0] = 0x05; // Run the OV and UV round robin and Go
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OVUV_CTRL , data_arr_);
    
    // determine the method of balancing
    data_arr_[0] = 0x22; //  Configures the balancing method to be used , FLTSTOP_EN , BAL_GO , OTCB_EN
    
    if(mode == 1) {
        data_arr_[0] = data_arr_[0] |(1<<0); //  Configures the balancing Auto method to be used
        Serial.println("Balancing mode: Auto"); 
    }else{
        Serial.println("Balancing mode: manaul"); 
    }
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::BAL_CTRL2 , data_arr_); 
}


void BQ79600::config_OV_UV(){
    //  Configures the overvoltage and undervoltage thresholds 
    data_arr_[0] = 0x23; //  Set the overvoltage threshold
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OV_THRESH , data_arr_);
    
    data_arr_[0] = 0x00; //  Set the undervoltage threshold 
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
    
}
void BQ79600::config_OT_UT(){
    // Configures the overtemperature and undertemperature thresholds
    data_arr_[0] = 0x00;
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_THRESH , data_arr_); 

    /* // Configures the overtemperature and undertemperature thresholds for run again
    data_arr_[0] = 0x00; 
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_CTRL , data_arr_); */

    // Configures mode of OT/UT detection
    data_arr_[0] = 0x00; // Run the OT and UT round robin and Go
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::OTUT_CTRL , data_arr_);

}

void BQ79600::config_Fault(){
    data_arr_[0] = 0x40; 
    sendCommandTo(RequestType::SingleWrite, 1, 0x01, RegisterAddress::FAULT_MSK2 , data_arr_);  
    // Configures BQ79600 and BQ79612 enable NFAULT , FCOMM_EN , HB_TX_EN ,FTO_EN
    data_arr_[0] = 0x57; //  Enable NFAULT , FCOMM_EN , HB_TX_EN , FTO_EN 
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::DEV_CONF , data_arr_);

    
}


    
void BQ79600::ReadVoltCellandTemp() {
    std::vector<std::vector<double>> die_temp;
    std::vector<std::vector<double>> temp;
    std::vector<std::vector<double>> voltages;
    std::vector<double> TSREF;
    RegisterAddress reg;
    bool ok;
    bool cheak = false; 
    //------------------------- go Main ADC ----------------------------- //
    data_arr_[0] = 0x05 ; //  LPF_BB_EN, LPF_VCELL ,MAIN_MODE
    sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, RegisterAddress::ADC_CTRL1 ,data_arr_);   // START ADC
    delay(200);
    /* while(!cheak) {
        delay(5);
        cheak = ReadMainADC(); //  read Main ADC
        if (cheak) {
            Serial.println("Read Main ADC done");
            cheak = true; // exit the loop
        } else {
            Serial.println("wait for Read Main ADC done 8 round robin");
        }
    } */
    // --------------------------- Read Vcell --------------------------- //
    /* Serial.println("");
    Serial.println("READ CELL VOLTAGE from register "); */
    data_arr_[0] = (NumCellsSeries * 2) - 1;
    reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::VCELL16_HI) + (16 - NumCellsSeries) * 2);
    ok = this->sendAndReceive(RequestType::StackRead, (NumCellsSeries * 2), DEV_ADDR, reg, data_arr_, response); 
    voltages = parseCellVoltages(response, NumSegments, NumCellsSeries);

    // --------------------------- Read TSREF --------------------------- //
    /* Serial.println("");
    Serial.println("READ TSREF VOLTAGE from register "); */
    data_arr_[0] = 0x01; // 2 byte for TSREF
    ok = this->sendAndReceive(RequestType::StackRead, 2, DEV_ADDR,RegisterAddress:: TSREF_HI, data_arr_, response); 
    TSREF = parseTSREF(response, NumSegments);
    // --------------------------- Read GPIO Temp --------------------------- //
    /* Serial.println("");
    Serial.println(" READ Temp from register "); */
    data_arr_[0] = (NumThermistors * 2) - 1;
    reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::GPIO1_HI));
    ok = this->sendAndReceive(RequestType::StackRead, (NumThermistors * 2), DEV_ADDR, reg, data_arr_, response);
    temp = parseTemp(response,TSREF,NumSegments, NumThermistors);

    // --------------------------- Read Die Temp --------------------------- //
    /* Serial.println("");
    Serial.println(" READ die Temp from register "); */
    data_arr_[0] = 0x01;
    reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::DIETEMP1_HI));
    ok = this->sendAndReceive(RequestType::StackRead, 2, DEV_ADDR, reg, data_arr_, response);
    die_temp = parsedie_Temp(response, NumSegments); 

    // --------------------------- Map to struct --------------------------- //
    if (ok) {
        batteryData_pack.clear();  // clear previous data
        batteryData_pack.resize(NumSegments);


        for (uint8_t stack = 0; stack < NumSegments; ++stack) {
            StackData& currentStack = batteryData_pack[stack];
            currentStack.cells.resize(NumCellsSeries);
            // --- set voltage per cell --- //
            
            for (uint8_t cell = 0; cell < NumCellsSeries; ++cell) {
                
                currentStack.cells[cell].voltage = voltages[stack][NumCellsSeries - 1 - cell];
            }

             // --- set GPIO temps (stack level) --- //
            currentStack.gpioTemps.resize(NumThermistors);
            for (uint8_t num = 0; num < NumThermistors; ++num) {
                currentStack.gpioTemps[num] = temp[stack][num];
                
            }

            // --- set die temp --- //
            if (stack < die_temp.size() && die_temp[stack].size() > 0) {
                currentStack.dieTemp = die_temp[stack][0];
            } else {
                Serial.printf("Warning: die_temp[%d] is empty!\n", stack);
                currentStack.dieTemp = -999; // หรือค่า default/error อื่น
            } 
        }

    } else {
        Serial.println("Failed to read volt cell!");
    }  
}
bool BQ79600::ReadMainADC(){
    data_arr_[0] = 0x00; //  
    sendCommandTo(RequestType::SingleRead, 1, 0x01, RegisterAddress::ADC_STAT1 ,data_arr_);   // START ADC
    if(response[4] & 0x01){
        Serial.println("ADC_STAT1: ADC is done!");
        return true; // ADC is still busy
    }else{
        Serial.println("ADC_STAT1: wait ADC complete....");
        return false;
    }
    
}
std::vector<std::vector<double>> BQ79600::parsedie_Temp( const std::vector<byte>& response, uint8_t numStacks){

    std::vector<std::vector<double>> die_temp(numStacks);
    const size_t frameHeaderSize = 4;
    const size_t crcSize = 2;
    const size_t dataSizePerStack = 1 ;
    const size_t frameSize = frameHeaderSize + dataSizePerStack + crcSize;
    if (response.size() < frameSize * numStacks) {
        Serial.println("Response size too small!");
        return die_temp; // early return with empty or partial vector
    }
    for (uint8_t stack = 0; stack < numStacks; ++stack) {
        size_t index = frameHeaderSize+(stack * frameSize) ;
            if (index + 1 >= response.size()) {
                Serial.println("Index out of bounds in parsedie_Temp!");
                continue;
            }

            uint16_t raw = (response[index] << 8) | response[index + 1];
            double Temp = convertTo_dietemp(raw);
            //Serial.println(Temp);

            die_temp[stack].push_back(Temp);
        
    }

    return die_temp;
}

std::vector<std::vector<double>> BQ79600::parseTemp( const std::vector<byte>& response,const std::vector<double> TSREF, uint8_t numStacks, uint8_t numNTC){

    std::vector<std::vector<double>> Temp(numStacks, std::vector<double>(numNTC));
    const size_t frameHeaderSize = 4;
    const size_t crcSize = 2;
    const size_t dataSizePerStack = NumThermistors * 2;
    const size_t frameSize = frameHeaderSize + dataSizePerStack + crcSize;
    int R1 =  10000; // 10k ohm
    float B = 3977;
    float T0 = 25.0 + 273.15;
    

    if (response.size() < frameSize * numStacks) {
        Serial.println("Response size too small!");
        return Temp; // early return with empty or partial vector
    }
    for (uint8_t stack = 0; stack < numStacks; ++stack) {
        size_t base = stack * frameSize;

        for (uint8_t num = 0; num < numNTC; ++num) {
            size_t index = base + frameHeaderSize + num * 2;

            if (index + 1 >= response.size()) {
                Serial.println("Index out of bounds in parseTemp!");
                continue;
            }

            uint16_t raw = (response[index] << 8) | response[index + 1];
            double GPIO_Volt = convertTo_VoltGPIO(raw); 
            double Rntc =  (GPIO_Volt*R1 / (5 - GPIO_Volt));
            double lnR = log(Rntc / R1);
            double Temp_K = 1.0 / ((lnR / B) + (1.0 / T0));
            double GPIO_Temp = Temp_K - 273.15;

            Temp[stack][num] = GPIO_Temp;
            Serial.println(GPIO_Temp);
            Serial.println(TSREF[stack]);
            Serial.println(GPIO_Volt); 
        }
    }

    return Temp;
}
std::vector<double> BQ79600::parseTSREF( const std::vector<byte>& response, uint8_t numStacks){

    std::vector<double> TSREF(numStacks);
    const size_t frameHeaderSize = 4;
    const size_t crcSize = 2;
    const size_t dataSizePerStack = 1 ;
    const size_t frameSize = frameHeaderSize + dataSizePerStack + crcSize;
    if (response.size() < frameSize * numStacks) {
        Serial.println("Response size too small!");
        return TSREF; // early return with empty or partial vector
    }
    for (uint8_t stack = 0; stack < numStacks; ++stack) {
        size_t index = frameHeaderSize+(stack * frameSize) ;
            if (index + 1 >= response.size()) {
                Serial.println("Index out of bounds in TSREF!");
                continue;
            }

            uint16_t raw = (response[index] << 8) | response[index + 1];
            double TSREF_volt = convertTo_TREFVolt(raw);
            /* Serial.println("TSREF_volt");
            Serial.println(TSREF_volt); */

            TSREF.push_back(TSREF_volt);
        
    }

    return TSREF;
}

std::vector<std::vector<double>> BQ79600::parseCellVoltages( const std::vector<byte>& response, uint8_t numStacks, uint8_t numCells) {

    std::vector<std::vector<double>> voltages(numStacks, std::vector<double>(numCells));
    const size_t frameHeaderSize = 4;
    const size_t crcSize = 2;
    const size_t dataSizePerStack = numCells * 2;
    const size_t frameSize = frameHeaderSize + dataSizePerStack + crcSize;
    if (response.size() < frameSize * numStacks) {
        Serial.println("Response size too small!");
        return voltages; // early return with empty or partial vector
    }

    for (uint8_t stack = 0; stack < numStacks; ++stack) {
        size_t base = stack * frameSize;

        for (uint8_t cell = 0; cell < numCells; ++cell) {
            size_t index = base + frameHeaderSize + cell * 2;

            if (index + 1 >= response.size()) {
                Serial.println("Index out of bounds in parseCellVoltages2!");
                continue;
            }

            uint16_t raw = (response[index] << 8) | response[index + 1];
            double voltage = convertTo_VoltagCell(raw);

            voltages[stack][cell] = voltage;
        }
    }

    return voltages;
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


uint8_t BQ79600::calcADC_DLY(uint8_t numDevices) {
    float afeSettle = 4000.0f ;
    float propDelay    = (numDevices - 1) * 4.0f;     // µs
    float totalDelay   = propDelay + afeSettle ;
    uint8_t code       = uint8_t(totalDelay / 5.0f + 0.5f); // ปัดขึ้น
    return (code <= 40) ? code : 40;  // จำกัด 0–40
}
void BQ79600::AutoAddressing(){
    bool ok ;
    stack_size_ = NumSegments ;
    // ----- step 1: Write 0x00 to OTP_ECC_DATAIN registers 
    for (int i = 0; i < 8; i++) {
        RegisterAddress reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::OTP_ECC_DATAIN) + i);
        sendCommandTo(RequestType::StackWrite, 1, DEV_ADDR, reg, {0x00}); 
    }

    // ----- step 2:  Broadcast write to enable auto-addressing mode (CONTROL1=0x01) 
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::CONTROL1, {0x01});

    // ----- step 3:  Broadcast write consecutively to DIR0_ADDR = 0, 1, 2, 3..  (register address 0x306) 
    for (int i = 0;i <= stack_size_; i++){
        data_arr_[0] = i;
        sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::DIR0_ADDR, data_arr_);
    }

    // ----- step 4:  Broadcast write to set all devices as stack device first (COMM_CTRL=0x02).
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::COMM_CTRL, {0x02});

    // ----- step 5: Single device write to the highest device in the stack to configure it as both stack and top of stack (COMM_CTRL=0x03).
    sendCommandTo(RequestType::SingleWrite, 1, stack_size_, RegisterAddress::COMM_CTRL, {0x03});

    // ----- step 6:   Dummy stack read registers OTP_ECC_DATAOUT. These are 8 stack read commands
    data_arr_[0] = 0x00;  // Dummy data to read
    for (int i = 0; i < 8; i++) {
        RegisterAddress reg = static_cast<RegisterAddress>(static_cast<uint16_t>(RegisterAddress::OTP_ECC_DATAOUT) + i);
        ok =  this->sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, reg, data_arr_ ,response);
        if (ok) {
            Serial.print("DATAIN"); Serial.print(i + 1);
            Serial.println(" completed successfully."); 
        } else {
            Serial.print("Failed on register "); Serial.println((uint16_t)reg, HEX);
        }
    }

    // ----- step 7:  stack read address 0x306 (read back to verify address are correct for stack devices)
    ok = this->sendAndReceive(RequestType::StackRead, 1, DEV_ADDR, RegisterAddress::DIR0_ADDR ,data_arr_, response);
    if (ok) {
        Serial.print("Stack device address verification from 0x306: ");
    } else {
        Serial.println("Failed to read from address 0x306");
    }

    // ----- step 8:  single device read to BQ79600-Q1, verify 0x2001 = 0x14
    ok = this->sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR,RegisterAddress::DEV_CONF1 ,data_arr_, response);
    if (ok) {
        if (response[4] == 0x14) {
            Serial.println("Verification Passed: Address 0x2001 = 0x14");
            
        } else {
            Serial.println("Verification Failed: Address 0x2001 not equal to 0x14");
        }
    } else {
        Serial.println("Failed to read from address 0x2001");
    }
    

}

bool BQ79600::receiveResponse(std::vector<byte>& response, size_t expected_size, unsigned long timeout_ms) {
    
    response.clear();
    resp_data.clear();
    //response.resize(20);
    //resp_data.resize(20);
    if (!uart) {
        Serial.println("UART not initialized.");
        return false;
    }
    
    unsigned long start_time = millis();

    while ((millis() - start_time) < timeout_ms) {
        
        if (uart->available()) {
            byte incoming = uart->read();
            response.push_back(incoming);
            //Serial.println(response[4], HEX); 
            
            if (response.size() == expected_size) {
                if (!validateCRC(response)) {
                    Serial.println("CRC check failed!");
                    return false;
                }

                Serial.println("Received valid response.");
                for(size_t i = 0; i < response.size(); ++i) {
                    Serial.print(response[i], HEX);
                    Serial.print(" ");
                }
                Serial.println(""); 
                return true;
            }
        }
    }

    Serial.println("Timeout while reading response.");
    return false;
}
bool BQ79600::receiveStackResponse(std::vector<byte>& response, size_t device_count, size_t data_per_device, unsigned long timeout_ms) {
    //beginUart();
    response.clear();
    resp_data.clear();
    //response.resize(device_count*data_per_device);
    //resp_data.resize(20);
    if (!uart) {
        Serial.println("UART not initialized.");
        return false;
        }
    for (size_t i = 0; i < device_count; i++) {
        std::vector<byte> frame;
        size_t expected_frame_size = 4 + data_per_device + 2;  // header + data + CRC
        //frame.resize(expected_frame_size);
        unsigned long start_time = millis();

        while ((millis() - start_time) < timeout_ms) {
            if (uart->available()) {
                byte incoming = uart->read();
                frame.push_back(incoming);

                if (frame.size() == expected_frame_size) {
                    if (!validateCRC(frame)) {
                        Serial.print("CRC failed at device ");
                        Serial.println(i);
                        return false;
                    }

                    // Append this device's frame to the final response
                    response.insert(response.end(), frame.begin(), frame.end());
                    break; // Finish this frame, go to next device
                }
            }
        }
        /* Serial.print("frame ");
        for(int k = 0; k < frame.size(); k++) {
            Serial.print(frame[k], HEX);
            Serial.print(" ");
        }
        Serial.println(" "); */
        // Check if frame is incomplete (timeout case)
        if (frame.size() != expected_frame_size) {
            Serial.print("Timeout or incomplete frame at device ");
            Serial.println(i); 
            return false;
        }
    }
    /* Serial.println("response: ");
    for(int i =0; i < device_count*(4+data_per_device+2) ; i++) {
            Serial.print(response[i ], HEX);  // response[] is the first data byte
            Serial.print(" ");
        }
    Serial.println();
    Serial.println("All stack responses received successfully."); */
    return true;
}


bool BQ79600::validateCRC(const std::vector<byte>& frame) {
    if (frame.size() < 3) {
        Serial.println("Frame too short for CRC16 check.");
        return false;
    }
    // สร้างสำเนาข้อมูลที่ไม่รวม 2 bytes สุดท้าย (CRC)
    std::vector<byte> dataOnly(frame.begin(), frame.end() - 2);
    // ใช้ CRC16 แบบเดียวกับ BQ79600
    Crc16 crc;
    uint16_t calculated_crc = crc.fastCrc(
        dataOnly.data(),                 // pointer ไปยังข้อมูล
        0,                               // เริ่มต้นที่ index 0
        dataOnly.size() ,                 
        true, true,                      // reflectIn, reflectOut
        0x8005, 0xFFFF, 0x0000,          // poly, xorIn, xorOut
        0x8000, 0xFFFF                   // msbMask, mask
    );

    // ดึง CRC ที่แนบมาจาก frame (MSB อยู่ก่อน)
    uint16_t received_crc = ((uint16_t)frame[frame.size() - 1] << 8) | frame[frame.size() - 2];

    if (calculated_crc != received_crc) {
        Serial.print("CRC mismatch. Expected: 0x");
        Serial.print(received_crc, HEX);
        Serial.print(" Calculated: 0x");
        Serial.println(calculated_crc, HEX);
        return false;
    }

    return true; // CRC ตรงกัน
}

bool BQ79600::sendAndReceive(RequestType req_type, byte data_size, byte dev_addr,RegisterAddress reg_addr, std::vector<byte>& data, std::vector<byte>& response)
{
    bool isSingle = (req_type == RequestType::SingleRead);
    bool isStack  = (req_type == RequestType::StackRead);
    sendCommandTo(req_type, 1, dev_addr, reg_addr, data);
    
        if (isSingle) {
            // คำนวณความยาว Response: header 4 bytes + data + CRC 2 bytes
            size_t expected_response_len = 4 + data_size + 2;
            // รอรับ Response และตรวจ CRC
            return receiveResponse(response, expected_response_len, 500);
            
        }
        if (isStack){
            return receiveStackResponse(response, NumSegments ,data_size, 500);
            
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

void BQ79600::offcommand() {

    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR , RegisterAddress::CONTROL1,{0x40}); // Send WAKE tone up the stack
    

}


void BQ79600::IronManON(){
    data_arr_[0] = 0b00100100; 
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF2 , data_arr_); 
}
void BQ79600::IronManOFF(){
    data_arr_[0] = 0b00101101; 
    sendCommandTo(RequestType::BroadcastWrite, 1, DEV_ADDR, RegisterAddress::GPIO_CONF2 , data_arr_); 
}

bool BQ79600::cheakBalance (){
    

    bool K = true ;
    
    /* data_arr_[0] = 0x00;
    Serial.println("data CUST_CRC_HI");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_HI, data_arr_, response);

    Serial.println("data CUST_CRC_LO ");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_LO , data_arr_, response);

    Serial.println("data CUST_CRC_RSLT_HI");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_RSLT_HI, data_arr_, response);

    Serial.println("data CUST_CRC_RSLT_LO ");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_RSLT_LO , data_arr_, response);
    Serial.println("");  */
    /////////////////////////////////////////////////////////
    //Serial.println("send data CUST_CRC_HI");
    data_arr_[0] = 0x21; 
    sendCommandTo(RequestType::SingleWrite, 2, 0x01, RegisterAddress::CUST_CRC_HI, data_arr_);
    //Serial.println("send CUST_CRC_LO ");
    sendCommandTo(RequestType::SingleWrite, 1, 0x01,RegisterAddress::CUST_CRC_LO , data_arr_);
    K = cheakFaultbase();
    if(K){
        Serial.println(" Clear Fault "); 
        clearFault();
    }
    
    Serial.println(" data BAL_STAT"); 
    data_arr_[0] = 0x00; 
    bool ok = sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::BAL_STAT, data_arr_, response);
    if(ok){
        
        if (response[4] & 0x80){
            Serial.println("Invalid CB setting");
            Serial.println("Go to balance required");
            return false;
        }else if (response[4] & 0x40){
            Serial.println("NTC thermistor measurement is greater than OTCB_THR OR CBFET temp  is greater than CB TWARN");
            return false;
        }else if (response[4] & 0x20){
            Serial.println("cell balancing pause status");
            return true;
        }else if (response[4] & 0x10){
            Serial.println("Module balancing");
            return true;
        }else if (response[4] & 0x08){
            Serial.println("At least 1 cell is in active cell balancing");
            return true;
        }else if (response[4] & 0x04){
            Serial.println(" fault detect");
            return false;
        }else if (response[4] & 0x02){
            Serial.println("Module balancing completed");
            return false;
        }else {
            Serial.println(" All cell balancing is completed");
            return false;
        }
    }
    return false;
}
bool BQ79600::checkFaultBase (){
    
    Serial.println("");
    Serial.println("--------------- summary Fault base -----------------");
    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::FAULT_SUMMARY, data_arr_, response);
    if(response[4] == 0){
            Serial.println(" No Faults Detected");
            return false;
    }else
        if(response[4] & 0x80){
            Serial.println(" Protector fault is detected ");
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
            return true;
        }else if (response[4] & 0x02){
            Serial.println("FAULT_SYS");
            return true;
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_PWR");
            return true;
        }
        
        
        Serial.println("-----------------------------------");
        Serial.println("");
    return true ;
    /* Serial.println("    ");
    Serial.println("FAULT_OTP Fault");
    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::FAULT_OTP, data_arr_, response);  

    Serial.println("data CUST_CRC_HI");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_HI, data_arr_, response);

    Serial.println("data CUST_CRC_LO ");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_LO , data_arr_, response);

    Serial.println("data CUST_CRC_RSLT_HI");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_RSLT_HI, data_arr_, response);

    Serial.println("data CUST_CRC_RSLT_LO ");
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::CUST_CRC_RSLT_LO , data_arr_, response);

    Serial.println("    "); */


    /* Serial.println("FAULT_SYS Fault");
    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::FAULT_SYS, data_arr_, response);
    Serial.println("    ");
    Serial.println("FAULT_PWR1 Fault");
    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, 0x01,RegisterAddress::FAULT_PWR1, data_arr_, response); 
    Serial.println("    ");  */
}
bool BQ79600::checkFaultBrigh(){
    Serial.println("");
    Serial.println("--------------- summary Fault brigh -----------------");
    data_arr_[0] = 0x00; 
    sendAndReceive(RequestType::SingleRead, 1, DEV_ADDR ,RegisterAddress::FAULTM_SUMMARY, data_arr_, response);
    if(response[4] == 0){
            Serial.println(" No Faults Detected");
            return false;
    }else
        if (response[4] & 0x08){
            Serial.println("FAULT_COMM");
            return true;
        }else if (response[4] & 0x04){
            Serial.println("FAULT_REG");
            return true;
        }else if (response[4] & 0x02){
            Serial.println("FAULT_SYS");
            return true;
        }else if(response[4] & 0x01) {
            Serial.println("FAULT_PWR");
            return true;
        }
        
        Serial.println("-----------------------------------");
        Serial.println("");
        return true ;
    
}
void BQ79600::clearFault() {

    // -------------- Clear all faults of Bq79612 -------------------- //
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::SingleWrite, 1, 0x01, RegisterAddress::FAULT_RST2 , data_arr_); 
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::SingleWrite, 1, 0x01, RegisterAddress::FAULT_RST1 , data_arr_);

    // -------------- Clear all faults of Bq79600 -------------------- //
    data_arr_[0] = 0xFF;
    sendCommandTo(RequestType::SingleWrite, 1, DEV_ADDR , RegisterAddress::FAULT_RST , data_arr_); 
}


