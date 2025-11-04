#ifndef BQ79600_H
#define BQ79600_H

#include <Arduino.h>
#include "CRC16.h"
#include <vector>
#include <deque>
#define  DEV_ADDR   0x00   // Device Address  


struct BQ79600config {
    uint16_t num_cells_series  ;
    uint16_t num_thermistors ;
    uint16_t num_segments ;
    float r_shunt;  // in ohms

};
struct CellData {
    double voltage;
    
};

struct StackData {
    std::vector<CellData> cells;  // เก็บ volt + gpioTemp ของแต่ละ cell
    std::vector<double> gpioTemps; 
    double dieTemp;               //  dieTemp of stack 
    double busbarVolt;            //  busbarVolt of stack           
};

class BQ79600 {
public:
    uint16_t NumSegments; // จำนวน segment ที่ใช้ใน BQ79600
    uint16_t NumCellsSeries;
    uint16_t NumThermistors;
    float ShuntResistance;
    std::vector<StackData> batteryData_pack;
    
    BQ79600(HardwareSerial &serial, uint32_t baud = 1000000, int tx_pin = 17, BQ79600config config = BQ79600config{});    // constructor
    bool initialize();
    void wakeUp();
    void beginUart();
    bool AutoAddressing();
    bool ReadMainADC();
    bool ReadAuxADC();
    void cheakstatus();
    void get_data();
    void cheakstatus1();
    void offcommand();
    void Test();
    void IronManON();
    void IronManOFF();
    bool SetManualAddress(uint8_t manualAddr, unsigned long timeout_ms) ;
    void ReadTemp();
    void BalanceCells(uint8_t mode, size_t stack, const std::vector<size_t>& targetCells, size_t preservedCell, uint8_t hexThreshold);
    int cheakBalance ();
    bool checkFaultBase();
    bool checkFaultBrigh();
    void clearFault();
    
private:
    HardwareSerial* uart;
    uint32_t baudRate;
    uint8_t tx_pin_;  // <- เพื่อใช้ใน wakePing()
    int stack_size_{0}; // จำนวน stack ที่ใช้ใน BQ79600
    

    std::vector<uint8_t> bq_frame_;
    std::vector<uint8_t> data_arr_;
    std::vector<uint8_t> response;  
    std::vector<uint8_t> resp_data;  
    
    std::vector<std::vector<std::deque<float>>> voltageBuffer;    // [stack][cell]
    std::vector<std::deque<float>> dieTempBuffer;                 // [stack]
    std::vector<std::deque<float>> busbarBuffer;                  // [stack]
    std::vector<std::vector<std::deque<float>>> gpioTempBuffer;   // [stack][thermistor]
    size_t averageWindow = 1; // ขนาดของ window สำหรับ rolling average
    
    enum class RegisterAddress : uint16_t 
    { 
        /// BQ79612 Register Addresses//
        DEV_CONF =  0x0002,
        ACTIVE_CELL     =    0x0003,
        OTP_ECC_DATAIN  =   0x0343,   // OTP ECC Data In 1
        OTP_ECC_DATAOUT = 0x0510,   // OTP ECC Data Out 1
        ADC_CTRL1     =   0x030D,   // ADC Control 1
        ADC_CTRL2    =   0x030E,   // ADC Control 2 
        VCELL16_HI   =   0x0568,   // VCELL16 High
        VCELL16_LO   =   0x0569,   // VCELL16 Low
        GPIO_CONF1   =    0x000E,
        GPIO_CONF2  =    0x000F,
        GPIO_CONF3  =    0x0010,
        GPIO_CONF4  =    0x0011,
        ADC_CONF1   =    0x0007 ,
        ADC_CONF2   =    0x0008,   // ADC Control 2
        DEV_STAT    =     0x052C,   // Device Status
        ADC_STAT1   =     0x0527,
        DEBUG_COMH_BIT = 0x0783,
        FAULT_COMM2 = 0x0531,
        CB_CELL16_CTRL = 0x0318,
        CB_CELL15_CTRL = 0x0319,
        CB_CELL14_CTRL = 0x031A,
        CB_CELL13_CTRL = 0x031B,    
        CB_CELL12_CTRL = 0x031C,
        CB_CELL11_CTRL = 0x031D,
        CB_CELL10_CTRL = 0x031E,
        CB_CELL9_CTRL  = 0x031F,
        CB_CELL8_CTRL  = 0x0320,
        CB_CELL7_CTRL  = 0x0321,
        CB_CELL6_CTRL  = 0x0322,
        CB_CELL5_CTRL  = 0x0323,
        CB_CELL4_CTRL  = 0x0324,
        CB_CELL3_CTRL  = 0x0325,
        CB_CELL2_CTRL  = 0x0326,
        CB_CELL1_CTRL  = 0x0327,
        BAL_CTRL1      =   0x032E,  // Balance Control 1
        BAL_CTRL2      =   0x032F,  // Balance Control 2
        OTCB_THRESH    = 0x032B,  // Over Temperature Cell Balance Threshold
        VCB_DONE_THRESH = 0x032A,  // Voltage Cell Balance Done Threshold
        OV_THRESH      = 0x0009,  // Over Voltage Threshold
        UV_THRESH      = 0x000A,  // Under Voltage Threshold
        OVUV_CTRL    = 0x032C,  // Over Voltage and Under Voltage Control
        UV_DISABLE1 =  0x000C,  // Under Voltage Disable 1
        UV_DISABLE2 =  0x000D,  // Under Voltage Disable 2
        OTUT_THRESH = 0x000B,  // Over Temperature and Under Temperature Threshold
        OTUT_CTRL =  0x032D,  // Over Temperature and Under Temperature Control   
        GPIO1_HI = 0x058E ,
        DIETEMP1_HI = 0x05AE, // Die Temperature 1 High
        DIETEMP1_LO = 0x05AF, // Die Temperature 1 Low
        ADC_CTRL3 = 0x030F, // ADC Control 3
        BAL_STAT = 0x052B,
        FAULT_SUMMARY = 0x052D,
        FAULT_SYS = 0x0536,
        FAULT_OTP = 0x0535 ,
        FAULT_RST1 = 0x0331,
        FAULT_RST2 = 0x0332,
        FAULT_MSK1 = 0x0016,
        FAULT_MSK2 = 0x0017,
        FAULT_PWR1 = 0x0552, // Power Fault 1   
        FAULT_PWR2 = 0x0553, // Power Fault 2
        FAULT_PWR3 = 0x0554, // Communication Fault 1
        CUST_CRC_HI = 0x0036,
        CUST_CRC_LO = 0x0037,
        CUST_CRC_RSLT_HI = 0x050C,
        CUST_CRC_RSLT_LO = 0x050C,
        TSREF_HI = 0x058C, // TSREF High
        TSREF_LO = 0x058D, // TSREF Low
        FAULT_OV1 = 0x053C,
        FAULT_OV2 = 0x053D,
        FAULT_UV1 = 0x053E,
        FAULT_UV2 = 0x053F,
        FAULT_PROT1      = 0x053A,
        FAULT_PROT2      = 0x053B,
        BUSBAR_HI   = 0x0588,
        BUSBAR_LO   = 0x0589,


        /// BQ79600 Register Addresses//
        DIR0_ADDR        =   0x0306,    // Device Address North Direction
        DIR1_ADDR        =   0x0307,    // Device Address South Direction
        COMM_CTRL         =   0x0308,    // Device Address South Direction
        CONTROL1         =   0x0309,    // Control 1
        CONTROL2         =   0x030A,    // Control 2
        DIAG_CTRL        =   0x2000,   // Diagnostic Control
        DEV_CONF1        =   0x2001,   // Device Configure1
        DEV_CONF2        =   0x2002,   // Device Configure2
        TX_HOLD_OFF      =   0x2003,   // Transmitter Hold off Control
        SLP_TIMEOUT      =   0x2004,   // Sleep Timer
        COMM_TIMEOUT     =   0x2005,   // Communication Timeout Control
        SPI_FIFO_UNLOCK  =   0x2010,   // FIFO Diagnostic Unlock
        FAULT_MSK        =   0x2020,   // Fault Mask
        FAULT_RST        =   0x2030,   // Fault Reset
        FAULTM_SUMMARY    =   0x2100,   // Fault Summary master
        FAULTM_REG        =   0x2101,   // Register Faul
        FAULTM_SYS       =   0x2102,   // System Fault 
        FAULTM_PWR        =   0x2103,   // Power Fault
        FAULTM_COMM1      =   0x2104,   // Communication Fault 1 
        FAULTM_COMM2      =   0x2105,   // Communication Fault 2
        DEV_DIAG_STAT    =   0x2110,   // Diagnostic Status
        DEBUG_COMM_STAT  =  0x2300,
        DEBUG_COMM_CTRL  =  0x2201,   // Debug Communication Control
        
    
    };
    


    enum class RequestType : byte
    {
        SingleRead      =    0x80 ,
        SingleWrite     =    0x90 ,
        StackRead       =    0xA0 ,
        StackWrite      =    0xB0 ,
        BroadcastRead   =    0xC0 ,
        BroadcastWrite  =    0xD0 ,
        BroadcastWrite_REV = 0xE0  ,
    };

    
    void wakePing();
    void shutdownPing();
    void sleepToActivePing();
    void sendCommandTo(RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, std::vector<byte> data);
    double convertTo_VoltagCell(uint16_t rawValue) ;
    double convertTo_VoltGPIO(uint16_t rawValue) ;
    double convertTo_dietemp(uint16_t rawValue) ;
    double convertTo_TREFVolt(uint16_t rawValue) ;
    double convertTo_BUSBARVolt(uint16_t rawValue) ;
    bool receiveResponse(std::vector<byte>& response, size_t expected_size, unsigned long timeout_ms);
    bool validateCRC(const std::vector<byte>& frame) ;
    bool sendAndReceive(RequestType req_type, byte data_size, byte dev_addr,RegisterAddress reg_addr, std::vector<byte>& data, std::vector<byte>& response);
    bool receiveStackResponse(std::vector<byte>& response, size_t device_count, size_t data_per_device, unsigned long timeout_ms);
    void config_MainADC(uint8_t numcell,uint8_t numStack);
    void config_AuxADC();
    void config_OV_UV();
    void config_OT_UT();
    void config_Fault();
    void getResponse(std::vector<byte>& response);
    void FaultComm1();
    void FaultComm2();
    void FAULT_OV1();
    void FAULT_OV2();
    void FAULT_UV1();
    void FAULT_UV2();
    void Fault_PWR();
    void Fault_SYS();
    void Fault_REG();
    void FAULT_PROT2();
    void FAULT_PROT1();
    
    uint8_t calculateCRC(const uint8_t *data, uint8_t length);
    uint8_t calcADC_DLY(uint8_t numDevices );
    float calcAverage(const std::deque<float>& history);

    std::vector<std::vector<double>> parseCellVoltages( const std::vector<byte>& response, uint8_t numStacks, uint8_t numCells);
    std::vector<std::vector<double>> parseTemp( const std::vector<byte>& response,const std::vector<double> TSREF, uint8_t numStacks, uint8_t numNTC);
    std::vector<double> parsedie_Temp( const std::vector<byte>& response, uint8_t numStacks);
    std::vector<double> parseTSREF(const std::vector<byte>& response, uint8_t numStacks);
    std::vector<double> parseBUSBAR(const std::vector<byte>& response, uint8_t numStacks);
};

#endif
