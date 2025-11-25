#ifndef BQ79600_H
#define BQ79600_H
#define DEV_ADDR   0x00             // BQ79600 Address  
#define MAX_STACKS 6                // num stack maximum
#define MAX_CELLS_PER_STACK 16      // num cell maximum per stack
#define MAX_TEMPS_PER_STACK 2       // num temp sensor maximum per stack
#define AVERAGE_WINDOW 2          // rolling average  size

#include <Arduino.h>
#include "CRC16.h"

struct BQ79600config {
    uint8_t num_segments ;                       // number of segments
    uint8_t num_cells_series  ;                  // number of cells in series
    uint8_t num_thermistors ;                    // number of thermistors
    float shunt_resistance;                      // shunt resistance value (mohms)
};

struct CellData {
    double voltage;
};

struct StackData {
    CellData cells[MAX_CELLS_PER_STACK];        // array of cell data
    uint8_t numCells;                           // number of cells in the stack
    double gpioTemps[MAX_TEMPS_PER_STACK];       // array of GPIO temp sensors
    uint8_t numTemps;                           // number of temp sensors in the stack
    double dieTemp;                              //  Temp IC BQ79612 of stack 
    double busbarVolt;                           //  busbarVolt of stack           
};

class BQ79600 {
public:
    /// Constructor for BQ79600 class
    uint16_t NumSegments;                       // num segment 
    uint16_t NumCellsSeries;                    // num cell series
    uint16_t NumThermistors;                    // num thermistors
    float ShuntResistance;                      // shunt resistance value (mohms)
    BQ79600(HardwareSerial &serial, uint32_t baud = 1000000, int tx_pin = 17, BQ79600config config = BQ79600config{});    // constructor

    StackData batteryData_pack[MAX_STACKS];     // array of stack data
    
    // Functions (use in main program)
    bool initialize();
    bool AutoAddressing();
    void wakeUp();
    void get_data();
    void BalanceCells(uint8_t mode, size_t stack, const size_t targetCells[], uint8_t targetCellsCount, uint8_t hexThreshold);
    void pauseBalanceCells(uint8_t stack);
    void unpauseBalanceCells(uint8_t stack);
    void cheak();
    void clearFault();


    int checkBalance (uint8_t numstack);  // แก้ด้วยนะ

    //------- Fault Bright functions -------//
    uint8_t FaultM_Summary();
    uint8_t FaultM_Comm1();
    uint8_t FaultM_Comm2();
    uint8_t FaultM_REG();
    uint8_t FaultM_SYS();
    uint8_t FaultM_PWR();

    //------- Fault Stack functions -------//
    void Fault_Summary(uint8_t faults[]);
    void FAULT_PROT1(uint8_t faults[]);
    void FAULT_PROT2(uint8_t faults[]);
    void FAULT_OV1(uint8_t faults[]);
    void FAULT_OV2(uint8_t faults[]);
    void FAULT_UV1(uint8_t faults[]);
    void FAULT_UV2(uint8_t faults[]);
    void FAULT_COMP_VCCB1(uint8_t faults[]);
    void FAULT_COMP_VCCB2(uint8_t faults[]);
    void FAULT_COMP_CBOW1(uint8_t faults[]);
    void FAULT_COMP_CBOW2(uint8_t faults[]);
    void FAULT_COMP_CBFET1(uint8_t faults[]);
    void FAULT_COMP_CBFET2(uint8_t faults[]);
    void FAULT_COMP_GPIO(uint8_t faults[]);
    void FAULT_COMP_MISC(uint8_t faults[]);
    void FAULT_OTP(uint8_t faults[]);
    void Fault_Comm1(uint8_t faults[]);
    void Fault_Comm2(uint8_t faults[]);
    void Fault_Comm3(uint8_t faults[]);
    void FAULT_OT(uint8_t faults[]);
    void FAULT_UT(uint8_t faults[]);
    void FAULT_SYS(uint8_t faults[]);
    void FAULT_PWR1(uint8_t faults[]);
    void FAULT_PWR2(uint8_t faults[]);
    void FAULT_PWR3(uint8_t faults[]);

private:
    ///  Sends a command to the BQ79600 device
    HardwareSerial* uart;
    uint32_t baudRate;
    uint8_t tx_pin_;  // <- เพื่อใช้ใน wakePing()
    
    uint8_t data_arr_[10];              // data array for sending commands 
    uint8_t response[128];              // response array for receiving data
    
    
    //  static arrays 
    float voltageBuffer[MAX_STACKS][MAX_CELLS_PER_STACK][AVERAGE_WINDOW];
    float dieTempBuffer[MAX_STACKS][AVERAGE_WINDOW];
    float busbarBuffer[MAX_STACKS][AVERAGE_WINDOW];
    float gpioTempBuffer[MAX_STACKS][MAX_TEMPS_PER_STACK][AVERAGE_WINDOW];
    
    // index  circular buffer
    uint8_t voltageBufferIndex[MAX_STACKS][MAX_CELLS_PER_STACK];
    uint8_t dieTempBufferIndex[MAX_STACKS];
    uint8_t busbarBufferIndex[MAX_STACKS];
    uint8_t gpioTempBufferIndex[MAX_STACKS][MAX_TEMPS_PER_STACK];
    
    // count of data stored
    uint8_t voltageBufferCount[MAX_STACKS][MAX_CELLS_PER_STACK];
    uint8_t dieTempBufferCount[MAX_STACKS];
    uint8_t busbarBufferCount[MAX_STACKS];
    uint8_t gpioTempBufferCount[MAX_STACKS][MAX_TEMPS_PER_STACK];
    
    size_t averageWindow = AVERAGE_WINDOW;
    
    enum class RegisterAddress : uint16_t 
    { 
        ///---------- BQ79612 Register Addresses ----------------------------------//
        DEV_CONF             =      0x0002,
        ACTIVE_CELL          =      0x0003,
        OTP_ECC_DATAIN       =      0x0343,         // OTP ECC Data In 1
        OTP_ECC_DATAOUT      =      0x0510,         // OTP ECC Data Out 1
        ADC_CTRL1            =      0x030D,         // ADC Control 1
        ADC_CTRL2            =      0x030E,         // ADC Control 2 
        VCELL16_HI           =      0x0568,         // VCELL16 High
        VCELL16_LO           =      0x0569,         // VCELL16 Low
        GPIO_CONF1           =      0x000E,
        GPIO_CONF2           =      0x000F,
        GPIO_CONF3           =      0x0010,
        GPIO_CONF4           =      0x0011,
        ADC_CONF1            =      0x0007,
        ADC_CONF2            =      0x0008,         // ADC Control 2
        DEV_STAT             =      0x052C,         // Device Status
        ADC_STAT1            =      0x0527,
        DEBUG_COMH_BIT       =      0x0783,
        CB_CELL16_CTRL       =      0x0318,
        CB_CELL15_CTRL       =      0x0319,
        CB_CELL14_CTRL       =      0x031A,
        CB_CELL13_CTRL       =      0x031B,    
        CB_CELL12_CTRL       =      0x031C,
        CB_CELL11_CTRL       =      0x031D,
        CB_CELL10_CTRL       =      0x031E,
        CB_CELL9_CTRL        =      0x031F,
        CB_CELL8_CTRL        =      0x0320,
        CB_CELL7_CTRL        =      0x0321,
        CB_CELL6_CTRL        =      0x0322,
        CB_CELL5_CTRL        =      0x0323,
        CB_CELL4_CTRL        =      0x0324,
        CB_CELL3_CTRL        =      0x0325,
        CB_CELL2_CTRL        =      0x0326,
        CB_CELL1_CTRL        =      0x0327,
        BAL_CTRL1            =      0x032E,         // Balance Control 1
        BAL_CTRL2            =      0x032F,         // Balance Control 2
        OTCB_THRESH          =      0x032B,         // Over Temperature Cell Balance Threshold
        VCB_DONE_THRESH      =      0x032A,         // Voltage Cell Balance Done Threshold
        OV_THRESH            =      0x0009,         // Over Voltage Threshold
        UV_THRESH            =      0x000A,         // Under Voltage Threshold
        OVUV_CTRL            =      0x032C,         // Over Voltage and Under Voltage Control
        UV_DISABLE1          =      0x000C,         // Under Voltage Disable 1
        UV_DISABLE2          =      0x000D,         // Under Voltage Disable 2
        OTUT_THRESH          =      0x000B,         // Over Temperature and Under Temperature Threshold
        OTUT_CTRL            =      0x032D,         // Over Temperature and Under Temperature Control   
        GPIO1_HI             =      0x058E,
        DIETEMP1_HI          =      0x05AE,         // Die Temperature 1 High
        DIETEMP1_LO          =      0x05AF,         // Die Temperature 1 Low
        ADC_CTRL3            =      0x030F,         // ADC Control 3
        BAL_STAT             =      0x052B,
        DEBUG_COM_CTRL1      =      0x0701,
        CUST_CRC_HI          =      0x0036,
        CUST_CRC_LO          =      0x0037,
        CUST_CRC_RSLT_HI     =      0x050C,
        CUST_CRC_RSLT_LO     =      0x050D,
        TSREF_HI             =      0x058C,         // TSREF High
        TSREF_LO             =      0x058D,         // TSREF Low
        BUSBAR_HI            =      0x0588,
        BUSBAR_LO            =      0x0589,
        FAULT_SUMMARY        =      0x052D,
        FAULT_COMP_VCCB1     =      0x0545,
        FAULT_COMP_VCCB2     =      0x0546,
        FAULT_COMP_CBOW1     =      0x054B,
        FAULT_COMP_CBOW2     =      0x054C,
        FAULT_COMP_CBFET1    =      0x054E,
        FAULT_COMP_CBFET2    =      0x054F,
        FAULT_COMP_GPIO      =      0x0543,
        FAULT_COMP_MISC      =      0x0550,
        FAULT_OTP            =      0x0535,   
        FAULT_COMM1          =      0x0530,
        FAULT_COMM2          =      0x0531,
        FAULT_COMM3          =      0x0532,
        FAULT_OT             =      0x0540,
        FAULT_UT             =      0x0541,
        FAULT_SYS            =      0x0536,
        FAULT_PWR1           =      0x0552,
        FAULT_PWR2           =      0x0553,
        FAULT_PWR3           =      0x0554,
        FAULT_OV1            =      0x053C,
        FAULT_OV2            =      0x053D,
        FAULT_UV1            =      0x053E,
        FAULT_UV2            =      0x053F,
        FAULT_PROT1          =      0x053A,
        FAULT_PROT2          =      0x053B,
        FAULT_RST1           =      0x0331,
        FAULT_RST2           =      0x0332,
        FAULT_MSK1           =      0x0016,
        FAULT_MSK2           =      0x0017,



        ///------ BQ79600 Register  Addresses ----------------------------------//

        FAULTM_SUMMARY      =       0x2100,           // Fault Summary master
        FAULTM_REG          =       0x2101,           // Register Faul
        FAULTM_SYS          =       0x2102,           // System Fault
        FAULTM_PWR          =       0x2103,           // Power Fault
        FAULTM_COMM1        =       0x2104,           // Communication Fault 1 
        FAULTM_COMM2        =       0x2105,           // Communication Fault 2
        FAULTM_MSK          =       0x2020,           // Fault Mask
        FAULT_RST           =       0x2030,           // Fault Reset
        DIR0_ADDR           =       0x0306,           // Device Address North Direction
        DIR1_ADDR           =       0x0307,           // Device Address South Direction
        COMM_CTRL           =       0x0308,           // Device Address South Direction
        CONTROL1            =       0x0309,           // Control 1
        CONTROL2            =       0x030A,           // Control 2
        DIAG_CTRL           =       0x2000,           // Diagnostic Control
        DEV_CONF1           =       0x2001,           // Device Configure1
        DEV_CONF2           =       0x2002,           // Device Configure2
        TX_HOLD_OFF         =       0x2003,           // Transmitter Hold off Control
        SLP_TIMEOUT         =       0x2004,           // Sleep Timer
        COMM_TIMEOUT        =       0x2005,           // Communication Timeout Control
        SPI_FIFO_UNLOCK     =       0x2010,           // FIFO Diagnostic Unlock
        DEV_DIAG_STAT       =       0x2110,           // Diagnostic Status
        DEBUG_COMM_STAT     =       0x2300,           // Debug Communication Status
        DEBUG_COMM_CTRL     =       0x2201,           // Debug Communication Control
        
    
    };
    
     ///------ command initial frame ---------------------------------------------------//

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

    ///------ Functions (use in BQ79600 class only) ------------------------------------//
    void beginUart1();
    void beginUart2();
    void wakePing();
    void shutdownPing();
    void sleepToActivePing();
    void sendCommandTo(RequestType req_type, byte data_size, byte dev_addr, RegisterAddress reg_addr, uint8_t data[]);
    void config_MainADC(uint8_t numcell,uint8_t numStack);
    void config_AuxADC();
    void config_OV_UV();
    void config_OT_UT();
    void config_Fault();
        
    bool validateCRC(uint8_t frame[],size_t frame_size) ;
    bool sendAndReceive(RequestType req_type, byte data_size, byte dev_addr,RegisterAddress reg_addr, uint8_t data[], uint8_t response[]);
    bool receiveStackResponse(uint8_t response [], size_t expected_size,size_t data_per_device, unsigned long timeout_ms);
    bool receiveResponse(uint8_t response[], size_t expected_size, unsigned long timeout_ms) ;
        
    double convertTo_VoltagCell(uint16_t rawValue) ;
    double convertTo_VoltGPIO(uint16_t rawValue) ;
    double convertTo_dietemp(uint16_t rawValue) ;
    double convertTo_TREFVolt(uint16_t rawValue) ;
    double convertTo_BUSBARVolt(uint16_t rawValue) ;
    double parseBUSBAR_Single(const uint8_t response[], size_t response_size);

    //--------------------------------------//
    uint8_t calculateCRC(const uint8_t *data, uint8_t length);
    uint8_t calcADC_DLY(uint8_t numDevices );
    
    void parseCellVoltages(const uint8_t response[], size_t response_size,double voltages[][MAX_CELLS_PER_STACK], uint8_t numStacks, uint8_t numCells);
    void parseTemp(const uint8_t response[], size_t response_size,const double tsref[], double temps[][MAX_TEMPS_PER_STACK],uint8_t numStacks, uint8_t numNTC);
    void parsedie_Temp(const uint8_t response[], size_t response_size,double dieTemps[], uint8_t numStacks);
    void parseTSREF(const uint8_t response[], size_t response_size,double tsref[], uint8_t numStacks);
    void parseBUSBAR(const uint8_t response[], size_t response_size,double busbars[], uint8_t numStacks);
    
    void addVoltageToBuffer(uint8_t stack, uint8_t cell, float voltage);
    void addDieTempToBuffer(uint8_t stack, float temp);
    void addBusbarToBuffer(uint8_t stack, float voltage);
    void addGpioTempToBuffer(uint8_t stack, uint8_t thermistor, float temp);
    
    float getAverageVoltage(uint8_t stack, uint8_t cell);
    float getAverageDieTemp(uint8_t stack);
    float getAverageBusbar(uint8_t stack);
    float getAverageGpioTemp(uint8_t stack, uint8_t thermistor);
    double calculateTemperatureFromResistance(double resistance);
};  

#endif
