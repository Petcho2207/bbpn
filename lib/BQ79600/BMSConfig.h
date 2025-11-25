#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>

// ===================== Configuration Structures ===================== //

// Hardware Config (9 bytes total: 7 data + 2 CRC)
struct HardwareConfig {
    uint8_t num_segments;        // 1 byte  [0]
    uint8_t num_cells_series;    // 1 byte  [1]
    uint8_t num_thermistors;     // 1 byte  [2]
    float shunt_resistance;      // 4 bytes [3-6]  (stored as float, received as uint16_t mΩ)
    uint16_t crc;                // 2 bytes [7-8]  (Flash only)
};
// CAN sends: 5 bytes (3×uint8 + 1×uint16 mΩ)
// ESP32 converts: uint16_t mΩ → float Ω (divide by 1000)

// Protection Limits (34 bytes total: 32 data + 2 CRC)
struct ProtectionLimits {
    float temp_max;              // 4 bytes [0-3]   (stored as float, received as int16_t ×10)
    float temp_min;              // 4 bytes [4-7]
    float cell_v_max;            // 4 bytes [8-11]  (stored as float, received as uint16_t mV)
    float cell_v_min;            // 4 bytes [12-15]
    float pack_v_max;            // 4 bytes [16-19]
    float pack_v_min;            // 4 bytes [20-23]
    float current_max;           // 4 bytes [24-27] (stored as float, received as uint16_t ×10)
    float volt_diff_balance;     // 4 bytes [28-31] (stored as float, received as uint16_t mV)
    uint16_t crc;                // 2 bytes [32-33] (Flash only)
};
// CAN sends: 16 bytes (8×int16/uint16)
// ESP32 converts: scaled integers → float

// ===================== CAN Protocol (Command Byte) ===================== //

enum ConfigCommand {
    // Hardware Config (1 frame: 7 bytes data, NO CRC)
    CMD_SET_HW_CONFIG     = 0x01,  // Send 7 bytes (offset 0-6)
    
    // Protection Limits (5 frames: 7+7+7+7+4 bytes, NO CRC)
    CMD_SET_PROTECTION_PART1 = 0x03,  // bytes 0-6 (7 bytes)
    CMD_SET_PROTECTION_PART2 = 0x04,  // bytes 7-13 (7 bytes)
    CMD_SET_PROTECTION_PART3 = 0x05,  // bytes 14-20 (7 bytes)
    CMD_SET_PROTECTION_PART4 = 0x06,  // bytes 21-27 (7 bytes)
    CMD_SET_PROTECTION_PART5 = 0x07,  // bytes 28-31 (4 bytes, NO CRC)
    
    // System Commands
    CMD_COMMIT_CONFIG       = 0x10,  // Apply config
    CMD_SAVE_TO_FLASH       = 0x20,  // Save to Flash
    CMD_LOAD_FROM_FLASH     = 0x21,  // Load from Flash
    CMD_RESTORE_DEFAULTS    = 0x22,  // Restore defaults
    CMD_REINIT_BMS          = 0x30   // Re-init BMS
};

class BMSConfigManager {
private:
    Preferences prefs;
    HardwareConfig hw_config;
    ProtectionLimits prot_limits;
    
    HardwareConfig buffer_hw_config;
    ProtectionLimits buffer_prot_limits;
    
    bool hw_config_changed;
    bool hw_config_receiving;
    bool prot_limits_receiving;
    
    uint16_t calculateCRC(const void* data, size_t len);
    
public:
    BMSConfigManager();
    
    bool initialize();
    bool loadFromFlash();
    bool saveToFlash();
    void setDefaults();
    void clearRestartFlag() { hw_config_changed = false; }
    
    const HardwareConfig& getHardwareConfig() const { return hw_config; }
    const ProtectionLimits& getProtectionLimits() const { return prot_limits; }
    
    bool setHardwareConfig(const HardwareConfig& cfg);
    bool setProtectionLimits(const ProtectionLimits& limits);
    bool needsRestart() const { return hw_config_changed; }
    bool processCANCommand(uint16_t can_id, const uint8_t* data, uint8_t len);

    uint8_t getNumSegments() const { return hw_config.num_segments; }
    uint8_t getNumCellsSeries() const { return hw_config.num_cells_series; }
    uint8_t getNumThermistors() const { return hw_config.num_thermistors; }
    float getShuntResistance() const { return hw_config.shunt_resistance; }
    float getTempMax() const { return prot_limits.temp_max; }
    float getTempMin() const { return prot_limits.temp_min; }
    float getCellVMax() const { return prot_limits.cell_v_max; }
    float getCellVMin() const { return prot_limits.cell_v_min; }
    float getPackVMax() const { return prot_limits.pack_v_max; }
    float getPackVMin() const { return prot_limits.pack_v_min; }
    float getCurrentMax() const { return prot_limits.current_max; }
    float getVoltDiffBalance() const { return prot_limits.volt_diff_balance; }
};

#endif