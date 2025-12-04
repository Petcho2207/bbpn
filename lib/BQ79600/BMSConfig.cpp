#include "BMSConfig.h"

BMSConfigManager::BMSConfigManager() {
    hw_config_changed = false;
    hw_config_receiving = false;
    prot_limits_receiving = false;
    memset(&buffer_hw_config, 0, sizeof(buffer_hw_config));
    memset(&buffer_prot_limits, 0, sizeof(buffer_prot_limits));
}

void BMSConfigManager::setDefaults() {
    // Hardware Config (float values)
    hw_config.num_segments = 1;
    hw_config.num_cells_series = 10;
    hw_config.num_thermistors = 2;
    hw_config.shunt_resistance = 1.5f;  // 1.5 mΩ
    hw_config.crc = 0;
    
    // Protection Limits (float values)
    prot_limits.temp_max = 35.0f;
    prot_limits.temp_min = 23.0f;
    prot_limits.cell_v_max = 4.3f;
    prot_limits.cell_v_min = 3.1f;
    prot_limits.pack_v_max = 43.0f;
    prot_limits.pack_v_min = 32.0f;
    prot_limits.current_max = 100.0f;
    prot_limits.volt_diff_balance = 0.05f;
    prot_limits.crc = 0;
    
    Serial.println(" Default config loaded");
}

bool BMSConfigManager::initialize() {
    Serial.println("\n========================================");
    Serial.println("=== Initializing BMS Configuration ===");
    Serial.println("========================================\n");
    
    Serial.println("--- Attempting to load from Flash ---");
    bool loaded = loadFromFlash();
    
    if (loaded) {
        Serial.println(" Configuration loaded from Flash");
        
        Serial.println("\n--- Hardware Config ---");
        Serial.printf("  Segments      : %d\n", hw_config.num_segments);
        Serial.printf("  Cells/Series  : %d\n", hw_config.num_cells_series);
        Serial.printf("  Thermistors   : %d\n", hw_config.num_thermistors);
        Serial.printf("  Shunt R       : %.3f Ω\n", hw_config.shunt_resistance);
        
        Serial.println("\n--- Protection Limits ---");
        Serial.printf("  Temp Max      : %.1f °C\n", prot_limits.temp_max);
        Serial.printf("  Temp Min      : %.1f °C\n", prot_limits.temp_min);
        Serial.printf("  Cell V Max    : %.3f V\n", prot_limits.cell_v_max);
        Serial.printf("  Cell V Min    : %.3f V\n", prot_limits.cell_v_min);
        Serial.printf("  Pack V Max    : %.1f V\n", prot_limits.pack_v_max);
        Serial.printf("  Pack V Min    : %.1f V\n", prot_limits.pack_v_min);
        Serial.printf("  Current Max   : %.1f A\n", prot_limits.current_max);
        Serial.printf("  Balance Diff  : %.3f V\n", prot_limits.volt_diff_balance);
        
        Serial.println("\n========================================\n");
        return true;
    }
    
    Serial.println(" No configuration found in Flash");
    setDefaults();
    
    bool saved = saveToFlash();
    if (saved) {
        Serial.println("Default configuration saved to Flash");
    }
    
    Serial.println("========================================\n");
    return saved;
}

bool BMSConfigManager::loadFromFlash() {
   // ========== Step 1: เปิด NVS (Namespace) ==========
    if (!prefs.begin("bms_config", true)) {  // read-only
        Serial.println("Failed to open NVS");
        return false;
    }
    
    size_t len = prefs.getBytes("hw_config", &hw_config, sizeof(hw_config));
    if (len != sizeof(hw_config)) {
        prefs.end();
        return false;
    }
    
    uint16_t crc = calculateCRC(&hw_config, sizeof(hw_config) - 2);
    if (crc != hw_config.crc) {
        Serial.println("HW Config CRC mismatch!");
        prefs.end();
        return false;
    }
    
    len = prefs.getBytes("prot_limits", &prot_limits, sizeof(prot_limits));
    if (len == sizeof(prot_limits)) {
        crc = calculateCRC(&prot_limits, sizeof(prot_limits) - 2);
        if (crc != prot_limits.crc) {
            Serial.println("Protection Limits CRC mismatch!");
            prefs.end();
            return false;
        }
    }
    
    prefs.end();
    return true;
}

bool BMSConfigManager::saveToFlash() {
    if (!prefs.begin("bms_config", false)) {
        Serial.println("Failed to open NVS for writing");
        return false;
    }
    
    hw_config.crc = calculateCRC(&hw_config, sizeof(hw_config) - 2);
    prot_limits.crc = calculateCRC(&prot_limits, sizeof(prot_limits) - 2);
    
    prefs.putBytes("hw_config", &hw_config, sizeof(hw_config));
    prefs.putBytes("prot_limits", &prot_limits, sizeof(prot_limits));
    
    prefs.end();
    Serial.println("Config saved to Flash");
    return true;
}

bool BMSConfigManager::setHardwareConfig(const HardwareConfig& cfg) {
    if (cfg.num_segments < 1 || cfg.num_segments > 6) {
        Serial.println(" Invalid num_segments (1-6)");
        return false;
    }
    if (cfg.num_cells_series < 6 || cfg.num_cells_series > 16) {
        Serial.println(" Invalid num_cells_series (6-16)");
        return false;
    }
    if (cfg.num_thermistors > 6) {
        Serial.println(" Invalid num_thermistors (0-6)");
        return false;
    }
    if (cfg.shunt_resistance <= 0 || cfg.shunt_resistance > 10.0f) {
        Serial.println(" Invalid shunt_resistance (0.001-10 Ω)");
        return false;
    }
    
    if (memcmp(&hw_config, &cfg, sizeof(HardwareConfig) - 2) != 0) {
        hw_config = cfg;
        hw_config_changed = true;
        Serial.println(" Hardware config changed! Re-init required.");
    }
    
    return true;
}

bool BMSConfigManager::setProtectionLimits(const ProtectionLimits& limits) {
    if (limits.temp_max <= limits.temp_min) {
        Serial.println(" Invalid temp limits");
        return false;
    }
    if (limits.cell_v_max <= limits.cell_v_min) {
        Serial.println(" Invalid cell voltage limits");
        return false;
    }
    if (limits.pack_v_max <= limits.pack_v_min) {
        Serial.println(" Invalid pack voltage limits");
        return false;
    }
    
    prot_limits = limits;
    Serial.println(" Protection limits updated (Hot Reload)");
    return true;
}

uint16_t BMSConfigManager::calculateCRC(const void* data, size_t len) {
    const uint8_t* ptr = (const uint8_t*)data;
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

bool BMSConfigManager::processCANCommand(uint16_t can_id, const uint8_t* data, uint8_t len) {
    if (len < 1) return false;
    
    uint8_t cmd = data[0];
    
    switch (cmd) {
        // ========== Hardware Config (5 bytes CAN → float struct) ==========
        case CMD_SET_HW_CONFIG: {
            if (len < 6) {  // 1 cmd + 5 data
                Serial.println(" Invalid HW config size");
                return false;
            }
            
            // Parse CAN data (local variables in .cpp)
            uint8_t num_segments = data[1];
            uint8_t num_cells = data[2];
            uint8_t num_ntc = data[3];
            uint16_t shunt_mohm;  // mΩ
            memcpy(&shunt_mohm, &data[4], 2);
            
            Serial.println(" Received HW config (5 bytes CAN)");
            Serial.printf("   CAN: Shunt R = %u mΩ\n", shunt_mohm);
            
            // Convert to float struct
            buffer_hw_config.num_segments = num_segments;
            buffer_hw_config.num_cells_series = num_cells;
            buffer_hw_config.num_thermistors = num_ntc;
            buffer_hw_config.shunt_resistance = shunt_mohm / 1000.0f;  // mΩ → Ω
            
            Serial.printf("   Converted: %.3f Ω\n", buffer_hw_config.shunt_resistance);
            
            // Calculate CRC
            buffer_hw_config.crc = calculateCRC(&buffer_hw_config, sizeof(buffer_hw_config) - 2);
            Serial.printf("   CRC: 0x%04X\n", buffer_hw_config.crc);
            
            // Apply
            return setHardwareConfig(buffer_hw_config);
        }
        
        // ========== Protection Limits Part 1 (bytes 0-6) ==========
        case CMD_SET_PROTECTION_PART1: {
            if (len < 8) return false;
            
            prot_limits_receiving = true;
            
            // Store raw CAN bytes temporarily
            uint8_t can_buffer[16];
            memset(can_buffer, 0, 16);
            memcpy(can_buffer, &data[1], 7);
            
            // Copy to buffer_prot_limits (as temporary storage)
            memcpy(&buffer_prot_limits, can_buffer, 7);
            
            Serial.println(" Protection part 1/3 (7 bytes)");
            return true;
        }
        
        case CMD_SET_PROTECTION_PART2: {
            if (!prot_limits_receiving) {
                Serial.println(" Received part 2 without part 1");
                return false;
            }
            if (len < 8) return false;
            
            memcpy((uint8_t*)&buffer_prot_limits + 7, &data[1], 7);
            
            Serial.println(" Protection part 2/3 (7 bytes)");
            return true;
        }
        
        case CMD_SET_PROTECTION_PART3: {
            if (!prot_limits_receiving) return false;
            if (len < 3) return false;
            
            // Complete receiving
            uint8_t can_buffer[16];
            memcpy(can_buffer, &buffer_prot_limits, 14);
            memcpy(can_buffer + 14, &data[1], 2);
            
            prot_limits_receiving = false;
            
            Serial.println(" Protection part 3/3 (2 bytes)");
            
            // Parse CAN data (local variables in .cpp)
            int16_t temp_max_scaled, temp_min_scaled;
            uint16_t cell_vmax_mv, cell_vmin_mv;
            uint16_t pack_vmax_mv, pack_vmin_mv;
            uint16_t current_max_scaled, volt_diff_mv;
            
            memcpy(&temp_max_scaled, &can_buffer[0], 2);
            memcpy(&temp_min_scaled, &can_buffer[2], 2);
            memcpy(&cell_vmax_mv, &can_buffer[4], 2);
            memcpy(&cell_vmin_mv, &can_buffer[6], 2);
            memcpy(&pack_vmax_mv, &can_buffer[8], 2);
            memcpy(&pack_vmin_mv, &can_buffer[10], 2);
            memcpy(&current_max_scaled, &can_buffer[12], 2);
            memcpy(&volt_diff_mv, &can_buffer[14], 2);
            
            Serial.printf("   CAN: Cell Vmax = %u mV\n", cell_vmax_mv);
            
            // Convert to float struct
            buffer_prot_limits.temp_max = temp_max_scaled / 10.0f;
            buffer_prot_limits.temp_min = temp_min_scaled / 10.0f;
            buffer_prot_limits.cell_v_max = cell_vmax_mv / 1000.0f;
            buffer_prot_limits.cell_v_min = cell_vmin_mv / 1000.0f;
            buffer_prot_limits.pack_v_max = pack_vmax_mv / 1000.0f;
            buffer_prot_limits.pack_v_min = pack_vmin_mv / 1000.0f;
            buffer_prot_limits.current_max = current_max_scaled / 10.0f;
            buffer_prot_limits.volt_diff_balance = volt_diff_mv / 1000.0f;
            
            Serial.println("   Converted:");
            Serial.printf("     Temp: %.1f - %.1f °C\n", buffer_prot_limits.temp_min, buffer_prot_limits.temp_max);
            Serial.printf("     Cell V: %.3f - %.3f V\n", buffer_prot_limits.cell_v_min, buffer_prot_limits.cell_v_max);
            
            // Calculate CRC
            buffer_prot_limits.crc = calculateCRC(&buffer_prot_limits, sizeof(buffer_prot_limits) - 2);
            Serial.printf("   CRC: 0x%04X\n", buffer_prot_limits.crc);
            
            // Apply
            return setProtectionLimits(buffer_prot_limits);
        }
        
        // ========== System Commands ==========
        case CMD_COMMIT_CONFIG:
            Serial.println(" Configuration committed");
            return true;
        
        case CMD_SAVE_TO_FLASH:
            Serial.println(" Saving to Flash...");
            return saveToFlash();
        
        case CMD_LOAD_FROM_FLASH:
            Serial.println(" Loading from Flash...");
            return loadFromFlash();
        
        case CMD_RESTORE_DEFAULTS:
            Serial.println(" Restoring defaults...");
            setDefaults();
            return saveToFlash();
        
        case CMD_REINIT_BMS:
            Serial.println(" Received REINIT command");
            hw_config_changed = true;
            return true;
        
        default:
            Serial.printf(" Unknown command: 0x%02X\n", cmd);
            return false;
    }
}