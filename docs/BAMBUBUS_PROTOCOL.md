# BambuBus Communication Protocol - Technical Reference

## Overview

The BambuBus protocol enables the BMCU370x to communicate with Bambu Lab printers, providing seamless integration with both AMS (Advanced Material System) and AMS Lite configurations. This bidirectional communication protocol handles material management, status reporting, and coordinated printing operations.

## Protocol Architecture

### Communication Parameters
- **Interface**: UART (Serial communication)
- **Baud Rate**: Configurable (typically 115200 bps)
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow Control**: None
- **Buffer Size**: 1000 bytes
- **Error Detection**: CRC8 and CRC16 checksums

### Supported Device Types
```cpp
enum BambuBus_device_type {
    BambuBus_none = 0x0000,      // No device detected
    BambuBus_AMS = 0x0700,       // Full AMS system
    BambuBus_AMS_lite = 0x1200   // AMS Lite system
};
```

## Message Types and Structure

### Message Type Enumeration
```cpp
enum class BambuBus_package_type {
    ERROR = -1,                   // Communication error
    NONE = 0,                     // No message
    filament_motion_short,        // Basic movement commands
    filament_motion_long,         // Extended movement commands
    online_detect,                // Filament presence detection
    REQx6,                        // Request type 6
    NFC_detect,                   // NFC tag detection/reading
    set_filament_info,            // Material information setting
    MC_online,                    // Multi-channel unit status
    read_filament_info,           // Material information request
    set_filament_info_type2,      // Alternative material info format
    version,                      // Firmware version exchange
    serial_number,                // Device serial number
    heartbeat,                    // Keep-alive message
    ETC                          // Other message types
};
```

### Communication Buffer
```cpp
uint8_t BambuBus_data_buf[1000];  // Message buffer
int BambuBus_have_data = 0;       // Data available flag
uint16_t BambuBus_address = 0;    // Device address/type
uint8_t BambuBus_AMS_num = 0;     // AMS unit number (0-3 = A,B,C,D)
```

## Material Information Management

### Filament Data Structure
```cpp
struct _filament {
    // Material identification
    char ID[8] = "GFG00";                    // Material type identifier
    uint8_t color_R = 0xFF;                  // Red color component
    uint8_t color_G = 0xFF;                  // Green color component  
    uint8_t color_B = 0xFF;                  // Blue color component
    uint8_t color_A = 0xFF;                  // Alpha/transparency
    int16_t temperature_min = 220;           // Minimum print temperature
    int16_t temperature_max = 240;           // Maximum print temperature
    char name[20] = "PETG";                  // Material name string
    
    // Usage tracking
    float meters = 0;                        // Filament used (meters)
    uint64_t meters_virtual_count = 0;       // Virtual usage counter
    AMS_filament_stu statu = AMS_filament_stu::online;  // Current status
    
    // Control state
    AMS_filament_motion motion_set = AMS_filament_motion::idle;  // Motion command
    uint16_t pressure = 0xFFFF;              // Pressure reading
};
```

### Filament Status States
```cpp
enum class AMS_filament_stu {
    offline,        // No filament detected
    online,         // Filament present and ready
    NFC_waiting     // Waiting for NFC tag read
};
```

### Filament Motion Commands
```cpp
enum class AMS_filament_motion {
    before_pull_back,   // Prepare for retraction
    need_pull_back,     // Execute retraction
    need_send_out,      // Execute feeding
    on_use,             // Currently in use
    idle                // No motion required
};
```

## Persistent Storage System

### Flash Storage Structure
```cpp
struct alignas(4) flash_save_struct {
    _filament filament[4];                   // 4 channel filament data
    int BambuBus_now_filament_num = 0xFF;    // Currently active channel
    uint8_t filament_use_flag = 0x00;        // Usage flags
    uint32_t version = Bambubus_version;     // Protocol version
    uint32_t check = 0x40614061;             // Validation checksum
} data_save;

#define use_flash_addr ((uint32_t)0x0800F000)  // Flash storage address
```

### Data Persistence Functions
```cpp
// Read configuration from flash
bool Bambubus_read() {
    flash_save_struct *ptr = (flash_save_struct *)(use_flash_addr);
    if ((ptr->check == 0x40614061) && (ptr->version == Bambubus_version)) {
        memcpy(&data_save, ptr, sizeof(data_save));
        return true;
    }
    return false;
}

// Save configuration to flash
void Bambubus_save() {
    Flash_saves(&data_save, sizeof(data_save), use_flash_addr);
}
```

## Protocol Implementation

### Main Communication Loop
```cpp
BambuBus_package_type BambuBus_run() {
    // Process incoming messages
    BambuBus_package_type message_type = parse_incoming_data();
    
    // Handle different message types
    switch (message_type) {
        case BambuBus_package_type::filament_motion_short:
            handle_motion_command();
            break;
        case BambuBus_package_type::online_detect:
            respond_with_filament_status();
            break;
        case BambuBus_package_type::set_filament_info:
            update_material_information();
            break;
        case BambuBus_package_type::heartbeat:
            respond_to_heartbeat();
            break;
        // ... handle other message types
    }
    
    return message_type;
}
```

### Device Detection and Initialization
```cpp
void BambuBus_init() {
    // Initialize UART communication
    BambuBUS_UART_Init();
    
    // Load saved configuration
    if (!Bambubus_read()) {
        // Initialize default configuration
        initialize_default_settings();
    }
    
    // Set initial state
    BambuBus_address = BambuBus_none;
    BambuBus_have_data = 0;
}
```

## Material Management Functions

### Filament Usage Tracking
```cpp
// Reset filament usage counter
void reset_filament_meters(int num) {
    if (num < 4) {
        data_save.filament[num].meters = 0;
    }
}

// Add to filament usage
void add_filament_meters(int num, float meters) {
    if (num < 4) {
        if ((data_save.filament[num].motion_set == AMS_filament_motion::on_use) || 
            (data_save.filament[num].motion_set == AMS_filament_motion::need_pull_back)) {
            data_save.filament[num].meters += meters;
        }
    }
}

// Get current usage
float get_filament_meters(int num) {
    if (num < 4) {
        return data_save.filament[num].meters;
    }
    return 0;
}
```

### Filament Status Management
```cpp
// Set filament online status
void set_filament_online(int num, bool if_online) {
    if (num < 4) {
        if (if_online) {
            data_save.filament[num].statu = AMS_filament_stu::online;
        } else {
            data_save.filament[num].statu = AMS_filament_stu::offline;
        }
    }
}

// Get filament online status
bool get_filament_online(int num) {
    if (num < 4) {
        return (data_save.filament[num].statu == AMS_filament_stu::online);
    }
    return false;
}
```

### Motion Control Interface
```cpp
// Get current motion command for channel
AMS_filament_motion get_filament_motion(int num) {
    if (num < 4) {
        return data_save.filament[num].motion_set;
    }
    return AMS_filament_motion::idle;
}

// Set motion command for channel
void set_filament_motion(int num, AMS_filament_motion motion) {
    if (num < 4) {
        data_save.filament[num].motion_set = motion;
        Bambubus_set_need_to_save();  // Mark for saving
    }
}
```

## Message Processing and Error Handling

### CRC Error Detection
```cpp
#include "CRC16.h"
#include "CRC8.h"

CRC16 crc_16;  // 16-bit CRC for long messages
CRC8 crc_8;    // 8-bit CRC for short messages

// Validate message integrity
bool validate_message_crc(uint8_t *data, int length, uint16_t received_crc) {
    uint16_t calculated_crc = crc_16.calc(data, length);
    return (calculated_crc == received_crc);
}
```

### Connection Status Monitoring
```cpp
// Check if printer is actively printing
bool BambuBus_if_on_print() {
    // Determine print status based on communication state
    // and filament motion commands
    for (int i = 0; i < 4; i++) {
        if (data_save.filament[i].motion_set == AMS_filament_motion::on_use) {
            return true;
        }
    }
    return false;
}
```

### Timeout and Recovery
```cpp
// Handle communication timeouts
static uint64_t last_communication_time = 0;
const uint64_t COMMUNICATION_TIMEOUT = 5000;  // 5 seconds

BambuBus_package_type check_communication_status() {
    uint64_t current_time = get_time64();
    
    if (current_time - last_communication_time > COMMUNICATION_TIMEOUT) {
        // Communication timeout - go offline
        BambuBus_address = BambuBus_none;
        return BambuBus_package_type::ERROR;
    }
    
    return BambuBus_package_type::NONE;
}
```

## Integration with Motion Control

### Motion Command Processing
```cpp
void process_motion_commands() {
    for (int channel = 0; channel < 4; channel++) {
        AMS_filament_motion motion = get_filament_motion(channel);
        
        switch (motion) {
            case AMS_filament_motion::need_send_out:
                // Start feeding operation
                set_motion_mode(channel, filament_motion_enum::filament_motion_send);
                break;
                
            case AMS_filament_motion::need_pull_back:
                // Start retraction operation
                set_motion_mode(channel, filament_motion_enum::filament_motion_pull);
                break;
                
            case AMS_filament_motion::on_use:
                // Active pressure control mode
                set_motion_mode(channel, filament_motion_enum::filament_motion_pressure_ctrl_on_use);
                break;
                
            case AMS_filament_motion::idle:
                // Stop motion
                set_motion_mode(channel, filament_motion_enum::filament_motion_stop);
                break;
        }
    }
}
```

### Status Reporting
```cpp
// Send status updates to printer
void send_status_update() {
    uint8_t status_message[64];
    int msg_length = 0;
    
    // Build status message
    status_message[msg_length++] = BambuBus_AMS_num;  // Unit identifier
    
    // Add filament status for each channel
    for (int i = 0; i < 4; i++) {
        status_message[msg_length++] = get_filament_online(i) ? 1 : 0;
        status_message[msg_length++] = (uint8_t)get_filament_motion(i);
        
        // Add pressure reading
        uint16_t pressure = read_pressure_sensor(i);
        status_message[msg_length++] = pressure & 0xFF;
        status_message[msg_length++] = (pressure >> 8) & 0xFF;
    }
    
    // Add CRC and send
    uint16_t crc = crc_16.calc(status_message, msg_length);
    status_message[msg_length++] = crc & 0xFF;
    status_message[msg_length++] = (crc >> 8) & 0xFF;
    
    send_uart(status_message, msg_length);
}
```

## Advanced Features

### NFC Integration Support
```cpp
// Handle NFC tag detection and reading
void handle_nfc_message() {
    // Process NFC data from printer
    // Update material information based on NFC tag
    // Set filament status to indicate NFC processing
    for (int i = 0; i < 4; i++) {
        if (data_save.filament[i].statu == AMS_filament_stu::NFC_waiting) {
            // Process NFC data for this channel
            process_nfc_material_data(i);
        }
    }
}
```

### Multi-Material Coordination
```cpp
// Coordinate material changes between channels
void coordinate_material_change(int from_channel, int to_channel) {
    // Retract from current channel
    set_filament_motion(from_channel, AMS_filament_motion::need_pull_back);
    
    // Prepare new channel
    set_filament_motion(to_channel, AMS_filament_motion::need_send_out);
    
    // Update active channel
    data_save.BambuBus_now_filament_num = to_channel;
    Bambubus_set_need_to_save();
}
```

### Environmental Monitoring
```cpp
uint8_t AMS_humidity_wet = 12;  // 0-100% humidity reading

// Report environmental conditions
void report_environmental_data() {
    uint8_t env_message[8];
    env_message[0] = AMS_humidity_wet;
    env_message[1] = read_temperature_sensor();
    // Add other environmental data...
    
    send_uart(env_message, sizeof(env_message));
}
```

## Troubleshooting and Diagnostics

### Communication Debugging
```cpp
// Debug message logging
void log_communication_event(BambuBus_package_type msg_type) {
    switch (msg_type) {
        case BambuBus_package_type::ERROR:
            DEBUG_MY("BambuBus_offline\n");
            break;
        case BambuBus_package_type::heartbeat:
            DEBUG_MY("BambuBus_online\n");
            break;
        default:
            DEBUG_MY("BambuBus_message: ");
            DEBUG_float((float)msg_type, 0);
            DEBUG_MY("\n");
    }
}
```

### Protocol Version Management
```cpp
#define Bambubus_version 5  // Current protocol version

// Handle version compatibility
bool check_protocol_compatibility(uint32_t printer_version) {
    // Check if printer version is compatible
    return (printer_version >= 3 && printer_version <= 7);
}
```

The BambuBus communication system provides robust, reliable integration with Bambu Lab printers, enabling advanced material management and coordinated printing operations.