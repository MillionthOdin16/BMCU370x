# BMCU370x Hardware Documentation

## Overview

The BMCU370x (Bambu Multi-Channel Unit) is a sophisticated 4-channel filament management system designed to interface with Bambu Lab 3D printers. This document provides comprehensive information about the hardware sensors, components, and their functionality within the system.

## System Architecture

The BMCU370x is built around a CH32V203C8T6 RISC-V microcontroller running at 144MHz, providing real-time control for four independent filament channels. Each channel incorporates multiple sensors and actuators for precise filament handling and monitoring.

## Core Hardware Components

### Microcontroller
- **Model**: CH32V203C8T6
- **Architecture**: RISC-V 32-bit
- **Clock Speed**: 144MHz
- **Features**: Advanced timer peripherals, ADC, GPIO, UART
- **Purpose**: Main control unit coordinating all sensors and actuators

## Sensor Systems

### 1. AS5600 Magnetic Angle Sensors

#### Overview
Each channel features an AS5600 contactless magnetic position sensor for precise rotation measurement of the filament drive wheels.

#### Technical Specifications
- **Type**: Contactless magnetic rotary position sensor
- **Resolution**: 12-bit (4096 positions per revolution)
- **Interface**: I2C (software-implemented for multiple sensors)
- **Operating Voltage**: 3.3V
- **Accuracy**: ±0.1°

#### Pin Configuration
```cpp
// AS5600 I2C pins for 4 channels
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};  // Clock lines
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};   // Data lines
```

#### Functionality
- **Position Measurement**: Tracks exact wheel rotation for precise filament feed distance
- **Speed Calculation**: Derives filament feed speed from position changes over time
- **Magnet Status Detection**: Monitors magnetic field strength for sensor health
- **Multi-sensor Support**: Software I2C implementation allows simultaneous operation of 4 sensors

#### Features in BMCU
- Real-time filament distance tracking
- Speed-based PID control for consistent feeding
- Sensor health monitoring and error detection
- Calibration and offset compensation

#### Code Integration
```cpp
class AS5600_soft_IIC_many {
    // Supports multiple AS5600 sensors simultaneously
    bool *online;                    // Sensor connectivity status
    uint16_t *raw_angle;            // Raw angle readings
    _AS5600_magnet_stu *magnet_stu; // Magnet field status
};
```

### 2. Pressure Sensing System (ADC-based)

#### Overview
High-resolution analog pressure sensors monitor filament tension in real-time using the microcontroller's built-in ADC with DMA for continuous sampling.

#### Technical Specifications
- **ADC Resolution**: 12-bit (4096 levels)
- **Channels**: 8 ADC channels (2 per filament channel)
- **Sampling Rate**: ~8kHz per channel
- **Voltage Range**: 0-3.3V
- **Filtering**: 256-sample sliding window filter

#### Pin Configuration
```cpp
// ADC channels mapping (PA0-PA7)
// Channel arrangement: [CH4_pressure, CH4_switch, CH3_pressure, CH3_switch, 
//                       CH2_pressure, CH2_switch, CH1_pressure, CH1_switch]
```

#### Voltage Thresholds
```cpp
float PULL_voltage_up = 1.85f;   // High pressure threshold (red LED)
float PULL_voltage_down = 1.45f; // Low pressure threshold (blue LED)
float MC_PULL_voltage_pull = 1.5f; // Pull detection threshold
```

#### Functionality
- **Pressure Monitoring**: Continuous measurement of filament tension
- **Threshold Detection**: Automatic classification of pressure states
- **Real-time Filtering**: Hardware DMA with software filtering for noise reduction
- **Multi-channel Coordination**: Simultaneous monitoring of all 4 channels

#### Pressure States
1. **High Pressure** (>1.85V): Filament blockage or excessive tension
2. **Normal Pressure** (1.45V-1.85V): Optimal operating range
3. **Low Pressure** (<1.45V): Filament slack or absence

#### Features in BMCU
- Automatic pressure compensation during printing
- Filament jam detection
- Load balancing between channels
- Pressure-based feed rate adjustment

### 3. Filament Detection System (Microswitches)

#### Overview
Dual microswitch system per channel provides precise filament presence and position detection with redundancy for reliability.

#### Configuration Options
- **Single Microswitch Mode**: Basic presence detection
- **Dual Microswitch Mode**: Precise position detection with assisted feeding

#### Pin Configuration
```cpp
// Microswitch detection through ADC voltage levels
// Shared with pressure sensing ADC channels
```

#### Voltage-based Detection States
```cpp
if (MC_ONLINE_key_stu_raw[i] < 0.6f) {
    MC_ONLINE_key_stu[i] = 0;  // Offline - no filament
} else if ((MC_ONLINE_key_stu_raw[i] < 1.7f) && (MC_ONLINE_key_stu_raw[i] > 1.4f)) {
    MC_ONLINE_key_stu[i] = 2;  // Outer switch triggered - assist feeding needed
} else if (MC_ONLINE_key_stu_raw[i] > 1.7f) {
    MC_ONLINE_key_stu[i] = 1;  // Both switches triggered - filament in position
} else if (MC_ONLINE_key_stu_raw[i] < 1.4f) {
    MC_ONLINE_key_stu[i] = 3;  // Inner switch only - check for runout
}
```

#### Detection States
1. **Offline (0)**: No filament detected
2. **Outer Triggered (2)**: Filament at entrance, assisted feeding required
3. **Both Triggered (1)**: Filament properly positioned
4. **Inner Only (3)**: Potential filament runout condition

#### Features in BMCU
- Automatic filament loading assistance
- Precise insertion depth control
- Runout detection and prevention
- Redundant detection for reliability

## Actuator Systems

### 1. PWM Motor Control System

#### Overview
Four independent PWM-controlled motors provide precise filament feeding and retraction with closed-loop control.

#### Technical Specifications
- **PWM Frequency**: 72kHz (144MHz / 2000)
- **Resolution**: 1000 steps (0-999)
- **Timers Used**: TIM2, TIM3, TIM4
- **Control Range**: ±1000 PWM units

#### Pin Mapping
```cpp
// Motor control pins
// Channel 1: TIM2_CH1 (PA15) forward, TIM2_CH2 (PB3) reverse
// Channel 2: TIM3_CH1 (PB4) forward, TIM3_CH2 (PB5) reverse  
// Channel 3: TIM4_CH1 (PB6) forward, TIM4_CH2 (PB7) reverse
// Channel 4: TIM4_CH3 (PB8) forward, TIM4_CH4 (PB9) reverse
```

#### Control Modes
1. **Speed Control**: PID-controlled constant speed feeding
2. **Pressure Control**: Pressure-feedback adaptive feeding
3. **Position Control**: Precise distance-based movement
4. **Manual Control**: Direct PWM output for testing

#### PID Controller Implementation
```cpp
class MOTOR_PID {
    float P, I, D;           // PID gains
    float I_save;            // Integral accumulator
    float E_last;            // Previous error
    float pid_MAX = 1000;    // Maximum output
    float pid_MIN = -1000;   // Minimum output
};
```

#### Features in BMCU
- Individual motor control per channel
- Bi-directional operation (feed/retract)
- Closed-loop feedback control
- Configurable speed and acceleration profiles

### 2. RGB LED Status System

#### Overview
WS2812-based RGB LED system provides visual feedback for system status and individual channel states.

#### LED Configuration
```cpp
// Channel RGB LEDs (2 LEDs per channel)
#define LED_PA11_NUM 2  // Channel 1
#define LED_PA8_NUM 2   // Channel 2  
#define LED_PB1_NUM 2   // Channel 3
#define LED_PB0_NUM 2   // Channel 4
#define LED_PD1_NUM 1   // Main system status
```

#### Status Indication
- **Main Board LED (PD1)**:
  - Red: System offline/error
  - White: System online and operational
  
- **Channel LEDs (2 per channel)**:
  - LED 0: Channel status indicator
  - LED 1: Filament online/material color indicator

#### Color Coding
- **Red**: Error state, high pressure, or offline
- **Blue**: Low pressure or standby
- **Green**: Normal operation
- **White**: System ready
- **Material Colors**: Displays actual filament color when available

#### Features in BMCU
- Real-time status visualization
- Material color display
- Error indication
- Brightness control for different environments

## Communication Systems

### BambuBus Interface

#### Overview
Serial communication protocol for interfacing with Bambu Lab printers (AMS and AMS Lite systems).

#### Protocol Support
- **AMS (0x0700)**: Full Advanced Material System
- **AMS Lite (0x1200)**: Simplified material system
- **Heartbeat**: Regular status communication
- **Material Information**: Filament type, color, and properties

#### Message Types
```cpp
enum class BambuBus_package_type {
    filament_motion_short,     // Basic movement commands
    filament_motion_long,      // Extended movement commands
    online_detect,             // Presence detection
    NFC_detect,               // NFC tag reading
    set_filament_info,        // Material configuration
    MC_online,                // System status
    heartbeat,                // Keep-alive
    version,                  // Firmware version
    serial_number             // Device identification
};
```

#### Features in BMCU
- Bi-directional printer communication
- Automatic device type detection
- Material information exchange
- Real-time status reporting

## Storage System

### Flash Memory Management

#### Overview
Non-volatile storage for system configuration, material information, and operational parameters.

#### Storage Areas
- **Motion Control Data (0x0800E000)**: Motor calibration and direction settings
- **BambuBus Data (0x0800F000)**: Material information and system configuration

#### Stored Information
```cpp
struct _filament {
    char ID[8];                    // Material identifier
    uint8_t color_R, color_G, color_B, color_A;  // RGBA color values
    int16_t temperature_min, temperature_max;     // Temperature range
    char name[20];                 // Material name
    float meters;                  // Usage tracking
    AMS_filament_stu statu;       // Current status
    AMS_filament_motion motion_set; // Motion state
};
```

#### Features in BMCU
- Persistent configuration storage
- Material usage tracking
- System calibration retention
- Wear leveling and data integrity

## System Integration Features

### Multi-Channel Coordination
- Independent operation of 4 channels
- Load balancing and priority management
- Coordinated material changes
- Simultaneous multi-color printing support

### Safety Systems
- Over-pressure protection
- Motor overcurrent detection
- Filament jam prevention
- Emergency stop capabilities

### Diagnostic Features
- Real-time sensor monitoring
- Error logging and reporting
- Performance metrics tracking
- Debug output via UART

### Adaptive Control
- Material-specific feeding profiles
- Environmental compensation
- Wear compensation
- Learning algorithms for optimal performance

## Conclusion

The BMCU370x represents a sophisticated integration of multiple sensor systems, actuators, and control algorithms to provide reliable, precise filament management. The combination of magnetic position sensing, pressure monitoring, dual-switch detection, and closed-loop motor control enables advanced features like automatic loading, pressure compensation, and multi-material printing support.

Each hardware component is carefully selected and integrated to work together as a cohesive system, providing the foundation for advanced 3D printing workflows with Bambu Lab printers.