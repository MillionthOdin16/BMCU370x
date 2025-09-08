# AS5600 Magnetic Angle Sensors - Technical Reference

## Overview

The AS5600 sensors in the BMCU370x provide contactless magnetic position sensing for each of the four filament channels. These sensors are crucial for precise filament distance tracking and speed control.

## Hardware Specifications

### AS5600 Sensor Details
- **Manufacturer**: AMS (Austria Micro Systems)
- **Type**: Contactless magnetic rotary position sensor
- **Resolution**: 12-bit (4096 positions per revolution)
- **Interface**: I2C (0x36 address)
- **Supply Voltage**: 3.3V
- **Operating Temperature**: -40°C to +125°C
- **Accuracy**: ±0.1°
- **Magnetic Field Range**: 50-65 mT (optimum)

### Implementation in BMCU370x

#### Pin Assignment
```cpp
// AS5600 I2C pins for 4 channels
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};  // Clock lines
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};   // Data lines
```

#### Channel Mapping
- **Channel 1**: SCL=PB15, SDA=PD0
- **Channel 2**: SCL=PB14, SDA=PC15  
- **Channel 3**: SCL=PB13, SDA=PC14
- **Channel 4**: SCL=PB12, SDA=PC13

## Software Implementation

### Multi-Sensor I2C Driver

The BMCU370x implements a custom software I2C driver to handle multiple AS5600 sensors simultaneously:

```cpp
class AS5600_soft_IIC_many {
public:
    // Sensor status arrays
    bool *online;                    // Connectivity status per sensor
    uint16_t *raw_angle;            // Raw angle readings
    _AS5600_magnet_stu *magnet_stu; // Magnet field status
    
    // Initialize multiple sensors
    void init(uint32_t *GPIO_SCL, uint32_t *GPIO_SDA, int num);
    
    // Update sensor data
    void updata_stu();      // Update sensor status
    void updata_angle();    // Update angle readings
};
```

### Register Map and Functions

#### Key AS5600 Registers
```cpp
#define AS5600_write_address (0x36 << 1)       // I2C write address
#define AS5600_read_address ((0x36 << 1) + 1)  // I2C read address
#define AS5600_raw_angle 0x0C                  // Raw angle register (12-bit)
#define AS5600_angle 0x0E                      // Scaled angle register
#define AS5600_status 0x0B                     // Status register
#define AS5600_agc 0x1A                        // Automatic gain control
#define AS5600_magnitude 0x1B                  // Magnetic field magnitude
```

#### Magnet Status Detection
```cpp
enum _AS5600_magnet_stu {
    low = 1,        // Magnetic field too weak
    high = 2,       // Magnetic field too strong  
    offline = -1,   // Sensor not responding
    normal = 0      // Optimal magnetic field
};
```

## Mechanical Integration

### Magnet Requirements
- **Type**: Diametrically polarized magnet
- **Diameter**: 6mm (typical)
- **Thickness**: 2.5mm (typical)
- **Strength**: Neodymium N35 or stronger
- **Mounting**: Centered on rotation axis

### Sensor Positioning
- **Distance from magnet**: 0.5-3.0mm (optimal: 1.0mm)
- **Alignment**: Sensor center aligned with magnet center
- **Orientation**: Sensor IC facing the magnet pole

## Calibration and Setup

### Initial Configuration
1. Mount magnet on drive wheel shaft
2. Position AS5600 sensor PCB at optimal distance
3. Verify magnetic field strength using AGC register
4. Run calibration routine to establish zero position

### Field Strength Verification
```cpp
// Check magnet field strength
uint16_t magnitude = read_magnitude_register();
if (magnitude < 0x200) {
    // Magnet too weak or too far
} else if (magnitude > 0xF00) {
    // Magnet too strong or too close
}
```

## Usage in Filament Control

### Distance Calculation
```cpp
// Calculate filament distance from wheel rotation
float wheel_circumference = PI * wheel_diameter;  // mm
float distance_per_step = wheel_circumference / 4096;  // mm per angle step
float total_distance = angle_difference * distance_per_step;
```

### Speed Measurement
```cpp
// Calculate filament speed from angle change over time
float angular_velocity = (current_angle - previous_angle) / time_delta;
float linear_speed = angular_velocity * wheel_radius;  // mm/s
```

### Direction Detection
```cpp
// Determine feed direction
if (angle_difference > 0) {
    // Forward feeding
} else if (angle_difference < 0) {
    // Reverse feeding/retraction
}
```

## Error Handling and Diagnostics

### Common Issues

#### Sensor Offline
- **Symptoms**: No I2C response, online status = false
- **Causes**: Wiring issues, power supply problems
- **Solutions**: Check connections, verify 3.3V supply

#### Weak Magnetic Field
- **Symptoms**: magnet_stu = low, erratic readings
- **Causes**: Magnet too far, weak magnet, misalignment
- **Solutions**: Adjust distance, replace magnet, realign

#### Strong Magnetic Field
- **Symptoms**: magnet_stu = high, saturated readings
- **Causes**: Magnet too close, very strong magnet
- **Solutions**: Increase distance, use weaker magnet

#### Noisy Readings
- **Symptoms**: Jittery angle values, unstable speed calculations
- **Causes**: EMI, vibration, loose mounting
- **Solutions**: Add filtering, improve mounting, EMI shielding

### Diagnostic Functions
```cpp
// Check sensor health
bool check_sensor_health(int channel) {
    if (!MC_AS5600.online[channel]) {
        return false;  // Sensor offline
    }
    
    if (MC_AS5600.magnet_stu[channel] != normal) {
        return false;  // Magnetic field issues
    }
    
    return true;  // Sensor healthy
}
```

## Performance Optimization

### Sampling Rate
- **Maximum I2C Speed**: 1MHz (theoretical)
- **Practical Update Rate**: ~1kHz per sensor
- **Recommended Rate**: 100-500Hz for smooth control

### Filtering
```cpp
// Simple moving average filter for angle readings
#define FILTER_SIZE 8
uint16_t angle_buffer[4][FILTER_SIZE];
int filter_index[4] = {0};

uint16_t filtered_angle(int channel, uint16_t new_angle) {
    angle_buffer[channel][filter_index[channel]] = new_angle;
    filter_index[channel] = (filter_index[channel] + 1) % FILTER_SIZE;
    
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += angle_buffer[channel][i];
    }
    return sum / FILTER_SIZE;
}
```

## Maintenance and Troubleshooting

### Regular Maintenance
1. **Visual Inspection**: Check for dust accumulation on sensors
2. **Magnet Check**: Verify magnet is securely mounted
3. **Calibration**: Periodic zero position verification
4. **Connection Check**: Ensure I2C connections are secure

### Troubleshooting Steps
1. Verify power supply (3.3V ±5%)
2. Check I2C bus integrity with oscilloscope
3. Measure magnetic field strength
4. Test sensor with known good magnet
5. Verify PCB trace continuity

## Integration with Motion Control

The AS5600 sensors provide critical feedback for the BMCU's motion control system:

### PID Control Integration
- **Position Feedback**: Current wheel position
- **Speed Feedback**: Calculated from position derivatives  
- **Error Detection**: Encoder health monitoring
- **Calibration**: Automatic zero-point adjustment

### Multi-Channel Coordination
- **Synchronized Reading**: All sensors read simultaneously
- **Cross-Channel Validation**: Detect mechanical issues
- **Load Balancing**: Distribute filament feeding evenly
- **Error Propagation**: Channel errors affect system behavior

This comprehensive sensor system enables the BMCU370x to achieve precise, reliable filament management with real-time feedback and advanced control algorithms.