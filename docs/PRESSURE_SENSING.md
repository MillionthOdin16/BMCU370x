# Pressure Sensing and ADC System - Technical Reference

## Overview

The BMCU370x employs a sophisticated pressure sensing system using the CH32V203's built-in 12-bit ADC with DMA capabilities. This system monitors filament tension across 4 channels simultaneously, enabling real-time pressure-based control and filament jam detection.

## Hardware Architecture

### ADC Specifications
- **Resolution**: 12-bit (4096 levels)
- **Reference Voltage**: 3.3V
- **Sampling Rate**: 72kHz total (9kHz per channel)
- **Channels Used**: 8 channels (PA0-PA7)
- **DMA**: Circular buffer with continuous conversion
- **Filtering**: 256-sample sliding window

### Channel Mapping
```cpp
// ADC channel assignment (PA0-PA7)
// Arrangement: [CH4_pressure, CH4_switch, CH3_pressure, CH3_switch, 
//               CH2_pressure, CH2_switch, CH1_pressure, CH1_switch]
```

Channel assignments:
- **PA0**: Channel 4 pressure sensor
- **PA1**: Channel 4 microswitch detection
- **PA2**: Channel 3 pressure sensor  
- **PA3**: Channel 3 microswitch detection
- **PA4**: Channel 2 pressure sensor
- **PA5**: Channel 2 microswitch detection
- **PA6**: Channel 1 pressure sensor
- **PA7**: Channel 1 microswitch detection

## Pressure Sensor Implementation

### Voltage Thresholds
```cpp
// Pressure control thresholds
float PULL_voltage_up = 1.85f;   // High pressure threshold (red LED)
float PULL_voltage_down = 1.45f; // Low pressure threshold (blue LED)  
float MC_PULL_voltage_pull = 1.5f; // Pull detection threshold
```

### Pressure State Classification
```cpp
// Pressure state evaluation
if (MC_PULL_stu_raw[i] > PULL_voltage_up) {
    MC_PULL_stu[i] = 1;   // High pressure - potential jam
} else if (MC_PULL_stu_raw[i] < PULL_voltage_down) {
    MC_PULL_stu[i] = -1;  // Low pressure - filament slack
} else {
    MC_PULL_stu[i] = 0;   // Normal pressure range
}
```

### Pressure States
1. **High Pressure (+1)**: >1.85V
   - Filament blockage or mechanical obstruction
   - Excessive tension in feed path
   - Requires immediate motor adjustment

2. **Normal Pressure (0)**: 1.45V-1.85V  
   - Optimal operating range
   - Proper filament tension maintained
   - Standard PID control active

3. **Low Pressure (-1)**: <1.45V
   - Filament slack or absence
   - Possible runout condition
   - May require feeding assistance

## ADC Hardware Configuration

### GPIO Setup
```cpp
// Configure ADC input pins
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | 
                              GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | 
                              GPIO_Pin_6 | GPIO_Pin_7;
GPIO_Init(GPIOA, &GPIO_InitStructure);
```

### DMA Configuration
```cpp
// DMA for continuous ADC sampling
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_data;
DMA_InitStructure.DMA_BufferSize = ADC_filter_n * 8;  // 256 samples × 8 channels
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
```

### ADC Configuration
```cpp
// ADC setup for multi-channel scanning
ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
ADC_InitStructure.ADC_ScanConvMode = ENABLE;
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
ADC_InitStructure.ADC_NbrOfChannel = 8;
ADC_InitStructure.ADC_SampleTime = ADC_SampleTime_239Cycles5;
```

## Signal Processing and Filtering

### Sliding Window Filter
```cpp
#define ADC_filter_n_pow 8  // Window size = 2^8 = 256 samples
const int ADC_filter_n = 256;  // Sliding window length

uint16_t ADC_data[ADC_filter_n][8];  // Circular buffer
float ADC_V[8];  // Filtered voltage values
```

### Filter Implementation
```cpp
float *ADC_DMA_get_value() {
    for (int i = 0; i < 8; i++) {
        int data_sum = 0;
        
        // Calculate average over window
        for (int j = 0; j < ADC_filter_n; j++) {
            uint16_t val = ADC_data[j][i];
            int sum = val + ADC_Calibrattion_Val;  // Apply calibration
            
            // Clamp values to valid range
            if (sum < 0 || val == 0) {
                // data_sum += 0;  (no contribution)
            } else if (sum > 4095 || val == 4095) {
                data_sum += 4095;
            } else {
                data_sum += sum;
            }
        }
        
        data_sum >>= ADC_filter_n_pow;  // Divide by 256 (bit shift)
        ADC_V[i] = ((float)data_sum) / 4096 * 3.3;  // Convert to voltage
    }
    
    return ADC_V;
}
```

### Filter Characteristics
- **Window Size**: 256 samples
- **Effective Frequency**: ~15Hz (72kHz / 256 / 8 channels)
- **Noise Reduction**: ~16x (√256)
- **Latency**: ~32ms (256 samples at 8kHz)

## Microswitch Detection System

### Dual Microswitch Configuration
The system supports both single and dual microswitch configurations per channel:

```cpp
bool is_two = true;  // Enable dual microswitch mode
```

### Voltage-Based Switch Detection
```cpp
// Dual microswitch mode logic
if (MC_ONLINE_key_stu_raw[i] < 0.6f) {
    MC_ONLINE_key_stu[i] = 0;  // Offline - no filament
} else if ((MC_ONLINE_key_stu_raw[i] < 1.7f) && (MC_ONLINE_key_stu_raw[i] > 1.4f)) {
    MC_ONLINE_key_stu[i] = 2;  // Outer switch triggered - assist feeding
} else if (MC_ONLINE_key_stu_raw[i] > 1.7f) {
    MC_ONLINE_key_stu[i] = 1;  // Both switches triggered - proper position
} else if (MC_ONLINE_key_stu_raw[i] < 1.4f) {
    MC_ONLINE_key_stu[i] = 3;  // Inner switch only - check runout
}
```

### Switch States
1. **Offline (0)**: <0.6V - No filament detected
2. **Outer Only (2)**: 1.4V-1.7V - Filament at entrance, needs assistance
3. **Both Active (1)**: >1.7V - Filament properly positioned  
4. **Inner Only (3)**: <1.4V - Potential runout condition

## Pressure Control Integration

### PID Control with Pressure Feedback
```cpp
// Pressure-based motor control
if (MC_PULL_stu_raw[CHx] < 1.65) {
    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.65, time_E, pressure_control_enum::less_pressure);
} else if (MC_PULL_stu_raw[CHx] > 1.7) {
    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, pressure_control_enum::over_pressure);
}
```

### Adaptive Pressure Control
The system adjusts feeding behavior based on detected printer type:

```cpp
// AMS Lite pressure control
if (device_type == BambuBus_AMS_lite) {
    if (MC_PULL_stu_raw[CHx] < 1.8) {  // Target pressure reached
        speed_set = 50;
    } else {
        speed_set = 0;  // Hold position
    }
} else {
    speed_set = 50;  // Full speed for AMS
}
```

## Calibration and Maintenance

### ADC Calibration
```cpp
// Hardware calibration sequence
ADC_ResetCalibration(ADC1);
while (ADC_GetResetCalibrationStatus(ADC1));

ADC_StartCalibration(ADC1);
while (ADC_GetCalibrationStatus(ADC1));

ADC_Calibrattion_Val = Get_CalibrationValue(ADC1);
```

### Pressure Sensor Calibration
1. **Zero Pressure Calibration**: Record voltage with no filament
2. **Load Calibration**: Apply known tension and record voltage  
3. **Threshold Adjustment**: Set optimal trigger points
4. **Cross-Channel Validation**: Ensure consistency between channels

### Maintenance Procedures
1. **Regular Calibration**: Monthly ADC and pressure recalibration
2. **Sensor Cleaning**: Remove dust and debris from pressure sensors
3. **Voltage Monitoring**: Check power supply stability
4. **Connection Integrity**: Verify all ADC input connections

## Troubleshooting Guide

### Common Issues

#### Noisy Pressure Readings
- **Symptoms**: Erratic pressure values, unstable control
- **Causes**: EMI, poor connections, power supply noise
- **Solutions**: 
  - Increase filter window size
  - Add hardware filtering capacitors
  - Improve grounding and shielding

#### Incorrect Pressure Thresholds  
- **Symptoms**: False jam detection, poor feeding
- **Causes**: Sensor drift, mechanical changes
- **Solutions**:
  - Recalibrate pressure thresholds
  - Check sensor mounting and alignment
  - Verify mechanical systems

#### ADC Calibration Errors
- **Symptoms**: Offset readings, non-linear response
- **Causes**: Temperature drift, component aging
- **Solutions**:
  - Perform hardware recalibration
  - Check reference voltage stability
  - Replace sensors if necessary

#### Switch Detection Issues
- **Symptoms**: Missed filament detection, false triggers
- **Causes**: Switch wear, contamination, wiring issues
- **Solutions**:
  - Clean switch contacts
  - Adjust mechanical positioning
  - Check wiring continuity

## Performance Optimization

### Sampling Rate Optimization
```cpp
// Adjust sampling rate based on requirements
// Higher rate = better response, more CPU load
// Lower rate = less CPU load, slower response

// Current: 72kHz / 8 channels = 9kHz per channel
// Alternative: 36kHz / 8 channels = 4.5kHz per channel
```

### Memory Usage
```cpp
// Current buffer size
uint16_t ADC_data[256][8];  // 4KB RAM usage

// Optimization options:
// Smaller buffer: ADC_data[128][8] → 2KB RAM, 8Hz filter
// Larger buffer: ADC_data[512][8] → 8KB RAM, 4Hz filter
```

The pressure sensing system provides critical feedback for safe, reliable filament handling with real-time monitoring and adaptive control capabilities.