# PWM Motor Control System - Technical Reference

## Overview

The BMCU370x features a sophisticated 4-channel PWM motor control system that provides precise, bi-directional filament feeding with closed-loop feedback control. Each channel uses dedicated timer peripherals for independent motor control with PID-based speed and pressure regulation.

## Hardware Architecture

### Timer Configuration
- **TIM2**: Channel 1 motor control (PA15, PB3)
- **TIM3**: Channel 2 motor control (PB4, PB5)  
- **TIM4**: Channels 3 & 4 motor control (PB6, PB7, PB8, PB9)

### PWM Specifications
- **PWM Frequency**: 72kHz (144MHz system clock / 2000)
- **Resolution**: 1000 steps (0-999 duty cycle)
- **Control Range**: ±1000 PWM units
- **Timer Period**: 999 + 1 = 1000 counts
- **Timer Prescaler**: 1 + 1 = 2 (72MHz effective)

## Pin Configuration and Mapping

### GPIO Pin Assignments
```cpp
// Motor control pin mapping
// Channel 1: TIM2_CH1 (PA15) forward, TIM2_CH2 (PB3) reverse
// Channel 2: TIM3_CH1 (PB4) forward, TIM3_CH2 (PB5) reverse  
// Channel 3: TIM4_CH1 (PB6) forward, TIM4_CH2 (PB7) reverse
// Channel 4: TIM4_CH3 (PB8) forward, TIM4_CH4 (PB9) reverse
```

### Timer Remapping
```cpp
// Timer pin remapping configuration
GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2 full remap
GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3 partial remap  
GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4 no remap
```

### GPIO Initialization
```cpp
// Configure PWM output pins
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                              GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // Alternate function push-pull
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
GPIO_Init(GPIOA, &GPIO_InitStructure);
```

## PWM Generation and Control

### Timer Base Configuration
```cpp
// Timer base configuration for all PWM timers
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_TimeBaseStructure.TIM_Period = 999;              // PWM period (1000 steps)
TIM_TimeBaseStructure.TIM_Prescaler = 1;             // Prescaler (divide by 2)
TIM_TimeBaseStructure.TIM_ClockDivision = 0;         // No clock division
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // Up counting

// Apply to all timers
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
```

### PWM Output Configuration
```cpp
// PWM mode configuration
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        // PWM mode 1
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse = 0;                       // Initial duty cycle = 0
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

// Initialize all PWM channels
TIM_OC1Init(TIM2, &TIM_OCInitStructure);  // Channel 1 forward
TIM_OC2Init(TIM2, &TIM_OCInitStructure);  // Channel 1 reverse
TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // Channel 2 forward
TIM_OC2Init(TIM3, &TIM_OCInitStructure);  // Channel 2 reverse
TIM_OC1Init(TIM4, &TIM_OCInitStructure);  // Channel 3 forward
TIM_OC2Init(TIM4, &TIM_OCInitStructure);  // Channel 3 reverse
TIM_OC3Init(TIM4, &TIM_OCInitStructure);  // Channel 4 forward
TIM_OC4Init(TIM4, &TIM_OCInitStructure);  // Channel 4 reverse
```

## Motor Control Implementation

### PWM Output Function
```cpp
void Motion_control_set_PWM(uint8_t CHx, int PWM) {
    uint16_t set1 = 0;  // Forward PWM value
    uint16_t set2 = 0;  // Reverse PWM value
    
    // Determine direction and magnitude
    if (PWM > 0) {
        set1 = PWM;     // Forward direction
        set2 = 0;
    } else if (PWM < 0) {
        set1 = 0;
        set2 = -PWM;    // Reverse direction
    } else {            // PWM == 0
        set1 = 0;       // Stop motor
        set2 = 0;
    }
    
    // Apply PWM values to appropriate timer channels
    switch (CHx) {
        case 0:  // Channel 1
            TIM_SetCompare1(TIM2, set1);  // Forward
            TIM_SetCompare2(TIM2, set2);  // Reverse
            break;
        case 1:  // Channel 2
            TIM_SetCompare1(TIM3, set1);
            TIM_SetCompare2(TIM3, set2);
            break;
        case 2:  // Channel 3
            TIM_SetCompare1(TIM4, set1);
            TIM_SetCompare2(TIM4, set2);
            break;
        case 3:  // Channel 4
            TIM_SetCompare3(TIM4, set1);
            TIM_SetCompare4(TIM4, set2);
            break;
    }
}
```

### Bi-Directional Control
The system uses complementary PWM signals for bi-directional motor control:
- **Forward**: Channel A active, Channel B inactive
- **Reverse**: Channel A inactive, Channel B active  
- **Brake**: Both channels inactive (not both active to prevent shoot-through)

## PID Control System

### PID Controller Class
```cpp
class MOTOR_PID {
private:
    float P = 0;           // Proportional gain
    float I = 0;           // Integral gain  
    float D = 0;           // Derivative gain
    float I_save = 0;      // Integral accumulator
    float E_last = 0;      // Previous error value
    float pid_MAX = 1000;  // Maximum output limit
    float pid_MIN = -1000; // Minimum output limit
    float pid_range = 1000; // Output range for integral limiting

public:
    void init_PID(float P_set, float I_set, float D_set);
    float caculate(float E, float time_E);
    void clear();
};
```

### PID Calculation
```cpp
float MOTOR_PID::caculate(float E, float time_E) {
    // Integral calculation with limiting
    I_save += I * E * time_E;
    if (I_save > pid_range) I_save = pid_range;
    if (I_save < -pid_range) I_save = -pid_range;
    
    // PID output calculation
    float output_buf;
    if (time_E != 0) {  // Prevent division by zero
        output_buf = P * E + I_save + D * (E - E_last) / time_E;
    } else {
        output_buf = P * E + I_save;
    }
    
    // Output limiting
    if (output_buf > pid_MAX) output_buf = pid_MAX;
    if (output_buf < pid_MIN) output_buf = pid_MIN;
    
    E_last = E;  // Store error for next derivative calculation
    return output_buf;
}
```

## Control Modes and Operation

### Speed Control Mode
```cpp
// Speed-based PID control
float speed_set = 50;  // Target speed (mm/s)
float now_speed = calculated_from_AS5600;  // Current speed from encoder
int dir = Motion_control_data_save.Motion_control_dir[CHx];  // Direction

float error = now_speed - speed_set;
int x = dir * PID_speed.caculate(error, time_E);
Motion_control_set_PWM(CHx, x);
```

### Pressure Control Mode  
```cpp
// Pressure-based adaptive control
if (MC_PULL_stu_raw[CHx] < 1.65) {
    // Low pressure - increase feeding
    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.65, time_E, less_pressure);
} else if (MC_PULL_stu_raw[CHx] > 1.7) {
    // High pressure - reduce feeding  
    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, over_pressure);
}
```

### Filament Motion States
```cpp
enum class filament_motion_enum {
    filament_motion_stop,                    // Motor stopped
    filament_motion_send,                    // Forward feeding
    filament_motion_slow_send,               // Slow forward feeding
    filament_motion_pull,                    // Reverse feeding/retraction
    filament_motion_pressure_ctrl_idle,      // Pressure control idle
    filament_motion_pressure_ctrl_on_use     // Active pressure control
};
```

## Advanced Control Features

### Assisted Filament Loading
```cpp
// Automatic filament loading assistance
if (Assist_send_filament[CHx] && is_two) {
    if (MC_ONLINE_key_stu[CHx] == 2) {      // Outer switch triggered
        x = -dir * 666;  // Drive feeding
    }
    if (MC_ONLINE_key_stu[CHx] == 1) {      // Both switches triggered
        if (countdownStart[CHx] == 0) {
            countdownStart[CHx] = get_time64();  // Start timer
        }
        uint64_t now = get_time64();
        if (now - countdownStart[CHx] >= Assist_send_time) {
            x = 0;                               // Stop motor
            Assist_send_filament[CHx] = false;   // Complete assistance
        } else {
            x = -dir * 666;  // Continue feeding
        }
    }
}
```

### Printer-Specific Control
```cpp
// Adaptive control based on printer type
if (device_type == BambuBus_AMS_lite) {
    // AMS Lite pressure-limited feeding
    if (MC_PULL_stu_raw[CHx] < 1.8) {
        speed_set = 50;  // Feed until pressure reached
    } else {
        speed_set = 0;   // Stop when pressure sufficient
    }
} else {
    // Standard AMS full-speed feeding
    speed_set = 50;
}
```

### Retraction Control
```cpp
// Precise retraction distance control
float P1X_OUT_filament_meters = 200.0f;  // Retraction distance (mm)
float last_total_distance[4] = {0.0f};   // Starting position tracking

// Calculate retraction completion
float distance_retracted = last_total_distance[CHx] - current_distance;
if (distance_retracted >= P1X_OUT_filament_meters) {
    Motion_control_set_PWM(CHx, 0);  // Stop retraction
    // Complete retraction sequence
}
```

## Safety and Error Handling

### PWM Limiting
```cpp
#define PWM_lim 1000

// Ensure PWM values stay within safe limits
if (x > PWM_lim) {
    x = PWM_lim;
} else if (x < -PWM_lim) {
    x = -PWM_lim;
}
```

### Emergency Stop
```cpp
// Immediate motor stop for all channels
void emergency_stop_all_motors() {
    for (int i = 0; i < 4; i++) {
        Motion_control_set_PWM(i, 0);
        PID_speed.clear();    // Reset PID state
        PID_pressure.clear(); // Reset pressure PID
    }
}
```

### Error Detection
```cpp
// Monitor for motor control errors
bool check_motor_health(int channel) {
    // Check for encoder feedback
    if (!MC_AS5600.online[channel]) {
        MC_STU_ERROR[channel] = true;
        return false;
    }
    
    // Check for pressure sensor feedback
    if (pressure_sensor_offline[channel]) {
        MC_STU_ERROR[channel] = true;
        return false;
    }
    
    return true;
}
```

## Performance Tuning

### PID Tuning Guidelines
1. **Proportional (P)**: Start with small value, increase until stable
2. **Integral (I)**: Add to eliminate steady-state error
3. **Derivative (D)**: Add to reduce overshoot and improve stability

### Recommended PID Values
```cpp
// Speed control PID (typical values)
PID_speed.init_PID(2.0, 0.1, 0.05);

// Pressure control PID (typical values)  
PID_pressure.init_PID(1.5, 0.05, 0.02);
```

### Performance Monitoring
```cpp
// Monitor control loop timing
uint64_t loop_start_time = get_time64();
// ... control calculations ...
uint64_t loop_time = get_time64() - loop_start_time;

// Log performance metrics
DEBUG_MY("Control loop time: ");
DEBUG_float(loop_time, 2);
DEBUG_MY(" ms\n");
```

## Maintenance and Calibration

### Motor Direction Calibration
```cpp
// Store motor direction settings in flash
struct Motion_control_save_struct {
    int Motion_control_dir[4];  // Direction multiplier for each channel
    int check = 0x40614061;     // Validation marker
} Motion_control_data_save;
```

### Regular Maintenance Tasks
1. **Direction Verification**: Ensure motors turn correct direction
2. **PID Tuning**: Optimize control parameters for material changes
3. **PWM Verification**: Check output waveforms with oscilloscope
4. **Thermal Monitoring**: Verify motors don't overheat
5. **Mechanical Inspection**: Check for wear in drive systems

The PWM motor control system provides the precise, responsive actuation needed for reliable filament management with sophisticated feedback control and safety features.