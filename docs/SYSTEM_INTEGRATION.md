# System Integration and Features - Technical Overview

## Overview

The BMCU370x integrates multiple hardware subsystems and software components to provide a comprehensive filament management solution. This document describes how the various sensors, actuators, and control systems work together to deliver advanced 3D printing capabilities.

## System-Level Architecture

### Core Integration Points
1. **Sensor Fusion**: Combining AS5600 position, pressure, and microswitch data
2. **Motion Control**: Coordinated PWM motor control with multi-modal feedback
3. **Communication**: BambuBus protocol integration with printer systems
4. **Visual Feedback**: RGB LED status indication and material color display
5. **Data Persistence**: Flash storage for configuration and usage tracking

### Real-Time Control Loop
```cpp
void main_control_loop() {
    // 1. Read all sensor inputs
    update_AS5600_sensors();
    read_pressure_and_switches();
    
    // 2. Process communication
    BambuBus_package_type status = BambuBus_run();
    
    // 3. Execute motion control
    if (motion_can_run) {
        Motion_control_run(error_status);
    }
    
    // 4. Update visual feedback
    update_LED_status();
    
    // 5. Handle data persistence
    if (save_required) {
        save_configuration();
    }
}
```

## Multi-Channel Coordination

### Independent Channel Operation
Each of the 4 channels operates independently with its own:
- AS5600 magnetic position sensor
- Pressure monitoring (ADC channel)
- Dual microswitch detection
- PWM motor control
- RGB status LEDs
- Material information storage

### Channel State Management
```cpp
// Per-channel state tracking
struct channel_state {
    // Sensor status
    bool AS5600_online;
    bool pressure_sensor_online;
    int microswitch_state;  // 0=offline, 1=both, 2=outer, 3=inner
    
    // Motion control
    filament_motion_enum current_motion;
    float target_speed;
    float current_speed;
    float pressure_reading;
    
    // Material information
    _filament material_data;
    float usage_meters;
    
    // Error states
    bool error_condition;
    uint64_t last_activity_time;
};

channel_state channels[4];
```

### Load Balancing and Priority
```cpp
// Coordinate multiple channel operations
void coordinate_channels() {
    int active_channels = 0;
    int priority_channel = -1;
    
    // Count active channels and identify priority
    for (int i = 0; i < 4; i++) {
        if (channels[i].current_motion != filament_motion_enum::filament_motion_stop) {
            active_channels++;
            if (channels[i].current_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) {
                priority_channel = i;  // Printing channel has priority
            }
        }
    }
    
    // Adjust power allocation based on load
    if (active_channels > 2) {
        reduce_motor_power_all_channels(0.8f);  // 80% power when busy
    } else {
        restore_motor_power_all_channels();     // Full power when not busy
    }
}
```

## Advanced Control Features

### Automatic Filament Loading
The system provides intelligent filament loading assistance:

```cpp
void handle_assisted_loading(int channel) {
    static uint64_t assist_start_time[4] = {0};
    static bool assist_active[4] = {false};
    
    // Check if assistance is needed
    if (channels[channel].microswitch_state == 2) {  // Outer switch only
        if (!assist_active[channel]) {
            assist_active[channel] = true;
            assist_start_time[channel] = get_time64();
            DEBUG_MY("Starting assisted loading for channel ");
            DEBUG_float(channel, 0);
            DEBUG_MY("\n");
        }
        
        // Continue feeding until both switches active
        set_motor_speed(channel, ASSIST_FEED_SPEED);
        
    } else if (channels[channel].microswitch_state == 1) {  // Both switches
        if (assist_active[channel]) {
            uint64_t assist_duration = get_time64() - assist_start_time[channel];
            
            if (assist_duration >= ASSIST_COMPLETION_TIME) {
                // Assistance complete
                assist_active[channel] = false;
                set_motor_speed(channel, 0);
                DEBUG_MY("Assisted loading complete for channel ");
                DEBUG_float(channel, 0);
                DEBUG_MY("\n");
            } else {
                // Continue feeding for completion time
                set_motor_speed(channel, ASSIST_FEED_SPEED);
            }
        }
    } else {
        // No filament or inner switch only - stop assistance
        assist_active[channel] = false;
        set_motor_speed(channel, 0);
    }
}
```

### Pressure-Based Adaptive Control
```cpp
void adaptive_pressure_control(int channel) {
    float pressure = channels[channel].pressure_reading;
    float target_pressure = get_target_pressure(channel);
    
    // Determine control mode based on pressure deviation
    float pressure_error = pressure - target_pressure;
    
    if (abs(pressure_error) < 0.1f) {
        // Within tolerance - maintain current speed
        return;
    } else if (pressure_error > 0.2f) {
        // High pressure - reduce feed rate
        reduce_feed_rate(channel, pressure_error);
    } else if (pressure_error < -0.2f) {
        // Low pressure - increase feed rate  
        increase_feed_rate(channel, abs(pressure_error));
    }
    
    // Emergency jam detection
    if (pressure_error > 0.5f) {
        emergency_stop_channel(channel);
        set_error_state(channel, "Pressure jam detected");
    }
}
```

### Material-Specific Profiles
```cpp
struct material_profile {
    char name[20];
    float feed_speed_multiplier;
    float pressure_tolerance;
    float retraction_speed;
    float retraction_distance;
    int temperature_range[2];
};

// Pre-defined material profiles
material_profile material_profiles[] = {
    {"PLA",  1.0f, 0.15f, 1.2f, 150.0f, {190, 220}},
    {"PETG", 0.8f, 0.20f, 1.0f, 200.0f, {220, 250}},
    {"ABS",  0.9f, 0.18f, 1.1f, 180.0f, {210, 260}},
    {"TPU",  0.6f, 0.25f, 0.8f, 100.0f, {200, 230}}
};

// Apply material-specific settings
void apply_material_profile(int channel, const char* material_name) {
    for (int i = 0; i < sizeof(material_profiles)/sizeof(material_profile); i++) {
        if (strcmp(material_profiles[i].name, material_name) == 0) {
            channels[channel].material_profile = &material_profiles[i];
            adjust_control_parameters(channel, &material_profiles[i]);
            break;
        }
    }
}
```

## Safety and Error Handling

### Multi-Level Safety System
```cpp
enum safety_level {
    SAFETY_NORMAL = 0,
    SAFETY_WARNING = 1,
    SAFETY_ERROR = 2,
    SAFETY_CRITICAL = 3
};

void evaluate_system_safety() {
    safety_level max_level = SAFETY_NORMAL;
    
    for (int i = 0; i < 4; i++) {
        safety_level channel_level = evaluate_channel_safety(i);
        if (channel_level > max_level) {
            max_level = channel_level;
        }
    }
    
    // Take system-wide action based on highest safety level
    switch (max_level) {
        case SAFETY_WARNING:
            // Reduce speeds, increase monitoring
            reduce_all_speeds(0.8f);
            increase_monitoring_frequency();
            break;
            
        case SAFETY_ERROR:
            // Stop problematic channels
            stop_error_channels();
            send_error_notification();
            break;
            
        case SAFETY_CRITICAL:
            // Emergency stop all operations
            emergency_stop_all();
            enter_safe_mode();
            break;
    }
}
```

### Predictive Error Detection
```cpp
void predictive_error_analysis(int channel) {
    static float pressure_history[4][10] = {0};
    static float speed_history[4][10] = {0};
    static int history_index[4] = {0};
    
    // Update history buffers
    pressure_history[channel][history_index[channel]] = channels[channel].pressure_reading;
    speed_history[channel][history_index[channel]] = channels[channel].current_speed;
    history_index[channel] = (history_index[channel] + 1) % 10;
    
    // Analyze trends
    float pressure_trend = calculate_trend(pressure_history[channel], 10);
    float speed_trend = calculate_trend(speed_history[channel], 10);
    
    // Predict potential issues
    if (pressure_trend > 0.1f && speed_trend < -0.05f) {
        // Increasing pressure with decreasing speed - potential jam
        set_warning_state(channel, "Potential jam developing");
    } else if (pressure_trend < -0.15f && speed_trend > 0.05f) {
        // Decreasing pressure with increasing speed - potential runout
        set_warning_state(channel, "Potential filament runout");
    }
}
```

## Visual Feedback System

### Comprehensive Status Display
```cpp
void update_comprehensive_status() {
    // Update main system LED
    update_system_status_led();
    
    // Update each channel LED
    for (int i = 0; i < 4; i++) {
        update_channel_status_led(i);
    }
}

void update_channel_status_led(int channel) {
    uint8_t r, g, b;
    
    // Determine LED color based on state
    if (channels[channel].error_condition) {
        r = 255; g = 0; b = 0;  // Red for error
    } else if (channels[channel].current_motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) {
        // Show material color when printing
        r = channels[channel].material_data.color_R;
        g = channels[channel].material_data.color_G;
        b = channels[channel].material_data.color_B;
    } else if (channels[channel].microswitch_state == 1) {
        r = 0; g = 255; b = 0;  // Green for ready
    } else if (channels[channel].microswitch_state == 2) {
        r = 255; g = 255; b = 0;  // Yellow for loading
    } else {
        r = 0; g = 0; b = 255;  // Blue for empty
    }
    
    // Apply brightness based on activity
    float brightness = calculate_activity_brightness(channel);
    r = (uint8_t)(r * brightness);
    g = (uint8_t)(g * brightness);
    b = (uint8_t)(b * brightness);
    
    Set_MC_RGB(channel, 0, r, g, b);
}
```

### Dynamic Brightness Control
```cpp
float calculate_activity_brightness(int channel) {
    static uint64_t last_activity[4] = {0};
    uint64_t current_time = get_time64();
    
    // Update activity timestamp
    if (channels[channel].current_motion != filament_motion_enum::filament_motion_stop) {
        last_activity[channel] = current_time;
    }
    
    // Calculate brightness based on recent activity
    uint64_t time_since_activity = current_time - last_activity[channel];
    
    if (time_since_activity < 1000) {
        return 1.0f;  // Full brightness for active channels
    } else if (time_since_activity < 10000) {
        return 0.5f;  // Medium brightness for recently active
    } else {
        return 0.2f;  // Low brightness for idle channels
    }
}
```

## Performance Optimization

### Resource Management
```cpp
void optimize_system_performance() {
    // Adjust sampling rates based on activity
    adjust_adc_sampling_rate();
    adjust_as5600_update_rate();
    
    // Optimize memory usage
    cleanup_old_data();
    compact_history_buffers();
    
    // Balance CPU load
    distribute_processing_load();
}

void adjust_adc_sampling_rate() {
    int active_channels = count_active_channels();
    
    if (active_channels <= 1) {
        // Reduce sampling rate for power savings
        set_adc_sampling_rate(ADC_RATE_LOW);
    } else if (active_channels >= 3) {
        // Increase sampling rate for responsiveness
        set_adc_sampling_rate(ADC_RATE_HIGH);
    } else {
        // Standard sampling rate
        set_adc_sampling_rate(ADC_RATE_NORMAL);
    }
}
```

### Adaptive Control Algorithms
```cpp
void adaptive_control_update() {
    static uint32_t performance_counter = 0;
    performance_counter++;
    
    // Periodically optimize control parameters
    if (performance_counter % 1000 == 0) {
        for (int i = 0; i < 4; i++) {
            if (channels[i].current_motion != filament_motion_enum::filament_motion_stop) {
                optimize_pid_parameters(i);
            }
        }
    }
    
    // Learn from usage patterns
    if (performance_counter % 10000 == 0) {
        update_material_profiles_from_usage();
        save_learned_parameters();
    }
}
```

## System Health Monitoring

### Comprehensive Diagnostics
```cpp
struct system_health {
    // Hardware health
    bool sensors_healthy[4];
    bool motors_healthy[4];
    bool communication_healthy;
    
    // Performance metrics
    float average_response_time;
    uint32_t error_count;
    uint32_t successful_operations;
    
    // Resource usage
    float cpu_usage;
    uint32_t memory_usage;
    float temperature;
};

system_health current_health;

void update_system_health() {
    // Check hardware components
    for (int i = 0; i < 4; i++) {
        current_health.sensors_healthy[i] = check_sensor_health(i);
        current_health.motors_healthy[i] = check_motor_health(i);
    }
    
    // Check communication
    current_health.communication_healthy = check_communication_health();
    
    // Update performance metrics
    update_performance_metrics();
    
    // Log health status
    if (should_log_health()) {
        log_system_health();
    }
}
```

### Maintenance Scheduling
```cpp
void check_maintenance_schedule() {
    static uint64_t last_maintenance_check = 0;
    uint64_t current_time = get_time64();
    
    // Check every hour
    if (current_time - last_maintenance_check > 3600000) {
        last_maintenance_check = current_time;
        
        // Check various maintenance items
        check_calibration_drift();
        check_wear_indicators();
        check_cleaning_requirements();
        
        // Schedule maintenance if needed
        if (maintenance_required()) {
            schedule_maintenance_notification();
        }
    }
}
```

## Integration Benefits

### Synchronized Multi-Material Printing
- Coordinated material changes with minimal waste
- Pressure balancing across channels during printing
- Intelligent purge and prime sequences
- Cross-contamination prevention

### Adaptive Learning
- Material behavior learning from usage patterns
- Automatic parameter optimization
- Predictive maintenance scheduling
- User preference adaptation

### Enhanced Reliability
- Redundant sensor systems for critical functions
- Graceful degradation during component failures
- Comprehensive error recovery procedures
- Real-time health monitoring

### User Experience
- Intuitive visual feedback system
- Automatic material detection and configuration
- Minimal user intervention required
- Comprehensive diagnostic information

The BMCU370x system integration demonstrates how multiple subsystems can work together to create a solution that is greater than the sum of its parts, providing reliable, intelligent, and user-friendly filament management for advanced 3D printing applications.