# BMCU370x Documentation Index

## Overview

This directory contains comprehensive technical documentation for the BMCU370x (Bambu Multi-Channel Unit) hardware and software systems. The documentation covers all aspects of the 4-channel filament management system designed for integration with Bambu Lab 3D printers.

## Documentation Structure

### 📖 Main Documentation
- **[HARDWARE_DOCUMENTATION.md](HARDWARE_DOCUMENTATION.md)** - Complete hardware overview and system architecture
- **[SYSTEM_INTEGRATION.md](SYSTEM_INTEGRATION.md)** - System-level integration, features, and coordination

### 🔧 Hardware Components
- **[AS5600_SENSORS.md](AS5600_SENSORS.md)** - Magnetic angle sensors for position/speed measurement
- **[PRESSURE_SENSING.md](PRESSURE_SENSING.md)** - ADC-based pressure monitoring and microswitch detection
- **[PWM_MOTOR_CONTROL.md](PWM_MOTOR_CONTROL.md)** - Motor control system with PID feedback

### 📡 Communication
- **[BAMBUBUS_PROTOCOL.md](BAMBUBUS_PROTOCOL.md)** - Communication protocol with Bambu Lab printers

## Quick Reference

### Hardware Summary
| Component | Quantity | Purpose | Interface |
|-----------|----------|---------|-----------|
| **CH32V203C8T6** | 1 | Main microcontroller | - |
| **AS5600** | 4 | Magnetic position sensors | Software I2C |
| **Pressure Sensors** | 4 | Filament tension monitoring | 12-bit ADC |
| **Microswitches** | 8 | Dual filament detection per channel | ADC voltage levels |
| **PWM Motors** | 4 | Filament feeding/retraction | Timer PWM |
| **WS2812 LEDs** | 9 | Status indication (2 per channel + 1 main) | Bit-banged protocol |

### Key Features
- **Precise Position Control**: 12-bit magnetic encoders with 0.1° accuracy
- **Real-time Pressure Monitoring**: Continuous filament tension feedback
- **Dual Filament Detection**: Redundant microswitch system per channel
- **Adaptive Control**: PID-based speed and pressure regulation
- **Visual Feedback**: RGB status indication with material colors
- **Smart Integration**: BambuBus protocol for printer communication
- **Data Persistence**: Flash storage for configuration and usage tracking

### System Capabilities
- **4 Independent Channels**: Simultaneous multi-material operation
- **Automatic Loading**: Intelligent filament insertion assistance
- **Jam Detection**: Pressure-based blockage prevention
- **Material Tracking**: Usage monitoring and material identification
- **Error Recovery**: Comprehensive safety and diagnostic systems

## Getting Started

### For Hardware Engineers
1. Start with [HARDWARE_DOCUMENTATION.md](HARDWARE_DOCUMENTATION.md) for system overview
2. Review individual component docs for detailed specifications
3. Check [SYSTEM_INTEGRATION.md](SYSTEM_INTEGRATION.md) for coordination details

### For Firmware Developers
1. Review [PWM_MOTOR_CONTROL.md](PWM_MOTOR_CONTROL.md) for control algorithms
2. Study [PRESSURE_SENSING.md](PRESSURE_SENSING.md) for sensor integration
3. Examine [BAMBUBUS_PROTOCOL.md](BAMBUBUS_PROTOCOL.md) for communication

### For System Integrators
1. Read [SYSTEM_INTEGRATION.md](SYSTEM_INTEGRATION.md) for feature overview
2. Check [BAMBUBUS_PROTOCOL.md](BAMBUBUS_PROTOCOL.md) for printer integration
3. Review [HARDWARE_DOCUMENTATION.md](HARDWARE_DOCUMENTATION.md) for requirements

## Technical Specifications

### Performance Metrics
- **Position Resolution**: 4096 steps per revolution (AS5600)
- **Pressure Sampling**: 8kHz per channel with 256-sample filtering
- **PWM Frequency**: 72kHz with 1000-step resolution
- **Communication**: UART-based BambuBus protocol
- **Response Time**: <10ms for motion commands
- **Accuracy**: ±0.1° position, ±0.1V pressure

### Supported Printer Types
- **Bambu Lab AMS**: Full feature support
- **Bambu Lab AMS Lite**: Optimized pressure control
- **Compatible Models**: X1 Carbon, X1E, A1 series, P1 series

### Material Compatibility
- **PLA**: Standard profiles with optimized settings
- **PETG**: Enhanced pressure tolerance
- **ABS**: Temperature-compensated control
- **TPU**: Flexible material support
- **Custom Materials**: Programmable profiles

## Maintenance and Support

### Regular Maintenance
- **Sensor Calibration**: Monthly AS5600 and pressure calibration
- **Mechanical Inspection**: Check for wear and alignment
- **Software Updates**: Keep firmware current for latest features
- **Diagnostic Checks**: Monitor system health indicators

### Troubleshooting Resources
Each component documentation includes:
- Common issues and solutions
- Diagnostic procedures
- Performance optimization tips
- Error code explanations

### Support Resources
- **Hardware Schematics**: Available on OSHWHub
- **Firmware Source**: Open source GPL 2.0 license
- **Community Support**: Wiki and discussion forums
- **Technical Support**: Diagnostic logs and error reporting

## Development and Contribution

### Building the Project
1. Install PlatformIO IDE
2. Clone the repository
3. Open the `src` folder in VS Code
4. Build and upload using PlatformIO

### Customization Options
- **Motor Speed Adjustment**: Modify speed profiles in Motion_control.cpp
- **Retraction Distance**: Configure per-material retraction settings
- **Pressure Thresholds**: Adjust sensitivity for different materials
- **LED Behavior**: Customize status indication patterns

### Contributing
- Follow GPL 2.0 license requirements
- Submit pull requests for improvements
- Report issues through proper channels
- Participate in community discussions

This documentation provides the foundation for understanding, maintaining, and extending the BMCU370x system capabilities.