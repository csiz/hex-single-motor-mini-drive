AI Agent Instructions - Hex Mini Driver Master Controller
==========================================================

AI Agent Guidelines
-------------------

### Agent Behavior
- Keep responses concise and focus on a single suggestion.
- Do NOT edit files for multiple solutions in one response! Focus on a single suggestion.

### Code Style & Standards
- Use lower_case for variable and function nanmes, CamelCase for types.
- Prefer to name intermediate value with const qualifiers, ie. `const int c = a + b;`, rely on the compiler to optimize the code.
- Prefer plain data structs, with global functions operating on them. Use classes where it makes more sense or for RAII patterns.
- Comment most lines explaining the functionality and purpose of the code.


Project Overview
----------------

This is an ESP32-S3 based embedded system project for controlling a hexapod robot's mini drive system. The project is built using PlatformIO with the ESP-IDF framework and is designed to control 24 servos, communicate via CAN bus and Wi-Fi.

### Key Technologies
- **Microcontroller**: ESP32-S3 (dual-core, built-in USB-JTAG)
- **Framework**: ESP-IDF (Espressif IoT Development Framework)
- **Build System**: PlatformIO
- **Display**: ST7789 TFT LCD (240x280 pixels, SPI interface)
- **Communication**: CAN bus, Wi-Fi
- **Motor Control**: 24 servo motors

Project Structure
------------------

```
master-code/
├── compile_commands.json     # Clang completion database
├── platformio.ini           # PlatformIO configuration
├── readme.md               # Project documentation
├── boards/
│   └── hex-master-circuit.json  # Custom board definition
├── include/                # Header files
└── src/
    └── main.cpp           # Main application code
```

Hardware Configuration
----------------------

### Display Interface (ST7789 TFT)
- **MOSI**: GPIO 11
- **MISO**: GPIO 13  
- **SCLK**: GPIO 12
- **CS**: GPIO 37
- **DC**: GPIO 38
- **RST**: GPIO 18
- **Resolution**: 240x280 pixels
- **Color Format**: RGB565 (16-bit)

### Status LED
- **Pin**: GPIO 35
- **Control**: PWM via LEDC (8-bit resolution, 1kHz frequency)

Development Environment
------------------------

### Required Tools
1. **PlatformIO** extension for VS Code
2. **ESP32-S3 toolchain** (auto-installed by PlatformIO)
3. **ESP32-S3 JTAG drivers** for debugging and upload

### Build & Upload
- **Platform**: espressif32 @ 6.12.0
- **Board**: hex-master-circuit (custom board definition)
- **Upload Protocol**: esp-builtin (USB-JTAG)
- **Debug Tool**: esp-builtin
- **Monitor Speed**: 115200 baud

