Hex mini driver master controller
=================================

This is the main controller code for the Hex single motor mini drive system. It is designed to run on an ESP32-S3 microcontroller and provides the necessary functionality to control 24V servos, communicate via CAN bus and wi-fi.


Settings up dev environment
---------------------------
* Need to install [PlatformIO](https://platformio.org/install/ide?install=vscode) extension for VSCode.
* Might have delete the pio libraries if switching framework versions.
* Install [ESP32-S3 toolchain](https://docs.platformio.org/en/latest/platforms/espressif32.html#installation) if not done automatically by PlatformIO.
* Need to install the [ESP32-S3 JTAG drivers](https://docs.platformio.org/en/latest/boards/espressif32/esp32-s3-devkitc-1.html#debugging) for uploading and debugging over JTAG.