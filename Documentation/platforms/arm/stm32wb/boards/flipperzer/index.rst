============
Flipper Zero
============

This README file discusses the port of NuttX to the Flipper Zero multi-tool
device. See https://flipperzero.one/ for device details.

Device features
===============

- Multi-protocol wireless STM32WB55RGV6 MCU with 1MiB of Flash and 256KiB of SRAM.
- USB Type-C connector for communication and charging
- LiPo battery with charger BQ25896 and fuel gauge BQ27220
- 5-button joystick and a Back button
- ST7565 128x64 LCD
- RGB LED with LP5562 I2C driver
- Buzzer
- Vibration motor
- Micro SD slot connected over SPI
- CC1101 RF transceiver
- ST25R3916 high-performance NFC Universal Device and EMVCo reader
- 125 kHz RFID analog circuit
- IR led
- TSOP75538 IR receiver
- iButton connector
- GPIO connector with power out and SWD pins

Status
======

Oct 2022: initial nsh configuration. LCD seems working.

Programming
===========

The device can be normally flashing and debugging over SWD interface or flashing via
USB interface when the device is in DFU mode using STMicro's STM32CubeProgrammer. The
DFU mode will be activated after pressing two round button for 30s. Original firmware
can be flashed back and stay working until the secure flash area is not changed.

Serial Console
==============

The MCU's USART1 PB6/PB7 pins are available as external GPIO pins 13/14.

Configurations
==============

nsh:
----

Configures the NuttShell (nsh) located at examples/nsh.  This
configuration is focused on low level, command-line driver testing.
