=================
stm32f401rc-rs485
=================

This page discusses issues unique to NuttX configurations for the
NuttX STM32F4-RS485 development board.

.. figure:: stm32f401rc_rs485.jpg
   :align: center

Board information
=================

This board was release on NuttX International Workshop 2023 and developed based on
STM32F401RCT6 microcontroller.

STM32F401RCT6 microcontroller features:
 - Arm 32-bit Cortex®-M4 CPU with FPU
 - 128 Kbytes of Flash memory
 - 64 Kbytes of SRAM
 - Serial wire debug (SWD) & JTAG interfaces
 - Up to 81 I/O ports with interrupt capability
 - Up to 11 communication interfaces
 - Up to 3 I2C interfaces
 - Up to 3 USARTs
 - Up to 4 SPIs
 - SDIO interface
 - USB 2.0 full-speed device/host/OTG controller with on-chip PHY


The board features:

- Digital I2C Temperature Sensor (TMP75)
- 2K bits (256x8) I2C EEPROM
- On-board RS485 Transceiver
- Two Analog Input Stages with Amplifier Buffer
- Two Analog Output Stages with Amplifier Buffer
- MicroSD Connector supporting 1 or 4-bit bus
- Four User LEDs
- Four User Buttons
- USB for DFU (Device Firmware Update) and USB device functionality, as well as powering the board
- Onboard voltage regulator from 5V to 3.3V
- SWD Pins for use as STLink (Pin header) and TC2030-IDC 6-Pin Tag-Connect Plug-of-Nails™ Connector
- Crystal for HS 8MHz
- Crystal for RTC 32.768KHz

Board documentation:
https://github.com/lucaszampar/NuttX_STM32F4_RS485_DevBoard

As F4 series have a USB DFuSe-capable BootROM [AN2606], the board can be flashed
via `dfu-util` over USB, or via `stm32flash` over UART without any debuggers.

LEDs
====

The STM32F4-RS485 has 4 software controllable LEDs.

  =====  =====
  LED    PINS
  =====  =====
  LED_1  PC0
  LED_2  PC1
  LED_4  PC2
  LED_5  PC3
  =====  =====

User Buttons
============

The STM32F4-RS485 has 4 user switches.

  ======= =====
  SWITCH  PINS
  ======  =====
  SWIO_1  PB13
  SWIO_2  PB14
  SWIO_3  PB15
  SWIO_4  PC6
  ======  =====

UARTs
=====

The STM32F4-RS485 has 1 USART available for user.

USART2
------

  ========== =====
  UART/USART PINS
  ========== =====
  CTS        PA0
  RTS        PA1
  TX         PA2  *Warning you make need to reverse RX/TX on
  RX         PA3   some RS-232 converters
  CK         PA4
  ========== =====


SDCARD
=====

The STM32F4-RS485 has 1 SDCard slot connected as below:

  ========== =====
  SDIO       PINS
  ========== =====
  SDIO_D0    PC8
  SDIO_D1    PC9
  SDIO_D2    PC10
  SDIO_D3    PC11
  SDIO_DK    PC12
  ========== =====


EEPROM
======

The STM32F4-RS485 development board has serial EEPROM HX24LC02B, with 2k bits (256x8) and internally
organized with 32 pages of 8 bytes each. It is connected through I2C as below:
  ====== =====
  I2C    PINS
  ====== =====
  SDA    PB7
  SCL    PB8
  ====== =====


Temperature Sensor
==================

The STM32F4-RS485 development board has a temperature sensor TMP75 (TMP75AIDR) connected through I2C as below:

  ====== =====
  I2C    PINS
  ====== =====
  SDA    PB7
  SCL    PB8
  ====== =====


RS485 Transceiver
=====
The STM32F4-RS485 development board has a half-duplex RS-485 transceiver, the BL3085B it is connected
through USART2 as below:
  ==========   =====
  USART2         PINS
  ==========   =====
  USART2_RX    RO
  USART2_RTS   DE, /RE
  USART2_RX    DI
  ==========   =====

A/D Converter
=====
The STM32F4-RS485 development board has two Analog to Digital converters with Amplifier Buffer (1COS724SR)
and connected as below:
  ======= =====
  PWM     PINS
  ======= =====
  PWM_1   PB6
  PWM_2   PA6
  ======= =====

D/C Converter
=====
The STM32F4-RS485 development board has two Digital to Analog converters with Amplifier Buffer (1COS724SR)
and connected as below:
  ======= =====
  ADC     PINS
  ======= =====
  ADC_1   PA0
  ADC_2   PA4
  ======= =====

Configurations
==============

Each stm32f401rc-rs485 configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh stm32f401rc-rs485:<subdir>

  Where <subdir> is one of the following:


Configuration Directories
-------------------------

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on UART2.
