============
SiLabs EFM32
============

SiLabs EFM32 Gecko
------------------

This is a port for the Silicon Laboratories' EFM32
*Gecko* family. Board support is available for the following:

#. **SiLabs EFM32 Gecko Starter Kit (EFM32-G8XX-STK)**. The Gecko
   Starter Kit features:

   -  EFM32G890F128 MCU with 128 kB flash and 16 kB RAM
   -  32.768 kHz crystal (LXFO) and 32 MHz crystal (HXFO)
   -  Advanced Energy Monitoring
   -  Touch slider
   -  4x40 LCD
   -  4 User LEDs
   -  2 pushbutton switches
   -  Reset button and a switch to disconnect the battery.
   -  On-board SEGGER J-Link USB emulator
   -  ARM 20 pin JTAG/SWD standard Debug in/out connector

   **STATUS**. The basic port is verified and available now. This
   includes on-board LED and button support and a serial console
   available on LEUART0. A single configuration is available using the
   NuttShell NSH and the LEUART0 serial console. DMA and USART-based SPI
   supported are included, but not fully tested.

   Refer to the EFM32 Gecko Starter Kit
   `README.txt <https://github.com/apache/nuttx/blob/master/boards/arm/efm32/efm32-g8xx-stk/README.txt>`__
   file for further information.

#. **Olimex EFM32G880F120-STK**. This board features:

   -  EFM32G880F128 with 128 kB flash and 16 kB RAM
   -  32.768 kHz crystal (LXFO) and 32 MHz crystal (HXFO)
   -  LCD custom display
   -  DEBUG connector with ARM 2x10 pin layout for programming/debugging
      with ARM-JTAG-EW
   -  UEXT connector
   -  EXT extension connector
   -  RS232 connector and driver
   -  Four user buttons
   -  Buzzer

   **STATUS**. The board support is complete but untested because of
   tool-related issues. An OpenOCD compatible, SWD debugger would be
   required to make further progress in testing.

   Refer to the Olimex EFM32G880F120-STK
   `README.txt <https://github.com/apache/nuttx/blob/master/boards/arm/efm32/olimex-efm32g880f128-stk/README.txt>`__
   for further information.

SiLabs EFM32 Giant Gecko
------------------------

This is a port for the Silicon Laboratories'
EFM32 *Giant Gecko* family. This board features the EFM32GG990F1024 MCU
with 1 MB flash and 128 kB RAM.

Board support is available for the following:

-  **SiLabs EFM32 Giant Gecko Starter Kit t (EFM32GG-STK3700)**. The
   Gecko Starter Kit features:

   -  EFM32GG990F1024 MCU with 1 MB flash and 128 kB RAM
   -  32.768 kHz crystal (LXFO) and 48 MHz crystal (HXFO)
   -  32 MB NAND flash
   -  Advanced Energy Monitoring
   -  Touch slider
   -  8x20 LCD
   -  2 user LEDs
   -  2 user buttons
   -  USB interface for Host/Device/OTG
   -  Ambient light sensor and inductive-capacitive metal sensor
   -  EFM32 OPAMP footprint
   -  20 pin expansion header
   -  Breakout pads for easy access to I/O pins
   -  Power sources (USB and CR2032 battery)
   -  Backup Capacitor for RTC mode
   -  Integrated Segger J-Link USB debugger/emulator

   **STATUS**.

   -  The basic board support for the *Giant Gecko* was introduced int
      the NuttX source tree in NuttX-7.6. A verified configuration was
      available for the basic NuttShell (NSH) using LEUART0 for the
      serial console.
   -  Development of USB support is in started, but never completed.
   -  Reset Management Unit (RMU) was added Pierre-noel Bouteville in
      NuttX-7.7.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
