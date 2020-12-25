README for the Espressif ESP Wrover Kit
==============================================

  The ESP32 is a dual-core system from Espressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  peripherals are located on the data bus and/or the instruction bus of
  these CPUs. With some minor exceptions, the address mapping of two CPUs
  is symmetric, meaning they use the same addresses to access the same
  memory. Multiple peripherals in the system can access embedded memory via
  DMA.

ESP-WROVER-KIT is an ESP32-based development board produced by Espressif.

ESP-WROVER-KIT features the following integrated components:
  ESP32-WROVER-B module
  LCD screen
  MicroSD card slot

Its another distinguishing feature is the embedded FTDI FT2232HL chip,
an advanced multi-interface USB bridge. This chip enables to use JTAG
for direct debugging of ESP32 through the USB interface without a separate
JTAG debugger. ESP-WROVER-KIT makes development convenient, easy, and
cost-effective.

Most of the ESP32 I/O pins are broken out to the board’s pin headers for easy access.

Buttons and LEDs
================

  Buttons
  -------
  There are two buttons labeled Boot and EN.  The EN button is not available
  to software.  It pulls the chip enable line that doubles as a reset line.

  The BOOT button is connected to IO0.  On reset it is used as a strapping
  pin to determine whether the chip boots normally or into the serial
  bootloader.  After reset, however, the BOOT button can be used for software
  input.

  LEDs
  ----
  There are several on-board LEDs for that indicate the presence of power
  and USB activity.
  There is an RGB LED available for software.

