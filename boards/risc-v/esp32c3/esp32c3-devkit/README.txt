README for the Espressif ESP32-C3 DevKit
=======================================

  The ESP32-C3 DevKit is an entry-level development board equipped with either
  an ESP32-C3-WROOM-02 or an ESP32-C3-MINI-1.
  ESP32-C3-WROOM-02 and ESP32-C3-MINI-1 are SoMs based on the RISC-V ESP32-C3 CPU.

  Most of the I/O pins are broken out to the pin headers on both sides for easy
  interfacing. Developers can either connect peripherals with jumper wires or
  mount ESP32-C3 DevKit on a breadboard.

ESP32-C3 Features
=================
  The ESP32-C3 is an ultra-low-power and highly integrated SoC with a RISC-V
  core and supports 2.4 GHz Wi-Fi and Bluetooth Low Energy.

  * Address Space
    - 800 KB of internal memory address space accessed from the instruction bus
    - 560 KB of internal memory address space accessed from the data bus
    - 1016 KB of peripheral address space
    - 8 MB of external memory virtual address space accessed from the instruction bus
    - 8 MB of external memory virtual address space accessed from the data bus
    - 480 KB of internal DMA address space
  * Internal Memory
    - 384 KB ROM
    - 400 KB SRAM (16 KB can be configured as Cache)
    - 8 KB of SRAM in RTC
  * External Memory
    - Up to 16 MB of external flash
  * Peripherals
    - 35 peripherals
  * GDMA
    - 7 modules are capable of DMA operations.

ESP32-C3 Toolchain
==================

The configurations provided are using a generic RISC-V toolchain to build ESP32-C3 projects.
It can be downloaded from: https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

Second stage bootloader and partition table
===========================================

The NuttX port for now relies on IDF's second stage bootloader to carry on some hardware
initializations.  The binaries for the bootloader and the partition table can be found in
this repository: https://github.com/espressif/esp-nuttx-bootloader
That repository contains a dummy IDF project that's used to build the bootloader and
partition table, these are then presented as Github assets and can be downloaded
from: https://github.com/espressif/esp-nuttx-bootloader/releases
Download bootloader-esp32c3.bin and partition-table-esp32c3.bin and place them
in a folder, the path to this folder will be used later to program them. This
can be: "../esp-bins"

Buttons and LEDs
================

  Buttons
  -------
  There are two buttons labeled Boot and RST.  The RST button is not available
  to software.  It pulls the chip enable line that doubles as a reset line.

  The BOOT button is connected to IO9.  On reset it is used as a strapping
  pin to determine whether the chip boots normally or into the serial
  bootloader.  After reset, however, the BOOT button can be used for software
  input.

  LEDs
  ----
  There is one on-board LED that indicates the presence of power.
  Another WS2812 LED is connected to GPIO8 and is available for software.

Configurations
==============

  nsh
  ---

  Basic configuration to run the NuttShell (nsh).

  gpio
  ____

  This is a test for the GPIO driver.  It uses GPIO1 and GPIO2 as outputs and
  GPIO9 as an interrupt pin.

  At the nsh, we can turn the outputs on and off with the following:
    nsh> gpio -o 1 /dev/gpout0
    nsh> gpio -o 1 /dev/gpout1

    nsh> gpio -o 0 /dev/gpout0
    nsh> gpio -o 0 /dev/gpout1

  We can use the interrupt pin to send a signal when the interrupt fires:
    nsh> gpio -w 14 /dev/gpint2

  The pin is configured as a rising edge interrupt, so after issuing the
  above command, connect it to 3.3V.

  watchdog
  --------

  This configuration tests the watchdog timers. It includes the 2 MWDTS,
  adds driver support, registers the WDTs as devices and includes the watchdog
  example application.

  To test it, just run the following command:

  `nsh> wdog -i /dev/watchdogX`

  Where X ix the watchdog instance.

  watcher
  -------

  This configuration tests the watchdog timers in the capture mode.
  It includes the 2 MWDTS, adds driver support, registers the WDTs as devices
  and includes the watcher and watched example applications.

  To test it, just run the following command:

  ```
  nsh> watcher
  nsh> watched
  ```

Building and flashing
=====================

First make sure that `esptool.py` is installed.  This tool is used to convert
the ELF to a compatible ESP32 image and to flash the image into the board.
It can be installed with: `pip install esptool`.

Configure the NUttX project: `./tools/configure.sh esp32c3-devkit:nsh`
If the project isn't clean, please run `make distclean` before configure.sh or
run configure.sh with -E option.
Run `make` to build the project.  Note that the conversion mentioned above is
included in the build process.  
The esptool.py command to flash all the binaries is `esptool.py --chip esp32c3
--port /dev/ttyUSBXX --baud 921600 write_flash 0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 nuttx.bin`
However, this is also included in the build process and we can use build and flash with:
`make download ESPTOOL_PORT=/dev/ttyUSBXX ESPTOOL_BINDIR=../esp-bins`
The "../esp-bins" path is the path to the folder containing the bootloader and the
partition table for the ESP32-C3 as explained above.
Note that this step is required only one time.  Once the bootloader and partition
table are flashed, we don't need to flash them again.  So subsequent builds
would just require: `make download ESPTOOL_PORT=/dev/ttyUSBXX`

