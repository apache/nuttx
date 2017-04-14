README
======

This README discusses issues unique to NuttX configurations for the STMicro
Nucleo-64 board family

Contents
========

  - Nucleo-64 Boards
  - Nucleo-F303RE
  - Nucleo-F334R8
  - Nucleo-L476RG
  - Development Environment
  - Basic configuaration & build steps
  - Hardware
    - Button
    - LED
    - U[S]ARTs and Serial Consoles
    - SPI
    - SDIO - MMC
  - Configurations

Nucleo-64 Boards:
=================

The Nucleo-64 is a standard board for use with several STM32 parts in the
LQFP64 package.  Variants include

  Order code    Targeted STM32
  ------------- --------------
  NUCLEO-F030R8 STM32F030R8T6
  NUCLEO-F070RB STM32F070RBT6
  NUCLEO-F072RB STM32F072RBT6
  NUCLEO-F091RC STM32F091RCT6
  NUCLEO-F103RB STM32F103RBT6
  NUCLEO-F302R8 STM32F302R8T6
  NUCLEO-F303RE STM32F303RET6
  NUCLEO-F334R8 STM32F334R8T6
  NUCLEO-F401RE STM32F401RET6
  NUCLEO-F410RB STM32F410RBT6
  NUCLEO-F411RE STM32F411RET6
  NUCLEO-F446RE STM32F446RET6
  NUCLEO-L053R8 STM32L053R8T6
  NUCLEO-L073RZ STM32L073RZT6
  NUCLEO-L152RE STM32L152RET6
  NUCLEO-L452RE STM32L452RET6
  NUCLEO-L476RG STM32L476RGT6

This directory is intended to support all Nucleo-64 variants since the
boards are identical, differing only in the installed part.  This common
board design provides uniformity in the documentation from ST and should
allow us to quickly change configurations by just cloning a configuration
and changing the CPU choice and board initialization.  Unfortunately for
the developer, the CPU specific information must be extracted from the
common information in the documentation.

Please read the User Manaul UM1727: Getting started with STM32 Nucleo board
software development tools and take note of the Powering options for the
board (6.3 Power supply and power selection) and the Solder bridges based
hardware configuration changes that are configurable (6.11 Solder bridges).

Common Board Features:
---------------------

The STM32 Nucleo board offers the following features:

  - STM32 microcontroller in LQFP64 package
  - Two types of extension resources
    Arduino™ Uno V3 connectivity
    ST morpho extension pin headers for full access to all STM32 I/Os
  - ARM® mbed™ (see http://mbed.org)
  - On-board ST-LINK/V2-1 debugger and programmer with SWD connector
    Selection-mode switch to use the kit as a standalone ST-LINK/V2-1
  - Flexible board power supply:
    USB VBUS or external source (3.3V, 5V, 7 - 12V)
    Power management access point
  - Three LEDs:
    USB communication (LD1), user LED (LD2), power LED (LD3)
  - Two push-buttons: USER and RESET
  - USB re-enumeration capability. Three different interfaces supported on USB:
    Virtual COM port
    Mass storage
    Debug port
  - Comprehensive free software HAL library including a variety of software examples

  Peripherals:    8 leds, 2 push button (3 LEDs, 1 button) under software
                  control
  Debug:          STLINK/V2-1 debugger/programmer Uses a STM32F103CB to
                  provide a ST-Link for programming, debug similar to the
                  OpenOcd FTDI function - USB to JTAG front-end.

  Expansion I/F:  ST Zio and Extended Ardino and Morpho Headers

Nucleo-F303RE
=============

Nucleo-F334R8
=============

Nucleo-L476RG
=============

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

  All testing has been conducted using the GNU toolchain from ARM for Linux.
  found here https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

Basic configuration & build steps
==================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M4 GCC toolchain (if
  different from the default in your PATH variable).

   - Configures nuttx creating .config file in the nuttx directory.
     cd tools && ./configure.sh nucleo-f746zg/nsh && cd ..
   - Refreshes the .config file with the latest available configurations.
     make oldconfig
   - Select the features you want in the build.
     make menuconfig
   - Builds Nuttx with the features you selected.
     make

Hardware
========

Serial Console
==============

Configurations
==============

