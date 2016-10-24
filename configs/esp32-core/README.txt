README for the Expressif ESP32 Core board (V2)
==============================================

  The ESP32 is a dual-core system from Expressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  peripherals are located on the data bus and/or the instruction bus of
  these CPUs. With some minor exceptions, the address mapping of two CPUs
  is symmetric, meaning they use the same addresses to access the same
  memory. Multiple peripherals in the system can access embedded memory via
  DMA.

  The two CPUs are named "PRO_CPU" and "APP_CPU" (for "protocol" and
  "application"), however for most purposes the two CPUs are
  interchangeable.

Contents
========

  o STATUS
  o ESP32 Features
  o ESP32 Toolchain
  o Serial Console
  o Buttons and LEDs
  o Configurations

STATUS
======

  The basic port is underway.  No testing has yet been performed.

ESP32 Features
==============

  * Address Space
    - Symmetric address mapping
    - 4 GB (32-bit) address space for both data bus and instruction bus
    - 1296 KB embedded memory address space
    - 19704 KB external memory address space
    - 512 KB peripheral address space
    - Some embedded and external memory regions can be accessed by either
      data bus or instruction bus
    - 328 KB DMA address space
  * Embedded Memory
    - 448 KB Internal ROM
    - 520 KB Internal SRAM
    - 8 KB RTC FAST Memory
    - 8 KB RTC SLOW Memory
  * External Memory
    Off-chip SPI memory can be mapped into the available address space as
    external memory. Parts of the embedded memory can be used as transparent
    cache for this external memory.
    - Supports up to 16 MB off-Chip SPI Flash.
    - Supports up to 8 MB off-Chip SPI SRAM.
  * Peripherals
    - 41 peripherals
  * DMA
    - 13 modules are capable of DMA operation

ESP32 Toolchain
===============

  You must use the custom Xtensa toolchain in order to build the ESP32 Core
  BSP.  The steps to build toolchain with crosstool-NG on Linux are as
  follows:

    git clone -b xtensa-1.22.x https://github.com/espressif/crosstool-NG.git
    cd crosstool-NG
    ./bootstrap && ./configure --prefix=$PWD && make install
    ./ct-ng xtensa-esp32-elf
    ./ct-ng build
    chmod -R u+w builds/xtensa-esp32-elf

  These steps are given in setup guide in ESP-IDF repository:
  https://github.com/espressif/esp-idf/blob/master/docs/linux-setup.rst#alternative-step-1-compile-the-toolchain-from-source-using-crosstool-ng

  NOTE: The xtensa-esp32-elf configuration is only available in the
  xtensa-1.22.x branch.

Serial Console
==============

  USART0 is, by default, the serial console.  It connects to the on-board
  CP2102 converter and is available on the USB connector USB CON8 (J1).

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
  and USB activity.  None of these are available for use by sofware.

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each ESP32 core configuration is maintained in sub-directories and
  can be selected as follow:

    cd tools
    ./configure.sh esp32-core/<subdir>
    cd -
    make oldconfig
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and
  perform edits as necessary so that TOOLCHAIN_BIN is the correct path to
  the directory than holds your toolchain binaries.

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat esp32-core\<subdir>

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply,
  nuttx.

    make oldconfig
    make

  The <subdir> that is provided above as an argument to the
  tools/configure.sh must be is one of the directories listed below.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on [To be provided].

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:
