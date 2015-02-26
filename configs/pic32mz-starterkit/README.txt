configs/pic32mz-starterkit README
===============================

This README file discusses the port of NuttX to the Microchip PIC32MZ
Embedded Connectivity (EC) Starter Kit.  There are two configurations of the
starter kit:

  1) The PIC32MZ Embedded Connectivity Starter Kit based on the
     PIC32MZ2048ECH144-I/PH chip (DM320006), and
  2) The PIC32MZ Embedded Connectivity Starter Kit based on the
     PIC32MZ2048ECM144-I/PH w/Crypto Engine (DM320006-C)

See www.microchip.com for further information.

Key features of the PIC32MZ Starter Kit include;

  * On-board crystal or oscillator for precision microcontroller clocking
    (24 MHz).
  * 32 kHz oscillator for RTCC and Timer1 (optional).
  * Three push button switches for user-defined inputs.
  * Three user-defined indicator LEDs.
  * USB Type A receptacle connectivity for PIC32 host-based applications.
  * USB Type micro-AB receptacle for OTG and USB device connectivity for
    PIC32 OTG/device-based applications.
  * Daughter board connectors for flexible Ethernet PHY options.
  * 50 MHz Ethernet PHY oscillator.
  * External 4 GB SQI memory for expanded memory applications.
  * PIC24FJ256GB106 USB microcontroller for on-board debugging.
  * USB connectivity for on-board debugger communications.
  * Regulated +3.3V power supply for powering the starter kit through USB or
    expansion board.
  * Connector for various expansion boards.

The PIC32MZ starter kit comes complete with a LAN8740 PHY daughter board.

Testing was performed with the following additional hardware:

- Microchip PIC32MZ Embedded Connectivity (EC) Adapter Board (AC320006)
  that allows connection of the PIC32MZEC Starter Kit to the Microchip
  Multimedia Expansion Board (MEB, DM320005) or PIC32 I/O Expansion Board
  (DM320002).  These were previously used with the PIC32MX bringup.
- Microchip Multimedia Expansion Board II (MEB II,  DM320005-2).

Contents
========

  On Board Debug Support
  Creating Compatible NuttX HEX files
  Serial Console
  LEDs
  Configurations

On Board Debug Support
======================

  The starter kit includes a PIC24FJ256GB106 USB microcontroller that
  provides debugger connectivity over USB. The PIC24FJ256GB106 is hard-wired
  to the PIC32 device to provide protocol translation through the I/O pins
  of the PIC24FJ256GB106 to the ICSP™ pins of the PIC32 device.

  If MPLAB® REAL ICE™ or MPLAB ICD 3 is used with the starter kit,
  disconnect the onboard debugger from the PIC32 device by removing the
  jumper JP2. When the on-board debugger is required, replace the jumper
  JP2. When the jumper JP2 is installed, pin 1 must be connected to pin 3
  and pin 2 must be connected to pin 4.

Creating Compatible NuttX HEX files
===================================

  Intel Hex Format Files:
  -----------------------

    When NuttX is built it will produce two files in the top-level NuttX
    directory:

    1) nuttx - This is an ELF file, and
    2) nuttx.hex - This is an Intel Hex format file.  This is controlled by
       the setting CONFIG_INTELHEX_BINARY in the .config file.

    The PICkit tool wants an Intel Hex format file to burn into FLASH. However,
    there is a problem with the generated nutt.hex: The tool expects the nuttx.hex
    file to contain physical addresses.  But the nuttx.hex file generated from the
    top-level make will have address in the KSEG0 and KSEG1 regions.

  tools/pic32mx/mkpichex:
  ----------------------

    There is a simple tool in the NuttX tools/pic32mx directory that can be
    used to solve both issues with the nuttx.hex file.  But, first, you must
    build the tool:

      cd tools/pic32mx
      make

    Now you will have an excecutable file call mkpichex (or mkpichex.exe on
    Cygwin).  This program will take the nutt.hex file as an input, it will
    convert all of the KSEG0 and KSEG1 addresses to physical address, and
    it will write the modified file, replacing the original nuttx.hex.

    To use this file, you need to do the following things:

      . ./setenv.sh    # Source setenv.sh.  Among other this, this script
                       # will add the NuttX tools/pic32mx directory to your
                       # PATH variable
      make             # Build nuttx and nuttx.hex
      mkpichex $PWD    #  Convert addresses in nuttx.hex.  $PWD is the path
                       # to the top-level build directory.  It is the only
                       # required input to mkpichex.

Serial Console
==============

  The Microchip PIC32MZ Embedded Connectivity (EC) Adapter Board (AC320006)
  brings out UART signals as follows:

  JP7 redirects J1 U3_TX to either J2 SOSCO/RC14 or U1_TX:

    Adapter
    -----------------------------------------------------------------------
    JP7, Pin 1: J2 Pin 32, SOSCO/RC14
         Pin 2: J1 Pin 17, U3_TX
         Pin 3: J2 Pin 90, U1_TX

    PIC32MZ Starter Kit
    -----------------------------------------------------------------------
                J1 Pin 17, SOSCO/RC14  PIC32MZ SOSCO/RPC14/T1CK/RC14

    RPC14 supports U1RX, U4RX, and U3TX

  JP8 redirects J1 RB3/AN3/SDO4/WIFI_SDI to either J2 AN3/SDO4/WIFI_SDI or U3_RX:

    Adapter                                        PIC32MZ Starter Kit
    ---------------------------------------------- -------------------------
    JP8, Pin 1: J2, Pin 66,  AN3/SDO4/WIFI_SDI
         Pin 2: J1, Pin 105, RB3/AN3/SDO4/WIFI_SDI
         Pin 3: J2, Pin 88,  U3_RX

    PIC32MZ Starter Kit
    -----------------------------------------------------------------------
                J1, Pin 105, AN3/C2INA/RPB3/RB3

    RPB3 supports U3RX, U1TX, and U5TX

  Thus UART1 or UART3 could be used as a serial console if only the
  PIC32MZEC Adapter Board is connected.

  The default serial configuration here in these configurations is UART1
  using RPC14 and RPB3.  That UART selection can be change by running 'make
  menuconfig'.  The UART pin selections would need to be changed by editing
  configs/pc32mz-starterkit/include/board.h.

LEDs and Buttons
================

  LEDs
  ----
  The PIC32MZ Ethernet Starter kit has 3 user LEDs labelled LED1-3 on the
  board:

  PIN  LED   Notes
  ---  ----- -------------------------
  RH0  LED1  High illuminates (RED)
  RH1  LED3  High illuminates (YELLOW)
  RH2  LED2  High illuminates (GREEN)

  If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as
  follows:

                            ON                  OFF
  ------------------------- ---- ---- ---- ---- ---- ----
                            LED1 LED2 LED3 LED1 LED2 LED3
  ------------------------- ---- ---- ---- ---- ---- ----
  LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
  LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
  LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
  LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
  LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
  LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
  LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
  LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C

  Buttons
  -------

  The PIC32MZ Ethernet Starter kit has 3 user push buttons labelled SW1-3 on
  the board:

  PIN   LED  Notes
  ----  ---- -------------------------
  RB12  SW1  Active-low
  RB13  SW2  Active-low
  RB14  SW3  Active-low

  The switches do not have any debounce circuitry and require internal pull-
  up resistors. When Idle, the switches are pulled high (+3.3V), and they
  are grounded when pressed.

Configurations
==============

Each PIC32MZ configuration is maintained in a sub-directory and can be
selected as follow:

    cd tools
    ./configure.sh pic32mz-starterkit/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:

    This is the NuttShell (NSH) using the NSH startup logic at
    apps/examples/nsh.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Serial Output

       The OS test produces all of its test output on the serial console.
       This configuration has UART1 enabled as a serial console.

    3. Toolchain

       By default, the Pinguino MIPs tool chain is used.  This toolchain
       selection can easily be changed with 'make menuconfig'.
