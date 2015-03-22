configs/pic32mz-starterkit README
===============================

This README file discusses the port of NuttX to the Microchip PIC32MZ
Embedded Connectivity (EC) Starter Kit.

Contents
========

  Port Status
  Board Overview
  On Board Debug Support
  Creating Compatible NuttX HEX files
  Tool Issues
  Serial Console
  LEDs
  Configurations

Port Status
===========

  As of this writing (2015-03-01), the basic port is complete including
  minimal support for the NuttShell (NSH) over UART1.  No testing has yet
  been performed due to seemingly insurmountable debug problems:

  Thusfar, no one has been successful using NuttX with MPLABX.  All
  debug is being performed using a J-Link debugger via some custom
  interconnect boards.

  Patches were provided by Kristopher Tate on 2015-03-21 that support the
  serial console with the NuttShell, completing the basic bring-up.

Board Overview
==============

There are two configurations of the Microchip PIC32MZ Embedded Connectivity
(EC) Starter Kit:

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

Tool Issues
===========

  Pinquino Toolchain
  ------------------
  If you use the Pinguino toolchain, you will probably see this error:

  C:\pinguino-11\compilers\p32\bin\p32-ld.exe: target elf32-tradlittlemips not found

  This is due to linker differences in the toolchains.  The linker script
  at configs/pic32mz-starterkit has:

    OUTPUT_FORMAT("elf32-tradlittlemips")

  This error can be eliminated with the Pinguino toolchain by changing this to:

    OUTPUT_FORMAT("elf32-littlemips")

  Mentor Toolchain
  ----------------

    https://sourcery.mentor.com/GNUToolchain/release2934 tools.

  If you use this toolchain, you will need to add CROSSDEV=mips-sde-elf- to
  your Make.defs file.

  ICD3
  ----
  The onboard debugger is Slow and one is better off using an ICD3. By removing
  jumper JP2, I can disable the on-board OpenHCD debugger an enable the RJ11
  debug connector.  My ICD 3 does seems to work properly using this configuration,
  at least in the sense that it is recognized by both MPLABX IDE and IPE.

  Segger J-Link
  -------------
  If using a Jlink that only these versions work with PIC32:

    J-Link BASE / EDU V9 or later
    J-Link ULTRA+ / PRO V4 or later

  Oddly, you must use the G version in the command:

    JLinkGDBServer  -device PIC32MZ2048ECG144 -if 2-wire-JTAG-PIC32 -speed 12000

    Even though we have PIC32MZ2048ECM144 parts on our board. (JLinkGDBServer
    will except anything and just mess up your weekend)

Serial Console
==============

  MEB-II
  ------
  By default, the UART1 is configured for the pins used by the MEB-II
  board.  The UART1 signals are available at the MEB-II PICTail
  connector:

    --------------- --------- -------------- ------------
    PIC32MZ PIN     CONNECTOR MEB-II PIN     PICTAIL PIN
    FUNCTION        J1        NAME           J2
    --------------- --------- -------------- ------------
    RPA14/SCL1/RA14 124       SCL1/TOUCH_SCL 4
    RPA15/SDA1/RA15 126       SDA1/TOUCH_SDA 6
                              +3.3V          1,26
                              GND            28
    --------------- --------- -------------- ------------

  This pin selection is controlled by these definitions in
  pic32mz-starterkit/include/board.h:

    #define BOARD_U1RX_PPS  U1RXR_RPA14
    #define BOARD_U1TX_PPS  U1TX_RPA15R

  PIC32MX I/O Expansion Board with Adapter Board
  ------------------------------------------
  If the MEB-II UART configuration when used with the I/O Expansion board
  (with the adapter), then UART will be on J11 with Pin 35 being U1RX (into
  MZ) and Pin 37 being TU1X (out od MZ).

  Directly from the Adapter Board
  -------------------------------
  But you can get serial port directly from the PIC32MZ Embedded
  Connectivity (EC) Adapter Board (AC320006).  The Microchip
  adapter board brings out UART signals as follows:

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

  If using a AC320006 by itself, JP7 pin 2 and JP8 pin 2 is where you would
  connect a 3.3 Volt TTL serial interface.

  For a configuration using UART1 connect:
   TX to AC320006-JP7 pin 2 which is PIC32MZ pin 106 (RPC14) used as U1RX
   RX to AC320006-JP8 pin 2 which is PIC32MZ pin 31  (RPB3)) used as U1TX

  For a configuration using For UART3 connect:
   TX to AC320006-JP8 pin 2 which is PIC32MZ pin 31  (RPB3)) used as U3RX
   RX to AC320006-JP7 pin 2 which is PIC32MZ pin 106 (RPC14) used as U3TX

  If using a AC320006 plugged into a DM320002 then regardless of which UART,
  UART1 or UART3 is configured in software, the jumpers on the AC320006 are
  the same, just the signal directions and UART changes.

                                          UART1   UART3
  AC320006-JP7 connect pin 2 to pin 3.     U1RX   U3TX
  AC320006-JP8 connect pin 2 to pin 3.     U1TX   U3RX

  For the default configuration using UART1 the PIC32MZ pin 106 (RPC14)
  will be configured as U1RX and is tied to the AC320006's JP7 Pin 2.
  With the jumpers as listed above, once the AC320006 is plugged into
  the DM320002, the PIC32MZ U1RX will be connected to the DM320002's
  J11 pin 43. The DM320002's J11 pin 43 should then be connected to
  the TX of a 3.3 volt TTL serial converter such as a FTDI TTL232RG.
  For the FTDI TTL232RG TX is the orange wire.

  Likewise the PIC32MZ pin 31 (RPB3) will be configured as U1TX and
  is tied to the AC320006's JP8 Pin 2.  With the jumpers as listed above,
  once the AC320006 is plugged into the DM320002, the PIC32MZ' U1TX will
  be connected to the DM320002's J11 pin 41. The DM320002's J11 pin 41
  should then be connected to the RX signal of a 3.3 volt TTL serial
  converter. For the FTDI TTL232RG RX is the yellow wire.

  For the alternate configuration using UART3 the PIC32MZ pin 106 (RPC14)
  will be configured as U3TX and is tied to the AC320006's JP7 Pin 2.
  With the jumpers as listed above, once the AC320006 is plugged into
  the DM320002, the PIC32MZ U3TX will be connected to the DM320002's
  J11 pin 43. The DM320002's J11 pin 43 should then be connected to
  the RX of a 3.3 volt TTL serial converter such as a FTDI TTL232RG.
  For the FTDI TTL232RG TX is the yellow wire.

  Likewise the PIC32MZ pin 31 (RPB3) will be configured as U3RX and
  is tied to the AC320006's JP8 Pin 2.  With the jumpers as listed above,
  once the AC320006 is plugged into the DM320002, the PIC32MZ' U3RX will
  be connected to the DM320002's J11 pin 41. The DM320002's J11 pin 41
  should then be connected to the TX signal of a 3.3 volt TTL serial
  converter. For the FTDI TTL232RG RX is the orange wire.

  board.h Header File Changes
  ---------------------------
  The board configuration is currently set up to use the Serial console
  on the MEB-II board.  If you  want to use the adapter board directly,
  you willneed to change pic32mz-starterkit/include/board.h as follows:

    -#define BOARD_U1RX_PPS  U1RXR_RPA14
    -#define BOARD_U1TX_PPS  U1TX_RPA15R
    +#define BOARD_U1RX_PPS  U1RXR_RPC14
    +#define BOARD_U1TX_PPS  U1TX_RPB3R

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
       This configuration has UART1 enabled as a serial console.  This
       can easily be changed by reconfiguring with 'make menuconfig'.

    3. Toolchain

       By default, the Pinguino MIPs tool chain is used.  This toolchain
       selection can easily be changed with 'make menuconfig'.

    4. Default configuration:  These are other things that you may want to
       change in the configuration:

       CONFIG_ARCH_CHIP_PIC32MZ2048ECM=y : Assumes part with Crypto Engine
       CONFIG_PIC32MZ_DEBUGGER_ENABLE=n  : Debugger is disabled
       CONFIG_PIC32MZ_TRACE_ENABLE=n     : Trace is disabled
       CONFIG_PIC32MZ_JTAG_ENABLE=n      : JTAG is disabled
