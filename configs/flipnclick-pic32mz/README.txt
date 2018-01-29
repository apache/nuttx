configs/flipnclick-pic32mz README
===============================

  This README file discusses the port of NuttX to the Mikroe Flip&Click
  PIC32MZ board.  That board features the PIC32MZ2048EFH100 MCU.

  Thanks to John Legg for contributing the Flip&Click PIC32MZ board!

Contents
========

  Port Status
  On Board Debug Support
  Creating Compatible NuttX HEX files
  Tool Issues
  Serial Console
  SPI
  LEDs
  Configurations

Port Status
===========

  2018-01-07:  Added architecture support for the PIC32MZ2048EFH100 used on
    the Flip&Click PIC32MZ board.
  2018-01-08:  Created the basic board configuration for the Mikroe
    Flip&Click PIC32MZ board.  No testing has yet been performed.  At this
    point, I have not even figured out how I am going to load and debug
    new firmware.  I need understand how the memory map is set up when used
    with the mikroBootloader.

On Board Debug Support
======================

  There are several debug options:

  1. Using the Arduino IDE (chipKIT core).  This is available on the USB-UART
     port between the C and D MikroBUS sockets.  Usage is described in the
     Flip&Click User Manual.

     I don't think trying to use the Arduino IDE is a good option.

  2. Using the mikroC USB HID bootloader.  This is is available on the USB
     port between the A and B MikroBUS sockets.  Usage is described in the
     Flip&Click User Manual.

     There is a simple application available at Mikroe that will allow you
     to write .hex files via the USB HID bootloader.  However, in order to
     use the bootloader, you will have to control the memory map so that the
     downloaded code does not clobber the bootloader code FLASH, data
     memory, exception vectors, etc.

     At this point, I have found no documentation describing how to build
     the code outside of the Mikroe toolchain for use with the Mikroe
     bootloader.

  3. There is an undocumented and unpopulated PICKit3 connector between the
     B and C mikroBUS sockets.

  4. There is an undocumented and unpopulated mikroProg connector between
     the A and D mikroBUS sockets.

     Since 3) and 4) are undocumented, this would require some research and
     would, most likely, clobber the USB HID bootloader (and possibly the
     Arduino support as well).

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

      export PATH=???  # Add the NuttX tools/pic32mx directory to your
                       # PATH variable
      make             # Build nuttx and nuttx.hex
      mkpichex $PWD    #  Convert addresses in nuttx.hex.  $PWD is the path
                       # to the top-level build directory.  It is the only
                       # required input to mkpichex.

Tool Issues
===========

  Segger J-Link
  -------------
  If using a Jlink that only these versions work with PIC32:

    J-Link BASE / EDU V9 or later
    J-Link ULTRA+ / PRO V4 or later

  This is the command to use:

    JLinkGDBServer -device PIC32MZ2048EFH100 -if 2-wire-JTAG-PIC32 -speed 12000

Serial Console
==============

  [REVISIT:  I am not sure if the USB VCOM ports are available to the
   software.  That is likely another serial port option].

  Convenient U[S]ARTs that may be used as the Serial console include:

  1) An Arduino Serial Shield.  The RX and TX pins are available on the
     Arduino connector D0 and D1 pins, respectively.  These are connected
     to UART5, UART5_RX and UART5_TX which are RD14 and RD15, respectively.

  2) Mikroe Click Serial Shield.  There are four Click bus connectors with
     serial ports available as follows:

     Click A:  UART4 UART4_RX and UART4_TX which are RG9 and RE3, respectively.
     Click B:  UART3 UART3_RX and UART3_TX which are RF0 and RF1, respectively.
     Click C:  UART1 UART1_RX and UART1_TX which are RC1 and RE5, respectively.
     Click D:  UART2 UART2_RX and UART2_TX which are RC3 and RC2, respectively.

  Other serial ports are probably available on the Arduino connector.  I
  will leave that as an exercise for the interested reader.

  The outputs from these pins is 3.3V.  You will need to connect RS232
  transceiver to get the signals to RS232 levels (or connect to the
  USB virtual COM port in the case of UART0).

SPI
===

   SPI3 is available on pins D10-D13 of the Arduino Shield connectors where
   you would expect then.  The SPI connector is configured as follows:

     Pin J1 Board Signal PIC32MZ
     --- -- ------------ -------
     D10 8  SPI3_SCK     RB14
     D10 7  SPI3_MISO    RB9
     D11 6  SPI3_MOSI    RB10
     D13 5  SPI3_SS      RB9

   SPI1 and SPI2 are also available on the mikroBUS Click connectors (in
   addition to 5V and GND).  The connectivity between connectors A and B and
   between C and D differs only in the chip select pin:

     MikroBUS A:                 MikroBUS B:
     Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
     ---- ------------ -------  ---- ------------ -------
     CS   SPI2_SS1     RA0      CS   SPI2_SS0     RE4
     SCK  SPI2_SCK     RG6      SCK  SPI2_SCK     RG6
     MISO SPI2_MISO    RC4      MISO SPI2_MISO    RC4
     MOSI SPI2_MOSI    RB5      MOSI SPI2_MOSI    RB5

     MikroBUS C:                 MikroBUS D:
     Pin  Board Signal PIC32MZ  Pin  Board Signal PIC32MZ
     ---- ------------ -------  ---- ------------ -------
     CS   SPI1_SS0     RD12     CS   SPI1_SS1     RD13
     SCK  SPI1_SCK     RD1      SCK  SPI1_SCK     RD1
     MISO SPI1_MISO    RD2      MISO SPI1_MISO    RD2
     MOSI SPI1_MOSI    RD3      MOSI SPI1_MOSI    RD3

LEDs and Buttons
================

  LEDs
  ----

  There are four LEDs on the top, red side of the board.  Only
  one can be controlled by software:

    LED L - RB14 (SPI3_SCK)

  There are also four LEDs on the back, white side of the board:

    LED A - RA6
    LED B - RA7
    LED C - RE0
    LED D - RE1

  A high output value illuminates the LEDs.

  These LEDs are available to the application and are all available to the
  application unless CONFIG_ARCH_LEDS is defined.  In that case, the usage
  by the board port is defined in include/board.h and src/sam_autoleds.c.
  The LEDs are used to encode OS-related events as follows:

    SYMBOL           MEANING                        LED STATE
                                              L   A   B   C   D
    ---------------- ----------------------- --- --- --- --- ---
    LED_STARTED      NuttX has been started  OFF ON  OFF OFF OFF
    LED_HEAPALLOCATE Heap has been allocated OFF OFF ON  OFF OFF
    LED_IRQSENABLED  Interrupts enabled      OFF OFF OFF ON  OFF
    LED_STACKCREATED Idle stack created      OFF OFF OFF OFF ON
    LED_INIRQ        In an interrupt         GLO N/C N/C N/C N/C
    LED_SIGNAL       In a signal handler     GLO N/C N/C N/C N/C
    LED_ASSERTION    An assertion failed     GLO N/C N/C N/C N/C
    LED_PANIC        The system has crashed  2Hz N/C N/C N/C N/C
    LED_IDLE         MCU is is sleep mode    ---- Not used -----

  Thus if LED L is glowing faintly and all other LEDs are off (except LED D
  which was left on but is no longer controlled by NuttX and so may be in any
  state), NuttX has successfully booted and is, apparently, running normally
  and taking interrupts.  If any of LEDs A-D are statically set, then NuttX
  failed to boot and the LED indicates the initialization phase where the
  failure occurred.  If LED L is flashing at approximately 2Hz, then a fatal
  error has been detected and the system has halted.

  NOTE: After booting, LEDs A-D are no longer used by the system and may
  be controlled the application.

  Buttons
  -------

  The Flip&Click PIC32MZ has 2 user push buttons labeled T1 and T2 on the
  white side of the board:

  PIN   LED  Notes
  ----- ---- -------------------------
  RD10  T1   Sensed low when closed
  RD11  T2   Sensed low when closed

  The switches have external pull-up resistors. The switches are pulled high
  (+3.3V) and grounded when pressed.

Configurations
==============

Information Common to All Configurations
----------------------------------------

  1. Each PIC32MZ configuration is maintained in a sub-directory and can be
     selected as follow:

       tools/configure.sh flipnclick-pic32mz/<subdir>

     Where typical options are -l to configure to build on Linux or -c to
     configure for Cygwin under Linux.  'tools/configure.sh -h' will show
     you all of the options.

     Before building, make sure the PATH environment variable includes the
     correct path to the directory than holds your toolchain binaries.

     And then build NuttX by simply typing the following.  At the conclusion
     of the make, the nuttx binary will reside in an ELF file called, simply,
     nuttx.

        make

     The <subdir> that is provided above as an argument to the
     tools/configure.sh must be is one of the directories listed in the
     following paragraph.

  2. These configurations uses the mconf-based configuration tool.  To
     change this configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in the top-level nuttx in order to start
        the reconfiguration process.

Configuration Directories
-------------------------

Where <subdir> is one of the following:

  nsh:

    This is the NuttShell (NSH) using the NSH startup logic at
    apps/examples/nsh.

    NOTES:

    1. Serial Console.  UART 4 is configured as the Serial Console.  This
       assumes that you will be using a Mikroe RS-232 Click card in the
       mikroBUS A slot.  Other serial consoles may be selected by re-
       configuring (see the section "Serial Consoles" above).

    2. Toolchain

       By default, the Pinguino MIPs tool chain is used.  This toolchain
       selection can easily be changed with 'make menuconfig'.

    3. Default configuration:  These are other things that you may want to
       change in the configuration:

       CONFIG_PIC32MZ_DEBUGGER_ENABLE=n  : Debugger is disabled
       CONFIG_PIC32MZ_TRACE_ENABLE=n     : Trace is disabled
       CONFIG_PIC32MZ_JTAG_ENABLE=n      : JTAG is disabled
