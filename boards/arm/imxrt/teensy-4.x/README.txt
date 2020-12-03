README
======

  This README file provides information about the port of NuttX to the Teensy 4.x board.  This board features the
  MIMXRT1062DVL6A MCU.  Some of the features of this board include:

    o Processor

      - MIMXRT1062DVL6A processor

    o Memory

      - 1 Mb RAM memory
      - 2 Mb QSPI Flash
      - 1 SDIO (4 bit) native SD

    o Connectivity

      - Micro USB host
      - CAN transceivers
      - 41 digital pins
      - 14 analog pins, 2 ADCs on chip

Contents
========

  o LEDs
  o Configurations
    - Configuration sub-directories

LEDs and buttons
================

  LEDs
  ----

  There are two LED status indicators located on the Teensy-4.x board.
  The functions of these LEDs include:

    - RED LED (loading status)
       - dim:    ready
       - bright: writing
       - blink:  no USB
    - USER LED (D3)

  Only a single LED, D3, is under software control.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/imxrt_autoleds.c. The LED is used to encode
  OS-related events as follows:

    ------------------- ----------------------- ------
    SYMBOL              Meaning                 LED
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus if the LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If the LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each Teensy-4.x configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] teensy-4.x:<subdir>

  Where typical options are -l to configure to build on Linux or -c to
  configure for Cygwin under Linux.  'tools/configure.sh -h' will show
  you all of the options.

  Before building, make sure the PATH environment variable include the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  nsh-4.0:

    Configures the NuttShell (nsh) located at examples/nsh for Teensy 4.0.  
    This NSH configuration is focused on low level, command-line driver testing.
    Built-in applications are supported, but none are enabled. NutShells then
    runs on UART6: Pin 0 is RX and pin 1 is TX

  nsh-4.1:

    Configures the NuttShell (nsh) located at examples/nsh for Teensy 4.1.  
    This NSH configuration is focused on low level, command-line driver testing.
    Built-in applications are supported, but none are enabled. NutShells then
    runs on UART1: Pin 24 is TX and pin 25 is RX

  can-4.1:

    This is an nsh configuration (see above) for Teensy-4.x with added support of 
    CAN driver. FlexCAN3 is chosen as default, the change can be made at System 
    type peripheral selection.

    Bitrate and sample point can be also changed at System type peripheral selection,
    basic values are 1 MHz for bitrate and 0.80 for sample point. The FlexCAN driver
    for imxrt runs at 80 MHz clock frequency.

    The configuration also includes CAN utilities as candump and cansend.

    CAN_FD supported but not enabled. For CAN_FD please select following:

    CAN_FD = y
    NET_CAN_CANFD = y
    NET_CAN_SOCK_OPTS = y

    This configuration can be easily changed to work with Teensy 4.0 by
    selecting CONFIG_TEENSY_40=y.

  netnsh-4.1:

    This configuration is similar to the nsh configuration except that is
    has networking enabled, both IPv4 and IPv6.  This NSH configuration is
    focused on network-related testing.

    This configuration cannot be changed to Teensy 4.0 as this board does
    not have Ethernet capability.

  sd-4.1

    This is an nsh configuration (see above) for Teensy-4.x with added support of 
    connecting micro SD card.

    You can mount micro SD card by "mount -t vfat /dev/mmcsd0 /mnt"

    This configuration cannot be changed to Teensy 4.0 as this board does
    not have micro SD card slot.
