README
======

  This README provides some information about the port of NuttX to the TI
  Hercules TMS570LS04x/03x LaunchPad Evaluation Kit (LAUNCHXL-TMS57004)
  featuring the Hercules TMS570LS0432PZ chip.

Contents
========

  - Toolchain
  - LEDs and Buttons
  - Serial Console
  - Debugging
  - Configurations

Toolchain
=========

  All of these configurations are set up to build with Cygwin under Windows
  using the "GNU Tools for ARM Embedded Processors" that is maintained by ARM
  (unless stated otherwise in the description of the configuration).

    https://launchpad.net/gcc-arm-embedded

  That toolchain selection can easily be reconfigured using 'make menuconfig'.
  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

LEDs and Buttons
================

  LEDs
  ----
  The launchpad has a four LEDs two power LEDs labeled D1 (red) that
  connects to the TMS570's NERROR pin and D7 (blue) that indicates the
  XDS200 POWER_EN signal, and two white, user LEDs labeled D12 that
  connects to the NHET08 pin and D11 that connects to GIOA2.

  NHET08 is one of 32 N2HET pins than can be available to the user if
  not used by N2HET.  This implementation, however, uses only the single
  LED driven by GIOA2.  That LED is tied to ground and illuminated
  with a high level output value.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/tms570_autoleds.c. The LED is used to encode
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

  Buttons
  -------
  The launchpad has three mechanical buttons. Two of these are reset buttons:
  One button is labeled PORRST performs a power-on reset and one labeled RST
  performs an MCU reset.  Only one button is available for general software
  usage.  That button is labeled GIOA7 and is, obviously, sensed on GIOA7.

  GIOA7 is tied to ground, but will be pulled high if the GIOA7 button is
  depressed.

Serial Console
==============

  This TMS570 has a single SCI.  The SCI_RX and TX pins are connected to
  the FTDI chip which provides a virtual COM port for the launchpad.

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each LaunchXL-TMS50704 configuration is maintained in a sub-directory and
can be selected as follow:

  cd tools
  ./configure.sh launchxl-tms57004/<subdir>
  cd -
  . ./setenv.sh

Before sourcing the setenv.sh file above, you should examine it and perform
edits as necessary so that TOOLCHAIN_BIN is the correct path to the directory
than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make oldconfig
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

       https://launchpad.net/gcc-arm-embedded

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.
