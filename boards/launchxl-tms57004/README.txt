README
======

  This README provides some information about the port of NuttX to the TI
  Hercules TMS570LS04x/03x LaunchPad Evaluation Kit (LAUNCHXL-TMS57004)
  featuring the Hercules TMS570LS0432PZ chip.

Contents
========

  - Status
  - Toolchain
  - LEDs and Buttons
  - Serial Console
  - Debugging
  - Configurations

Status
======

  2015-12-29:
  The basic port to the TMS570 is complete.  After a few debug attempts,
  I think I may have damaged my board or at least put it into a state where
  I can no longer use it:  The CPU NERROR LED illuminates and can't I re-
  program the FLASH.

  I was never able to use Code Composer Studio or UniFlash with the board.
  But I was initially able to load FLASH and debug using a Segger J-Link
  connected to the board as described below.  But I think that some of
  my initial code loads put the TMS570 in bad state (or worse).  Now
  the NERROR LED is on.  When I attempt to program the FLASH, the J-Link
  software complains that the CPU is running too slowly and then times
  out trying to erase the FLASH.

  I have made several important code fixes since them (some of which might
  improve this situation).  But I have been unable to test them.  At this
  point I will have to give up on this port OR perhaps order a new card.

Toolchain
=========

  Build Platform
  --------------
  All of these configurations are set up to build with Cygwin under Windows
  (unless stated otherwise in the description of the configuration).

  Endian-ness Issues
  ------------------
  I started using the "GNU Tools for ARM Embedded Processors" that is
  maintained by ARM.

    https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

  However, that tool chain will not support the TMS570 big-endian mode.
  Certainly the -mbig-endian options will compiler for big-endian, but the
  final link fails because there is no big-endian version lib libgcc.

  There are patches available here if you want to build that toolchain
  from scratch:

    https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/+question/27995

  I now use a version of the NuttX buildroot toolchain that can be built like
  this:

    cd buildroot/
    cp configs/cortexr4-armeb-eabi-4.8.3-defconfig .config
    make oldconfig
    make

  You have to have several obscure packages installed on your Linux or Cygwin
  system to build the toolchain like this:  GMP, MPFR, MPC, and probably
  others.  See the buildroot/README.txt file for additional important information
  about building the toolchain.

  Reconfiguring
  -------------
  The build configuration selections can easily be reconfigured using 'make
  menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

LEDs and Buttons
================

  LEDs
  ----
  The launchpad has several LEDs:

    - LEd D1 (white) that connects to the USB +5V supply,
    - LED D10 (red) that connects to the TMS570's NERROR pin,
    - D5 (blue), D6 (blue), and D8 (blue) connect to the XDS100 FT2322,
    - D7 (blue) connects to the XSD100 CPLD, and
    - Two white, user LEDs labeled D12 that connects to the NHET08
      pin and D11 that connects to GIOA2.

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

Debugging
=========

  I used a Segger J-Link connected to the Launchpad via the JTAG connector.
  The following table shows how I connected the 14-pin JTAG connector on
  the Launchpad to the Segger 20-pin JTAG connector:

    --- ----------- ------ ------------- --- ---------- ------ -------
    J12  LAUCHPAD   J-LINK J-LINK        J12 LAUCHPAD   J-LINK J-LINK
    PIN  SIGNAL     PIN    SIGNAL        PIN SIGNAL     PIN    SIGNAL
    --- ----------- ------ ------------- --- ---------- ------ -------
    1   TMS         7      TMS           2   TRTSN      3      nTRST
    3   TDI         5      TDI           4   GND        2      GND
    5   PD (+3V3)   1      VTref         6   N/C        -      N/C
    7   TDO         13     TDO           8   JTAG_SEL** 4      GND
    9   RTCK        11     RTCK          10  GND        6      GND
    11  TCK         9      TCK           12  GND        8      GND
    13  EMU0*       -      N/C           14  EMU1*      -      N/C
    --- ----------- ------ ------------- --- ---------- ------ -------

    * Pulled high on board
    ** Needs to be grounded to select JTAG

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each LaunchXL-TMS50704 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh launchxl-tms57004/<subdir>

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

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

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh.
