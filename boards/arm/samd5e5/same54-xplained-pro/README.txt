README
======

  This directory contains the port of NuttX to the Microchip SAME54 Xplained
  Pro board.  This board is powered by an ATSAME54P20A:

  o Cortex M4 core running at 120 MHz
  o Hardware DSP and floating point support
  o 1 MB flash, 256 KB RAM
  o 32-bit, 3.3V logic and power
  o Dual 1 MSPS DAC (A0 and A1)
  o Dual 1 MSPS ADC (8 analog pins)
  o 8 x hardware SERCOM (I2C, SPI or UART)
  o 16 x PWM outputs
  o Stereo I2S input/output with MCK pin
  o 14-bit Parallel capture controller (for camera/video in)
  o Built in crypto engines with AES (256 bit), true RNG, Pubkey controller
  o 10/100 Ethernet MAC
  o Dual SD/MMC controller
  o Dual CAN bus interfaces
  o 100-TQFP

Contents
========

  o STATUS
  o Serial Console
  o LEDs
  o Run from SRAM
  o Configurations

STATUS
======

  2019-09-17:  Board port started based on Metro M4 board.

  WARNING:  If you decide to invest the time to discover whey the XOSC32K
  clock source is not working, be certain to use the SRAM configuration.
  That configuration in FLASH is most likely lock up your board irrecoverably
  is there are any start-up errors!


Serial Console
==============

  The onboard debugger on the SAME54 Xplained Pro provides a virtual serial
  interface over the DEBUG USB port.  The pins on the SAME54 are as follows:

    ----------------- -----------
    SAMD5E5           FUNCTION
    ----------------- -----------
    PB24 SERCOM2 PAD1 RXD
    PB25 SERCOM2 PAD0 TXD


  An external RS-232 or serial-to-USB adapter can be connected on pins PA22
  and PA23:

    ----------------- ---------
    SAMD5E5           FUNCTION
    ----------------- ---------
    PA23 SERCOM3 PAD1 RXD
    PA22 SERCOM3 PAD0 TXD


LEDs
====

  The SAME54 Xplained Pro has three LEDs, but only one is controllable by software:

    1. LED0 near the edge of the board


    ----------------- -----------
    SAMD5E5           FUNCTION
    ----------------- -----------
    PC18              GPIO output

Run from SRAM
=============

  I bricked my first Metro M4 board because there were problems in the
  bring-up logic.  These problems left the chip in a bad state that was
  repeated on each reset because the code was written into FLASH and I was
  unable to ever connect to it again via SWD.

  To make the bring-up less risky, I added a configuration option to build
  the code to execution entirely out of SRAM.  By default, the setting
  CONFIG_SAME54_XPLAINED_PRO_RUNFROMFLASH=y is used and the code is built to run out of
  FLASH.  If CONFIG_SAME54_XPLAINED_PRO_RUNFROMSRAM=y is selected instead, then the
  code is built to run out of SRAM.

  To use the code in this configuration, the program must be started a
  little differently:

    gdb> mon reset
    gdb> mon halt
    gdb> load nuttx             << Load NuttX into SRAM
    gdb> file nuttx             << Assuming debug symbols are enabled
    gdb> mon memu32 0x20000000  << Get the address of initial stack
    gdb> mon reg sp 0x200161c4  << Set the initial stack pointer using this address
    gdb> mon memu32 0x20000004  << Get the address of __start entry point
    gdb> mon reg pc 0x20000264  << Set the PC using this address (without bit 0 set)
    gdb> si                     << Step in just to make sure everything is okay
    gdb> [ set breakpoints ]
    gdb> c                      << Then continue until you hit a breakpoint

  Where 0x200161c4 and 0x20000264 are the values of the initial stack and
  the __start entry point that I read from SRAM

Configurations
==============

  Each SAME54 Xplained Pro configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] same54-xplained-pro:<subdir>

  Do 'tools/configure.sh -h' for the list of options.  If you are building
  under Windows with Cygwin, you would need the -c option, for example.

  Before building, make sure that the PATH environmental variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of configurations listed in the following paragraph.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on SERCOM2 which is available via USB debug.

  3. Unless otherwise stated, the configurations are setup build under
     Linux with a generic ARM EABI toolchain:

Configuration sub-directories
-----------------------------

  nsh:
    This configuration directory will built the NuttShell.  See NOTES for
    common configuration above and the following:

    NOTES:

    1. The CMCC (Cortex M Cache Controller) is enabled.
