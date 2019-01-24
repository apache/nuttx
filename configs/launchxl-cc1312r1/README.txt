README
======

  This directory holds NuttX board support for the TI LaunchXL-CC1312R1.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o Version 1 or 2?
  o Running from SRAM

Status
======

  2018-12-03:  Fragmentary board support in place.  The initial intent
    of this board support is simply to assist in the CC13xx architecture
    development.  Serious board development will occur later.  Board
    support is missing LED and button support.

Serial Console
==============

  The on-board XDS110 Debugger provide a USB virtual serial console using
  UART0 (PA0/U0RX and PA1/U0TX).

LEDs and Buttons
================

LEDs
----


Buttons
-------

Version 1 or 2?
===============

  Two versions of the CC1312R1 are supported selected by CONFIG_ARCH_CHIP_CC13XX_V1
  or CONFIG_ARCH_CHIP_CC13XX_V2.  How can you tell which one you have?
  Perhaps you can tell by the markings on the chip, but I do not have the
  secret decoder ring necessary to do that.

  What you can do is enable CONFIG_DEBUG_ASSERTIONS.  The firmware can
  determine which version you have by looking at register contents.  The
  firmware will assert if you select the wrong version.  If that occurs,
  switch to the other version and the assertion should go away.

Running from SRAM
=================

  The LaunchXL-CC1312R1 port supports execution from RAM.  Execution from
  SRAM as a "safe" way to bring up a new board port without concern about
  borking the board because of a bad FLASH image.

  if CONFIG_BOOT_RUNFROMFLASH=y is set in the configuration, then the code
  will build to run from FLASH.  Otherwise (presumably CONFIG_BOOT_RUNFROMSRAM=y)
  the code will build to run from SRAM.  This is determined by the Make.defs
  file in the scripts/ sub-directory.  Based on those configuration
  settings, either scripts/flash.ld or sram.ld will be selected as the
  linker script file to be used.
