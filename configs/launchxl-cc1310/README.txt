README
======

  This directory holds NuttX board support for the TI LaunchXL-CC1310.  This
  board features the CC1310F128 part with 128Kb of FLASH and 20Kb of SRAM.

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons

Status
======

  2019-01-21:  Fragmentary board support in place.  The initial intent
    of this board support is simply to assist in the CC13x0 architecture
    development.  Serious board development will occur later. At present,
    the CC13x0 does not even compile error-free:  Compilation of cc14x0_rom.c
    fails because the DDI0 OSC header file has not yet been ported.

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

