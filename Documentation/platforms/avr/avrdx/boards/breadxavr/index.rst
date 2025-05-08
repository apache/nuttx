========================
AVR128DA28 on breadboard
========================

This is a AVR128DA28 stuck into a breadboard for the purpose
of developing this port for NuttX. It can also be used as a reference
for making ports for other boards.

Features
========

  - AVR128DA28

Configurations
==============

nsh
---

Basic testing configuration. When :menuselection:`Library Routines --> Support Builtin Applications`
and :menuselection:`Application Configuration --> NSH Library --> Enable built-in applications`
are both set, allows adding and execution of other applications.

Peripheral usage
================

Serial ports
------------

The board does not need any special configuration for serial port usage.
Everything is set in architecture configuration
in :menuselection:`System Type --> AVR DA/DB Peripheral Selections`

Buttons
-------

The board supports 4 buttons controlled by input button driver.
These are connected to I/O pins A2 (button 0), A3 (button 1), C2 (button 2) and C3 (button 3.)

To be able to use them, it is needed to:

  - enable :menuselection:`Board Selection --> Board button support`
  - enable :menuselection:`Button interrupt support` in the same section
  - enable :menuselection:`Device Drivers --> Input Device Support --> Button Inputs`
  - enable :menuselection:`Generic Lower Half Button Driver` in the same section
  - enable :menuselection:`System Type -->  GPIO ISR multiplexer`
  - enable :menuselection:`RTOS Features --> RTOS hooks --> Custom board early initialization`
  - enable :menuselection:`Board Selection --> Enable button input driver for GPIO pins`

Buttons can be used with the ``Buttons driver example`` example application.
Configure :menuselection:`Number of Buttons in the Board` to 4, default
:menuselection:`Button device path` ``/dev/buttons`` is correct.

The board code depends on GPIO ISR multiplexer - see
``Documentation/platforms/avr/avrdx/gpio_intr_mux.rst`` for details.

Note that the board configures the I/O pins reserved for buttons to invert
logical value, ie. logical zero applied to the pin is internally read
as logical one and vice-versa. This is to conform to button input driver's
expectation that requires pressed button to read as logical 1 and depressed
button as logical 0 while allowing the board to operate without external
components (internal pull-ups are used.)

Compile & Flash
===============

All code is tested with GCC for AVR. In Debian and operating systems derived
from it, ``gcc-avr`` package provides the compiler and ``avr-libc`` provides
related files. Note that older versions of GCC do not support these chips
out of the box
and some manual downloads are required. Other AVR compilers should work
as well but since these were not tested, not all features are enabled for them.

Default configuration produces ``nuttx.asm`` file with program disassembly
and ``nuttx.hex`` file in Intel HEX format to be used for upload to the chip.

Program can be uploaded for example by ``avrdude`` using UPDI (Unified Program
and Debug Interface.) As for programmer hardware, any serial port should
work. Serial port data pins need to be level-shifted to chip's supply voltage.
The pins are then connected through circuit like this one:

::

                              Vcc
                               |
                              100k
             schottky          |
  (PC) TX> -- |<|----------------------------- <RX (PC)
                                   \-- 470R -- <UPDI (MCU)

As long as TX is logical 1, there is no current between Vcc and TX pin
and both RX and UPDI pins read 1. When TX transitions to logical 0,
the schottky diode opens and overrides the 100k pull-up resistor,
both RX and UPDI read 0. When UPDI transitions to logical 0, it also
overrides the pull-up resistor but the schottky diode remains closed
and prevents TX pin from being affected. RX reads 0.
