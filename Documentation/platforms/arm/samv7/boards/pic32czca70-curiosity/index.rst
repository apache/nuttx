=====================================
PIC32CZ CA70 Curiosity Evaluation Kit
=====================================

The `PIC32CZ CA70 Curiosity <https://www.microchip.com/en-us/development-tool/EV56T44A>`_
is an evaluation kit for PIC32CZ CA70 series.

Features
========

- Processor
    - PIC32CZCA70144 processor
- Memory
    - 512 kB RAM memory
    - 2 MB Flash
    - external 4 MB NOR Flash

LEDs
====

There is one green and one red LED on PIC32CZ CA70 Evaluation board. The
LEDs can be activated by driving the connected I/O line to GND.

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined. In that case, the usage by the board port is defined in
include/board.h and src/sam_autoleds.c. The green LED0 is used to encode
OS-related events as follows:

================ ======================= =====
SYMBOL           Meaning                 LED
================ ======================= =====
LED_STARTED      NuttX has been started  OFF
LED_HEAPALLOCATE Heap has been allocated OFF
LED_IRQSENABLED  Interrupts enabled      OFF
LED_STACKCREATED Idle stack created      ON
LED_INIRQ        In an interrupt         N/C
LED_SIGNAL       In a signal handler     N/C
LED_ASSERTION    An assertion failed     N/C
LED_PANIC        The system has crashed  FLASH
================ ======================= =====

Thus if the LED is statically on, NuttX has successfully booted and is,
apparently, running normally. If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

Configurations
==============

nsh
---

Basic configuration with NuttShell (nsh) console over UART1 port. This
port is routed to debug USB, therefore the console is available
on the USB port.

max
---

This configuration intends to provide example support of available
peripherals. Currently only software buttons, USB device and micro
SD card are supported.

Flash
=====

There are two debug ports available on the board. 10 pin Cortex debug at
J300 header and 20 pin Cortex Trace Coreshight at J502 header. It is
possible to use SWD pins and connect ST Link or other programmer to the
microcontroller through these pins. The following command can be used
to program the board over ST Link with OpenOCD.

.. code-block:: console

    $ openocd -f boards/arm/samv7/common/tools/pic32cz-curiosity-stlink.cfg -c "program nuttx reset exit"

Note that PIC32CZ CA70 MCU default setting after erase is boot from ROM
instead of flash. Boot from flash can be setup from OpenOCD with a
following command.

.. code-block:: console

    mww 0x400e0c04 0x5a00010b

The board can be also flashed over USB with a proprietary Microchip's
`MPLAB X IDE <https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide>`_
