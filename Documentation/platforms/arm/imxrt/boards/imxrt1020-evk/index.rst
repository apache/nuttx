===============
i.MX RT1020 EVK
===============

`i.MX RT1020 EVK <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1020-evaluation-kit:MIMXRT1020-EVK>`_
is an evaluation kit by NXP company. This kit uses the i.MX RT1020 crossover MCU in LQFP144 package with ARM Cortex M7 core.

Features
========

- Processor
    - MIMXRT1021DAG5A processor
- Memory
    - 256 Mb SDRAM memory
    - 64 Mb QSPI Flash
    - TF socket for SD card
- Display and Audio
    - Audio CODEC
    - 4-pole audio headphone jack
    - External speaker connection
    - Microphone
- Connectivity
    - Micro USB host and OTG connectors
    - Ethernet (10/100T) connector
    - CAN transceivers
    - Arduino® interface

Serial Console
==============

The EVK default console is on LPUART1, which is multiplexed onto
the debug port (either OpenSDA or SEGGER JLink).

It runs at 115200,n,8,1.

LEDs and Buttons
================

LEDs
----

There is one user accessible LED status indicator located on the 1020-EVK,
USERLED.  The function of the LEDs include:

=== ============ ======
Pin Description  Colour
=== ============ ======
D3  Power Supply Green
D5  User LED     Green
D15 Reset LED    Red
=== ============ ======

This LED is not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/imxrt_autoleds.c. The LED is used to encode
OS-related events as documented in board.h

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

Thus if the LED is statically on, NuttX has successfully  booted and is,
apparently, running normally.  If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.


Buttons
-------

This IMXRT board has three external buttons

=== ==============  ========================
SW2 (IRQ88, ONOFF)  Not on a GPIO, No muxing
SW3 (IRQ88, POR)    Not on a GPIO, No muxing
SW4 (IRQ88, USER)   Wakeup, GPIO5-0
=== ==============  ========================

Configurations
==============

netnsh
------
    
This configuration is similar to the nsh configuration except that is
has networking enabled, both IPv4 and IPv6.  This NSH configuration is
focused on network-related testing.

nsh
---

Configures the NuttShell (nsh) located at examples/nsh.  This NSH
configuration is focused on low level, command-line driver testing.
Built-in applications are supported, but none are enabled.  This
configuration does not support a network.
