===============
i.MX RT1050 EVK
===============

`i.MX RT1050 EVK <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1050-evaluation-kit:MIMXRT1050-EVK>`_
is an evaluation kit by NXP company. This kit uses the i.MX RT1050 crossover MCU with ARM Cortex M7 core.

Features
========

- Processor
    - MIMXRT1052DVL6A processor
- Memory
    - 256 Mb SDRAM memory
    - 512 Mb Hyper Flash
    - Footprint for QSPI Flash
    - TF socket for SD card
- Display and Audio
    - Parallel LCD connector
    - Camera connector
    - Audio CODEC
    - 4-pole audio headphone jack
    - External speaker connection
    - Microphone
    - SPDIF connector
- Connectivity
    - Micro USB host and OTG connectors
    - Ethernet (10/100T) connector
    - CAN transceivers
    - Arduino® interface

Serial Console
==============

Virtual console port provided by OpenSDA:

========= ============= ==========
UART1_TXD GPIO_AD_B0_12 LPUART1_TX
UART1_RXD GPIO_AD_B0_13 LPUART1_RX
========= ============= ==========

Arduino RS-232 Shield:

=== == ======= ============= ==========
J22 D0 UART_RX GPIO_AD_B1_07 LPUART3_RX
J22 D1 UART_TX GPIO_AD_B1_06 LPUART3_TX
=== == ======= ============= ==========

LEDs and buttons
================

LEDs
----

There are four LED status indicators located on the EVK Board.  The
functions of these LEDs include:

=== ============
Pin Description
=== ============
D3  Power Supply
D15 Reset LED
D16 OpenSDA
D18 User LED
=== ============

Only a single LED, D18, is under software control.  It connects to
GPIO_AD_B0_09 which is shared with JTAG_TDI and ENET_RST

This LED is not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/imxrt_autoleds.c. The LED is used to encode
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

Thus if the LED is statically on, NuttX has successfully  booted and is,
apparently, running normally.  If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

Buttons
-------

There are four user interface switches on the MIMXRT1050 EVK Board:

  - SW1: Power Switch (slide switch)
  - SW2: ON/OFF Button
  - SW3: Reset button
  - SW8: User button

Only the user button is available to the software.  It is sensed on the
WAKEUP pin which will be pulled low when the button is pressed.

Configurations
==============

knsh
----

This is identical to the nsh configuration below except that NuttX
is built as a protected mode, monolithic module and the user applications
are built separately. For further information about compiling and
running this configuration please refer to imxrt1064-evk documentation.

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
