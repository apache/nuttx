===============
i.MX RT1050 EVK
===============

`i.MX RT1050 EVK <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1050-evaluation-kit:MIMXRT1050-EVK>`_
is an evaluation kit by NXP company. This kit uses the i.MX RT1050 crossover MCU with Arm Cortex M7 core.

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

- UART1_TXD   GPIO_AD_B0_12  LPUART1_TX
- UART1_RXD   GPIO_AD_B0_13  LPUART1_RX

Arduino RS-232 Shield:

- J22 D0 UART_RX/D0  GPIO_AD_B1_07  LPUART3_RX
- J22 D1 UART_TX/D1  GPIO_AD_B1_06  LPUART3_TX

LEDs and buttons
================

LEDs
----

There are four LED status indicators located on the EVK Board.  The
functions of these LEDs include:

- Main Power Supply(D3)
    - Green: DC 5V main supply is normal.
    - Red:   J2 input voltage is over 5.6V.
    - Off:   The board is not powered.
- Reset RED LED(D15)
- OpenSDA LED(D16)
- USER LED(D18)

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
are built separately.  It is recommends to use a special make command;
not just 'make' but make with the following two arguments:

.. code-block:: console

    $ make pass1 pass2

In the normal case (just 'make'), make will attempt to build both user-
and kernel-mode blobs more or less interleaved.  This actual works!
However, for me it is very confusing so I prefer the above make command:
Make the user-space binaries first (pass1), then make the kernel-space
binaries (pass2)

NOTES:

At the end of the build, there will be several files in the top-level
NuttX build directory:

PASS1:
  - nuttx_user.elf    - The pass1 user-space ELF file
  - nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
  - User.map          - Symbols in the user-space ELF file

PASS2:
  - nuttx             - The pass2 kernel-space ELF file
  - nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
  - System.map        - Symbols in the kernel-space ELF file

The J-Link programmer will except files in .hex, .mot, .srec, and .bin
formats.

Combining .hex files.  If you plan to use the .hex files with your
debugger or FLASH utility, then you may need to combine the two hex
files into a single .hex file.  Here is how you can do that.

The 'tail' of the nuttx.hex file should look something like this
(with my comments added beginning with #):

.. code-block:: console::

  $ tail nuttx.hex
  #xx xxxx 00 data records
  ...
  :10 C93C 00 000000000040184000C2010000000000 90
  :10 C94C 00 2400080000801B4000C01B4000001C40 5D
  :10 C95C 00 00401C4000000C4050BF0060FF000100 74
  #xx xxxx 05 Start Linear Address Record
  :04 0000 05 6000 02C1 D4
  #xx xxxx 01 End Of File record
  :00 0000 01 FF

Use an editor such as vi to remove the 05 and 01 records.

The 'head' of the nuttx_user.hex file should look something like
this (again with my comments added beginning with #):

.. code-block:: console::

  $ head nuttx_user.hex
  #xx xxxx 04 Extended Linear Address Record
  :02 0000 04 6020 7A
  #xx xxxx 00 data records
  :10 0000 00 8905206030002060F2622060FC622060 80
  :10 0010 00 0000242008002420080024205C012420 63
  :10 0020 00 140024203D0020603100206071052060 14
  ...

Nothing needs to be done here.  The nuttx_user.hex file should
be fine.

Combine the edited nuttx.hex and un-edited nuttx_user.hex
file to produce a single combined hex file:

.. code-block:: console::

  $ cat nuttx.hex nuttx_user.hex >combined.hex

Then use the combined.hex file with the to write the FLASH image.
If you do this a lot, you will probably want to invest a little time
to develop a tool to automate these steps.

STATUS:  This configuration was added on 8 June 2018 primarily to assure
that all of the components are in place to support the PROTECTED mode
build.  This configuration, however, has not been verified as of this
writing.


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
