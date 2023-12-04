===============
i.MX RT1170 EVK
===============

`i.MX RT1170 EVK <https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/mimxRT1170-evk-i-mx-RT1170-evaluation-kit:MIMXRT1170-EVK>`_
is an evaluation kit by NXP company. This kit uses the i.MX RT1170 crossover MCU with ARM Cortex M7 core.

Features
========

- Processor
    - MIMXRT1176DVMAA processor
    - 1GHz Cortex-M7 
    - 400Mhz Cortex-M4
- Memory
    - 2 Mb OCRAM memory
    - 512 Mbit SDRAM memory
    - 512 Mbit Hyper Flash - Populated but 0 ohm DNP
    - 64 Mbit QSPI Flash
    - TF socket for SD card
- Display and Audio
    - MIPI LCD connectors
- Connectivity
    - Micro USB host and OTG connectors
    - Ethernet (10/100T) connector
    - Ethernet (10/100/1000T) connector
    - CAN transceivers
    - Arduino® interface
- Sensors
    - FXOS8700CQ 6-Axis Ecompass (3-Axis Mag, 3-Axis Accel)

Serial Console
==============

Virtual console port provided by OpenSDA:

========= ========== ==========
UART1_TXD GPIO_AD_24 LPUART1_TX
UART1_RXD GPIO_AD_25 LPUART1_RX
========= ========== ==========

Arduino RS-232 Shield:

=== == ======= =============== ==========
J22 D0 UART_RX GPIO_DISP_B2_11 LPUART2_RX
J22 D1 UART_TX GPIO_DISP_B2_10 LPUART2_TX
=== == ======= =============== ==========

J-Link External Debug Probe
===========================

Install the J-Link Debug Host Tools and make sure they are in your search path.

Attach a J-Link 20-pin connector to J1. Check that jumpers J5, J6, J7 and J8 are
off (they are on by default when boards ship from the factory) to ensure SWD
signals are disconnected from the OpenSDA microcontroller.

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

.. code-block:: console

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

.. code-block:: console

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

.. code-block:: console

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