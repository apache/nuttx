=================
Microchip pic32mz
=================

Microchip PIC32MZEF
-------------------

(MIPS M5150 architecture).

A port is in available for the MikroElectronika `Flip&Click
PIC32MZ <https://www.mikroe.com/flipclick-pic32mz>`__ development board
based on the PIC32MZ2048EFH100 MCU. This board configuration was added
in NuttX-7.24 and is, for the most part, compatible with the PIC32MZEC
family.

**STATUS:**

**NuttX-7.9**. The first official release was in NuttX-7.9. Many drivers
port simply from the PIC32MX; others require more extensive efforts.
Driver status as of (2015-03-29) is provided below:

-  I/O ports include I/O port interrupts
-  UART serial driver that provides the NSH console,
-  Timer,
-  I2C (untested),
-  SPI (untested),
-  On-board buttons and LEDs,
-  Ethernet (code complete, but not yet functional),

**NuttX-7.29**. Abdelatif Guettouche contributed additional timer
support including: Timer lower half driver, free-running, and one-shot
timers.

**NuttX-7.31**. Abdelatif Guettouche contributed DMA support.

**NuttX-9.0**. Cache operations were implemented.

Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/mips/pic32mz/pic32mz-starterkit/README.txt>`__
file for further information.

**Development Environment:** Same as for the PIC32MZ.

Microchip PIC32MZEC
-------------------

PIC32MZEC Family (MIPS microAptiv). A port is in available for the
PIC32MZ Embedded Connectivity (EC) Starter Kit. There are two
configurations of the Microchip PIC32MZ EC Starter Kit:

#. The PIC32MZ Embedded Connectivity Starter Kit based on the
   PIC32MZ2048ECH144-I/PH chip (DM320006), and
#. The PIC32MZ Embedded Connectivity Starter Kit based on the
   PIC32MZ2048ECM144-I/PH w/Crypto Engine (DM320006-C).

See the `Microchip <http://www.microchip.com>`__ website for further
information.

This was a collaborative effort between Kristopher Tate, David Sidrane
and myself. The basic port is functional and a NuttShell (NSH)
configuration is available.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
