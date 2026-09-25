=============
Renesas RA8M1
=============

The RA8M1 is a Renesas RA family MCU built around an Arm Cortex-M85 core
(Armv8.1-M, Helium/MVE, running at up to 480 MHz), with TrustZone for
Armv8-M support in hardware.  This NuttX port is a flat, no-TrustZone
image: it always runs in the secure state and uses the secure memory and
register aliases (see the flat/secure model note in the port's board
documentation).

Supported MCUs
==============

The following list includes MCUs from the RA8M1 series and indicates
whether they are supported in NuttX.  Only R7FA8M1AHECBD (BGA224, on the
EK-RA8M1) has been tested; the others are selectable but untested.

=============  ======= ================
MCU            Support Note
=============  ======= ================
R7FA8M1AFECAM  No      Selectable, untested
R7FA8M1AFECBD  No      Selectable, untested
R7FA8M1AFECFB  No      Selectable, untested
R7FA8M1AFECFC  No      Selectable, untested
R7FA8M1AFECFP  No      Selectable, untested
R7FA8M1AHECAM  No      Selectable, untested
R7FA8M1AHECBD  Yes     Tested on the EK-RA8M1
R7FA8M1AHECFB  No      Selectable, untested
R7FA8M1AHECFC  No      Selectable, untested
R7FA8M1AHECFP  No      Selectable, untested
=============  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  =====================================
Peripheral  Support  Notes
==========  =======  =====================================
FLASH       No
CLOCK       Yes      MOCO, HOCO, MOSC, PLL1, PLL2, SCICLK
ICU         Yes
KINT        No
ELC         No
DTC         No
DMAC        No
GPT         No
AGT         No
RTC         No
WDT         No
IWDT        No
SCI         Yes      Just UART (SCI_B0-4, SCI_B9)
IIC         No
SPI         No
SSIE        No
QSPI        No
SDHI        No
CAN         No
USBFS       No
ADC12       No
DAC12       No
ACMPLP      No
OPAMP       No
CRC         No
GPIO        Yes
==========  =======  =====================================

CLOCK
-----

The board's ``include/board.h`` defines ``BOARD_*`` macros that select the clock
sources, PLL settings and dividers, and ``arch/arm/src/ra8m1/ra_clockconfig.h``
derives every clock frequency from them and checks it against the limits in
the RA8M1 User's Manual (chapter 8) at build time.  See the board's own
documentation for the specific clock tree it configures.

The RA8M1 has two PLLs.  PLL1 can feed the system clock or a peripheral
dedicated clock (SCICLK); PLL2 can only feed a dedicated clock.  SCICLK is
enabled automatically whenever an SCI UART is enabled, since it is the
clock of the SCI baud rate generator.

SCI
---

The Serial Communications Interface, Type B (SCI_B) is configurable to
support several serial communication modes: Asynchronous (UART), Clock
synchronous, Simple SPI, Smart card interface, Simple IIC (master-only).
NuttX driver support covers UART mode.  RA8M1 has SCI_B0-4 and SCI_B9 (no
SCI_B5-8); each channel's baud rate generator runs from SCICLK.

GPIO
----

Pins can be configured/operated using ``ra_gpio_*`` functions.

ICU
---

The Interrupt Controller Unit routes peripheral events to NVIC vectors:
any event can be routed to any of its IELSR slots.  NuttX assigns SCI_B
events to fixed slots (four per channel) and calls ``ra_attach_icu()`` at
start-up to route the enabled channels' events into them.

Supported Boards
=================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
