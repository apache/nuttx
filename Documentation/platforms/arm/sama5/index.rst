===============
Microchip SAMA5
===============

Microchip SAMA5D2
-----------------

-  **Microchip SAMA5D2 Xplained Ultra development board**. This is the port
   of NuttX to the Microchip SAMA5D2 Xplained Ultra development board. This
   board features the Microchip SAMA5D27 microprocessor.
   See https://www.microchip.com/Developmenttools/ProductDetails/ATSAMA5D2C-XULT

- **Groboards Giant Board**. This is the port of NuttX to the Groboards
  Giant Board board. This board features the Microchip SAMA5D27C-D1G.
  See http://groboards.com/giant-board.

Microchip SAMA5D3
-----------------

There are ports to two Microchip SAMA5D3 boards:

-  **Microchip SAMA5D3\ x-EK development boards**. This is the port of NuttX
   to the Microchip SAMA5D3\ *x*-EK development boards (where *x*\ =1,3,4,
   or 5). These boards feature the Microchip SAMA5D3\ *x* microprocessors.
   Four different SAMA5D3\ *x*-EK kits are available

   -  SAMA5D31-EK with the
      `ATSAMA5D31 <http://www.atmel.com/devices/sama5d31.aspx>`__
   -  SAMA5D33-EK with the
      `ATSAMA5D33 <http://www.atmel.com/devices/sama5d33.aspx>`__
   -  SAMA5D34-EK with the
      `ATSAMA5D34 <http://www.atmel.com/devices/sama5d34.aspx>`__
   -  SAMA5D35-EK with the
      `ATSAMA5D35 <http://www.atmel.com/devices/sama5d35.aspx>`__

   The each kit consist of an identical base board with different
   plug-in modules for each CPU. All four boards are supported by NuttX
   with a simple reconfiguration of the processor type.

   **STATUS**. Initial support for the SAMA5D3x-EK was released in
   NuttX-6.29. That initial support was minimal: There are simple test
   configurations that run out of internal SRAM and extended
   configurations that run out of the on-board NOR FLASH:

   -  A barebones NuttShell (:ref:`NSH <nsh>`) configuration
      that can be used as the basis for further application development.
   -  A full-loaded NuttShell (:ref:`NSH <nsh>`) configuration
      that demonstrates all of the SAMA5D3x features.

   The following support was added in NuttX 6.30:

   -  DMA support, and
   -  PIO interrupts,

   And drivers for

   -  SPI (with DMA support),
   -  AT25 Serial Flash,
   -  Two Wire Interface (TWI), and
   -  HSMCI memory cards.

   NuttX-6.30 also introduces full USB support:

   -  High speed device controller driver,
   -  OHCI (low- and full-speed) and
   -  EHCI (high-speed) host controller driver support.

   With NuttX-6.31, these additional drivers were added:

   -  A 10/100Base-T Ethernet (EMAC) driver,
   -  A 1000Base-T Ethernet (GMAC) driver,
   -  A Real Time Clock (RTC) driver and integrated with the NuttX
      system time logic
   -  ``/dev/random`` using the SAMA5D3x True Random Number Generator
      (TRNG),
   -  A Watchdog Timer (WDT) driver,
   -  A Timer/Counter (TC) library with interface that make be used by
      other drivers that need timer support,
   -  An ADC driver that can collect multiple samples using the
      sequencer, can be trigger by a timer/counter, and supports DMA
      data transfers,
   -  A touchscreen driver based on the special features of the SAMA5D3
      ADC peripheral, An LCD controller (LCDC) frame buffer driver, and
   -  A CAN driver (Testing of the CAN has been delayed because of
      cabling issues).

   Additional board configurations were added to test and demonstrate
   these new drivers including new graphics and NxWM configurations.

   These drivers were added in NuttX-6.32:

   -  A PWM driver with DMA support
   -  An SSC-based I2S driver
   -  Support for Programmable clock outputs
   -  NAND support including support for the PMECC hardware ECC and for
      DMA transfers.

   DBGU support was added in NuttX-7.2 (primarily for the SAMA5D3
   Xplained board).

   NuttX-7.4 added support for the on-board WM8904 CODEC chip and for
   *Tickless* operation.

   Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/sama5/sama5d3x-ek/README.txt>`__
   file for further information.

**Microchip SAMA5D3 Xplained development board** This is the port of NuttX
to the Microchip SAMA5D3 Xplained development board. The board features the
Microchip SAMA5D36 microprocessor. See the `Microchip
Website <http://www.atmel.com/devices/sama5d36.aspx>`__ for additional
information about this board.

**STATUS**. This port is complete as of this writing and ready for
general use. The basic port is expected to be simple because of the
similarity to the SAMAD3\ *x*-EK boards and is available in the NuttX
7.2 release.

Most of the drivers and capabilities of the SAMA5D3x-EK boards can be
used with the SAMA5D3 Xplained board. The primary difference between the
ports is that the SAMA5D3x-EK supports NOR FLASH and NuttX can be
configured to boot directly from NOR FLASH. The SAMA5D3 Xplained board
does not have NOR FLASH and, as a consequence NuttX must boot into SDRAM
with the help of U-Boot.

Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/sama5/sama5d3-xplained/README.txt>`__
file for further information.


Microchip SAMA5D4
-----------------

There is a port in progress on one Microchip SAMA5D4 board:

-  **Microchip SAMA5D4-EK/MB development boards** This is the port of NuttX
   to the Microchip SAMA5D4-MB Rev C. development board (which should be
   compatible with the SAMA5D4-EK). These boards feature the Microchip
   SAMA5D44 microprocessors with compatibility with most of the SAMA5D3
   peripherals.

   **STATUS**. At the time of the release of NuttX-7.3, the basic port
   for the SAMA5D4-MB was complete. The board had basic functionality.
   But full functionality was not available until NuttX-7.4. In
   NuttX-7.4 support was added for the L2 cache, many security features,
   XDMAC, HSMCI and Ethernet integrated with XDMAC, the LCDC, TWI, SSC,
   and most of the existing SAMA5 drivers. Timers were added to support
   *Tickless* operation. The TM7000 LCDC with the maXTouch multi-touch
   controller are also fully support in a special NxWM configuration for
   that larger display. Support for a graphics media player is included
   (although there were issues with the WM8904 audio CODEC on my board).
   An SRAM bootloader was also included. Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/sama5/sama5d4-ek/README.txt>`__
   file for current status.

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain, or 4) Native Windows. All testing has been performed
with the CodeSourcery toolchain (GCC version 4.7.3) in the Cygwin
environment under Windows.
