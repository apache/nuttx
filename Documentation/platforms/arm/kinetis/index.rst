=====================
NXP/FreeScale Kinetis
=====================

NXP/FreeScale Kinetis K20
-------------------------

Used by Teensy-3.x. Architecture support (only) was
added in NuttX-7.10. This support was taken from PX4 and is the work of
Jakob Odersky. Support was added for the PJRC Teensy-3.1 board in
NuttX-7.11. Backward compatible support for the Teensy-3.0 is included.

NXP/FreeScale Kinetis K28F
--------------------------

Use by Freedom-K28F. Architecture support for the
Kinetis K28F along with board support for the Freedom-K28F was added in
NuttX-7.15. The Freedom-K28F board is based on the Kinetis
MK28FN2M0VMI15 MCU (ARM Cortex-M4 at150 MHz, 1 MB SRAM, 2 MB flash, HS
and FS USB, 169 MAPBGA package). More information is available from the
`NXP
website <https://www.nxp.com/support/developer-resources/hardware-development-tools/freedom-development-boards/mcu-boards/nxp-freedom-development-board-for-kinetis-k27-and-k28-mcus:FRDM-K28F>`__.

NXP/FreeScale Kinetis K40
-------------------------

This port uses the Freescale Kinetis KwikStik
K40. Refer to the `Freescale web
site <http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KWIKSTIK-K40>`__
for further information about this board. The Kwikstik is used with the
FreeScale Tower System (mostly just to provide a simple UART connection)

NXP/FreeScale Kinetis K60
-------------------------

This port uses the **Freescale Kinetis
TWR-K60N512** tower system. Refer to the `Freescale web
site <http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=TWR-K60N512-KIT>`__
for further information about this board. The TWR-K60N51 includes with
the FreeScale Tower System which provides (among other things) a DBP
UART connection.

**MK60N512VLL100**. Architecture support for the MK60N512VLL100 was
contributed by Andrew Webster in NuttX-7.14.

NXP/FreeScale Kinetis K64
-------------------------

Support for the Kinetis K64 family and
specifically for the **NXP/Freescale Freedom K64F** board was added in
NuttX 7.17. Initial release includes two NSH configurations with support
for on-board LEDs, buttons, and Ethernet with the on-board KSZ8081 PHY.
SDHC supported has been integrated, but not verified. Refer to the NuttX
board
`README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/kinetis/boards/freedom-k64f/README.txt>`__
file for further information.

**MK64FN1M0VMD12**. Architecture support for the \_MK64FN1M0VMD12 was
contributed by Maciej Skrzypek in NuttX-7.20.

**NXP/Freescale Kinetis TWR-K64F120M**. Support for the Freescale
Kinetis TWR-K64F120M was contributed in NuttX-7.20 by Maciej Skrzypek.
Refer to the `Freescale web
site <http://www.nxp.com/products/sensors/accelerometers/3-axis-accelerometers/kinetis-k64-mcu-tower-system-module:TWR-K64F120M>`__
for further information about this board. The board may be complemented
by
`TWR-SER <http://www.nxp.com/pages/serial-usb-ethernet-can-rs232-485-tower-system-module:TWR-SER>`__
which includes (among other things), an RS232 and Ethernet connections.
Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/kinetis/boards/twr-k64f120m/README.txt>`__
file for further information.

**Driver Status**.

-  **NuttX-6.8**. Ethernet and SD card (SDHC) drivers also exist: The
   SDHC driver is partially integrated in to the NSH configuration but
   has some outstanding issues. Additional work remaining includes: (1)
   integrate th SDHC drivers, and (2) develop support for USB host and
   device. NOTE: Most of these remaining tasks are the same as the
   pending K40 tasks described above.
-  **NuttX-7.14**. The Ethernet driver became stable in NuttX-7.14
   thanks to the efforts of Andrew Webster.
-  **NuttX-7.17**. Ethernet support was extended and verified on the
   Freedom K64F. A Kinetis USB device controller driver and PWM support
   was contributed by kfazz.

NXP/FreeScale Kinetis K66
-------------------------

Support for the Kinetis K64 family and
specifically for the **NXP/Freescale Freedom K66F** board was
contributed by David Sidrane in NuttX 7.20. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/kinetis/boards/freedom-k66f/README.txt>`__
file for further information.

**Driver Status**.

-  Most K6x drivers are compatible with the K66.
-  **NuttX-7.20**. David Sidrane also contributed support for a serial
   driver on the K66's LPUART.
-  **NuttX-7.22**. David Sidrane contributed improvements to the USB and
   I2C device drivers, RTC alarm functionality, and new SPI driver.
-  **NuttX-7.26**. David Sidrane contributed DMA support to the Kinetis
   K6x family.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
