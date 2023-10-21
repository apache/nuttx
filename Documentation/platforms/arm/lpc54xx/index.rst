===========
NXP LPC54xx
===========

A port to the
`LPCXpresso-LPC54628 <https://www.nxp.com/support/developer-resources/hardware-development-tools/lpcxpresso-boards/lpcxpresso54628-development-board:OM13098>`__
was added in NuttX-7.24. Initial configurations include: A basic NSH
configuration (nsh), a networking configuration (netnsh), and three
graphics configurations (nxwm, fb, and lvgl).

**LPC4508**. The port was verified on an LPC5408 by a NuttX user with
relevant changes incorporated in NuttX-7.26.

**Driver Status**.

-  **NuttX-7.24** The initial release for the LPC54xx in NuttX included
   the following drivers: UARTs, SysTick, SD/MMC, DMA, GPIO, GPIO
   interrupts, LEDs and buttons, LCD, WWDT, RTC, RNG, Ethernet, and SPI.
   The SPI driver is untested and there are known issues with the SD/MMC
   driver, however.

-  **NuttX-7.29** Configurations were added to verify the "Per-Window
   Framebuffer" feature also added in NuttX-7.29.

Refer to the LPCXpresso-LPC54628 board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc54xx/lpcxpresso-lpc54628/README.txt>`__
file for more detailed information about this port.
