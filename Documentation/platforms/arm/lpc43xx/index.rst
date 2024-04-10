===========
NXP LPC43xx
===========

Several board ports are available for this higher end, NXP
Cortex-M4F part:

**NXG Technologies LPC4330-Xplorer**. This NuttX port is for the
LPC4330-Xplorer board from NGX Technologies featuring the NXP
LPC4330FET100 MCU. See the `NXG
website <http://shop.ngxtechnologies.com/product_info.php?cPath=21_37&products_id=104>`__
for further information about this board.

-  **STATUS:** Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/lpc43xx/boards/lpc4330-xplorer/README.txt>`__
   file for more detailed information about this port.

-  **NuttX-6.20** The basic LPC4330-Xplorer port is complete. The basic
   NuttShell (NSH) configuration is present and fully verified. This
   includes verified support for: SYSTICK system time, pin and GPIO
   configuration, and a serial console.

**NXP/Embest LPC4357-EVB**. This NuttX port is for the LPC4357-EVB from
NXP/Embest featuring the NXP LPC4357FET256 MCU. The LPC4357 differs from
the LPC4330 primarily in that it includes 1024KiB of on-chip NOR FLASH.
See the `NXP
website <http://www.nxp.com/news/news-archive/2013/nxp-development-kit-based-on-the-dual-core-lpc4357-microcontroller.html>`__
for more detailed information about the LPC4357 and the LPC4357-EVB.

-  **STATUS:** Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/lpc43xx/boards/lpc4357-evb/README.txt>`__
   file for more detailed information about this port.

-  **NuttX-7.6**. The basic port is was contributed by Toby Duckworth.
   This port leverages from the LPC4330-Xplorer port (and, as of this
   writing, still requires some clean up of the technical discussion in
   some files). The basic NuttShell (NSH) configuration is present and
   has been verified. Support is generally the same as for the
   LPC4330-Xplorer as discussed above.

**NXP LPC4370-Link2**. This is the NuttX port to the NXP LPC4370-Link2
development board featuring the NXP LPC4370FET100 MCU.

-  **STATUS:** Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/lpc43xx/boards/lpc4370-link2/README.txt>`__
   file for more detailed information about this port.

-  **NuttX-7.12** The NXP LPC4370-Link2 port is was contributed by Lok
   Tep and first released in NuttX-7.12.

**WaveShare LPC4337-WS**. This is the NuttX port to the WaveShare
LPC4337-WS development board featuring the NXP LPC4337JBD144 MCU.

-  **STATUS:** Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/Documentation/platforms/arm/lpc43xx/boards/lpc4337-ws/README.txt>`__
   file for more detailed information about this port.

-  **NuttX-7.14** The NXP WaveShare LPC4337-WS port is was contributed
   by Lok Tep and first released in NuttX-7.14.

-  **NuttX-7.16** Support for the LPC4337JET100 chip was contribed by
   Alexander Vasiljev. Alexander also contributed an LPC43xx AES driver
   available in NuttX-7.16.

**Driver Status**.

-  **NuttX-6.20** Several drivers have been copied from the related
   GPDMA, I2C, SPI, and SSP. The registers for these blocks are the same
   in both the LPC43xx and the LPC17xx and they should integrate into
   the LPC43xx very easily by simply adapting the clocking and pin
   configuration logic.

   Other LPC17xx drivers were not brought into the LPC43xx port because
   these peripherals have been completely redesigned: CAN, Ethernet, USB
   device, and USB host.

   So then there is no support for the following LPC43xx peripherals:
   SD/MMC, EMC, USB0,USB1, Ethernet, LCD, SCT, Timers 0-3, MCPWM, QEI,
   Alarm timer, WWDT, RTC, Event monitor, and CAN.

   Some of these can be leveraged from other MCUs that appear to support
   the same peripheral IP:

   -  The LPC43xx USB0 peripheral appears to be the same as the USB OTG
      peripheral for the LPC31xx. The LPC31xx USB0 device-side driver
      has been copied from the LPC31xx port but also integration into
      the LPC43xx (clocking and pin configuration). It should be
      possible to complete porting of this LPC31xx driver with a small
      porting effort.
   -  The Ethernet block looks to be based on the same IP as the STM32
      Ethernet and, as a result, it should be possible to leverage the
      NuttX STM32 Ethernet driver with a little more effort.

-  **NuttX-6.21** Added support for a SPIFI block driver and for RS-485
   option to the serial driver.

-  **NuttX-7.17** EMC support was extended to include support SDRAM by
   Vytautas Lukenska.

-  **NuttX-7.23** A CAN driver was contributed by Alexander Vasiljev in
   NuttX-7.23.

-  **NuttX-7.24** RTC and Windowed Watchdog Timer (WWDT) drivers were
   leveraged from the LPC17 and contributed by Gintaras Drukteinis.
   Leveraged the LPC54xx SD/MMC to the LPC43xx. There are still
   remaining issues with the SD/MMC driver and it is not yet functional.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
