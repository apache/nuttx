===========
NXP LPC17xx
===========

NXP LPC176x
-----------

NXP LPC1766, LPC1768, and LPC1769. Drivers are available for CAN, DAC,
Ethernet, GPIO, GPIO interrupts, I2C, UARTs, SPI, SSP, USB host, and USB
device. Additional drivers for the RTC, ADC, DAC, Timers, PWM and MCPWM
were contributed by Max (himax) in NuttX-7.3. Verified LPC17xx
configurations are available for these boards:

-  The Nucleus 2G board from `2G Engineering <http://www.2g-eng.com/>`__
   (LPC1768),
-  The mbed board from `mbed.org <http://mbed.org>`__ (LPC1768,
   Contributed by Dave Marples), and
-  The LPC1766-STK board from `Olimex <http://www.olimex.com/>`__
   (LPC1766).
-  The Embedded Artists base board with NXP LPCXpresso LPC1768.
-  Zilogic's ZKIT-ARM-1769 board.
-  The `Micromint <http://micromint.com/>`__ Lincoln60 board with an NXP
   LPC1769.
-  A version of the LPCXPresso LPC1768 board with special support for
   the U-Blox model evaluation board.
-  Support for the Keil MCB1700 was contributed by Alan Carvalho de
   Assis in NuttX-7.23.
-  Support for the NXP Semiconductors' PN5180 NFC Frontend Development
   Kit was contributed by Michael Jung in NuttX-7.1. This board is based
   on the NXP LPC1769 MCU.

The Nucleus 2G board, the mbed board, the LPCXpresso, and the MCB1700
all feature the NXP LPC1768 MCU; the Olimex LPC1766-STK board features
an LPC1766. All use a GNU arm-nuttx-elf or arm-eabi toolchain\* under
either Linux or Cygwin (with native Windows GNU tools or Cygwin-based
GNU tools).

**STATUS:** The following summarizes the features that has been
developed and verified on individual LPC17xx-based boards. These
features should, however, be common and available for all LPC17xx-based
boards.

#. **Nucleus2G LPC1768**

   -  Some initial files for the LPC17xx family were released in NuttX
      5.6, but
   -  The first functional release for the NXP LPC1768/Nucleus2G
      occurred with NuttX 5.7 with Some additional enhancements through
      NuttX-5.9. Refer to the NuttX board
      `README <https://bitbucket.org/patacongo/obsoleted/src/master/configs/nucleus2g/README.txt>`__
      file for further information.

   That initial, 5.6, basic release included *timer* interrupts and a
   *serial console* and was verified using the NuttX OS test
   (``apps/examples/ostest``). Configurations available include include
   a verified NuttShell (NSH) configuration (see :ref:`NSH <nsh>`). The
   NSH configuration supports the Nucleus2G's microSD slot and
   additional configurations are available to exercise the USB serial
   and USB mass storage devices. However, due to some technical reasons,
   neither the SPI nor the USB device drivers are fully verified.
   (Although they have since been verified on other platforms; this
   needs to be revisited on the Nucleus2G).

   **Obsoleted**. Support for the Nucleus2G board was terminated on
   2016-04-12. There has not been any activity with the commercial board
   in a few years and it no longer appears to be available from the
   2g-eng.com website. Since the board is commercial and no longer
   publicly available, it no longer qualifies for inclusion in the open
   source repositories. A snapshot of the code is still available in the
   `Obsoleted
   repository <https://bitbucket.org/patacongo/obsoleted/src/master/boards/nucleus2g>`__
   and can easily be *reconstitued* if needed.

#. **mbed LPC1768**

   -  Support for the mbed board was contributed by Dave Marples and
      released in NuttX-5.11. Refer to the NuttX board
      `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/mbed/README.txt>`__
      file for further information.

#. **Olimex LPC1766-STK**

   -  Support for that Olimex-LPC1766-STK board was added to NuttX 5.13.
   -  The NuttX-5.14 release extended that support with an *Ethernet
      driver*.
   -  The NuttX-5.15 release further extended the support with a
      functional *USB device driver* and *SPI-based micro-SD*.
   -  The NuttX-5.16 release added a functional *USB host controller
      driver* and *USB host mass storage class driver*.
   -  The NuttX-5.17 released added support for low-speed USB devices,
      interrupt endpoints, and a *USB host HID keyboard class driver*.
   -  Refer to the NuttX board
      `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/olimex-lpc1766stk/README.txt>`__
      file for further information.

   Verified configurations are now available for the NuttShell with
   networking and microSD support(NSH, see :ref:`NSH <nsh>`), for
   the NuttX network test, for the
   `THTTPD <http://acme.com/software/thttpd>`__ webserver, for USB
   serial deive and USB storage devices examples, and for the USB host
   HID keyboard driver. Support for the USB host mass storage device can
   optionally be configured for the NSH example. A driver for the *Nokia
   6100 LCD* and an NX graphics configuration for the Olimex LPC1766-STK
   have been added. However, neither the LCD driver nor the NX
   configuration have been verified as of the NuttX-5.17 release.

#. **Embedded Artists base board with NXP LPCXpresso LPC1768**

   An fully verified board configuration is included in NuttX-6.2. The
   Code Red toolchain is supported under either Linux or Windows.
   Verified configurations include DHCPD, the NuttShell (NSH), NuttX
   graphis (NX), THTTPD, and USB mass storage device. Refer to the NuttX
   board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/lpcxpresso-lpc1768/README.txt>`__
   file for further information.

#. **Zilogic's ZKIT-ARM-1769 board**

   Zilogic System's ARM development Kit, ZKIT-ARM-1769. This board is
   based on the NXP LPC1769. The initial release was included
   NuttX-6.26. The NuttX Buildroot toolchain is used by default. Verifed
   configurations include the "Hello, World!" example application and a
   THTTPD demonstration. Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/zkit-arm-1769/README.txt>`__
   file for further information.

#. **Micromint Lincoln60 board with an NXP LPC1769**

   This board configuration was contributed and made available in
   NuttX-6.20. As contributed board support, I am unsure of what all has
   been verfied and what has not. See the Microment website
   and the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/lincoln60/README.txt>`__
   file for further information about the Lincoln board.

#. **U-Blox Modem Evaluation (LPCXpresso LPC1768)**

   This board configuration was contributed by Vladimir Komendantskiy
   and made available in NuttX-7.15. This is a variant of the LPCXpresso
   LPC1768 board support with special provisions for the U-Blox Model
   Evaluation board. See the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/lpc17xx_40xx/u-blox-c027/README.txt>`__
   file for further information about this port.

#. **Keil MCB1700 (LPC1768)**

   This board configuration was contributed by Alan Carvalho de Assis in
   NuttX-7.23.

#. **PN5180 NFC Frontend Development Kit**

   This board configuration was contributed by Michael Jung in
   NuttX-7.31.

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain (CodeSourcery devkitARM or Code Red), or 4) Native
Windows. A DIY toolchain for Linux or Cygwin is provided by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package.

NXP LPC178x
-----------

The port of NuttX to the WaveShare Open1788 is a
collaborative effort between Rommel Marcelo and myself (with Rommel
being the leading contributor and I claiming only a support role). You
can get more information at the Open1788 board from the WaveShare
website.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
