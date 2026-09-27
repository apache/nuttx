===========
NXP LPC40xx
===========

The LPC40xx family is very similar to the LPC17xx family
except that it features a Cortex-M4F versus the LPC17xx's Cortex-M3.
Architectural support for the LPC40xx family was built on top of the
existing LPC17xx by jjlange in NuttX-7.31. With that architectural
support came support for two boards also contributed by jjlange:

.. container:: review-authored

   Those two are the Embedded Artists **LPC4088 Developer's Kit** and
   **LPC4088 Quickstart**, merged together on 2019-07-11 and released in
   NuttX-7.31 on 2019-07-21.  Neither is named above because neither has a
   page: their directories here carry only a ``README.txt``, so they never
   reach a board list.

**LX CPU**. Pavel Pisa added support for the PiKRON LX CPU board. This
board may be configured to use either the LPC4088 or the LPC1788.

.. container:: review-authored

   **Driver status.**  There is no separate set of LPC40xx drivers to report
   on, which is the point of the family: it shares
   ``arch/arm/src/lpc17xx_40xx/`` with the LPC17xx.  Of the 37 source files
   there, two are LPC178x/40xx implementations of their own --
   ``lpc178x_40xx_clockconfig.c`` and ``lpc178x_40xx_gpio.c`` -- and seven
   more are shared files that branch on the family: ``lpc17_40_clockconfig.c``,
   ``lpc17_40_gpio.c``, ``lpc17_40_gpioint.c``, ``lpc17_40_gpiodbg.c``,
   ``lpc17_40_can.c``, ``lpc17_40_ssp.c`` and ``lpc17_40_lowputc.c``.
   The Kconfig declares five chips -- ``LPC4072``, ``LPC4074``, ``LPC4076``,
   ``LPC4078`` and ``LPC4088`` -- across the ``ARCH_FAMILY_LPC407X`` and
   ``ARCH_FAMILY_LPC408X`` families.

   All three LPC4088 boards are listed together with the LPC17xx boards on
   the :doc:`index` page.
