=====================
LAUNCHXL2-RM57L
=====================

.. tags:: chip:rm57, arch:armv7-r, vendor:ti

The `LAUNCHXL2-RM57L <https://www.ti.com/tool/LAUNCHXL2-RM57L>`__ is a
LaunchPad-format evaluation board from Texas Instruments built around the
Hercules RM57L843 safety microcontroller (dual Cortex-R5F cores in
lockstep). It's aimed at evaluating TI's Hercules safety MCU family rather
than any particular application.

.. warning::

   This board port is new and experimental. Only the serial console (SCI1)
   and the two user LEDs are currently supported; pin-mux configuration,
   button support, and most on-chip peripherals are not yet implemented.
   Some of the values documented below (PLL/clock configuration, LED
   polarity, JTAG IDCODE) were taken from TI's HALCoGen-generated reference
   project or from the RM57L843 datasheet rather than confirmed against
   this specific board's schematic — see the comments in
   ``boards/arm/rm57/rm57l843-launchxl2/include/board.h`` for details.

Features
========

* TI Hercules RM57L843 microcontroller
* Dual-core lockstep ARM Cortex-R5F, running at 150 MHz HCLK (300 MHz
  PLL/GCLK) in the current clock configuration
* 4 MB of on-chip program flash, 512 KB of on-chip SRAM
* Onboard XDS110 debug probe (JTAG)
* 2 user LEDs
* Single SCI (serial) interface currently supported

.. note::

   Peripherals that are part of the RM57L843 chip itself (N2HET, MibSPI,
   CAN, ADC, the ESM diagnostic module, PBIST/STC self-test, etc.) but are
   not yet wired up by this board port are tracked on the
   :doc:`RM57 chip documentation page </platforms/arm/rm57/index>`.

Buttons and LEDs
================

LEDs
----

The LAUNCHXL2-RM57L has two user LEDs, labeled **B6** and **B7** on the
board silkscreen, driven by GIOB[6] and GIOB[7] respectively.

Buttons
-------

Button support is not yet implemented by this board port.

Pin Mapping
===========

Only the pins used by the currently supported peripherals are listed.

.. list-table::
   :widths: auto
   :header-rows: 1

   * - Pin
     - Signal
     - Notes
   * - (n/a)
     - LIN1RX / LIN1TX
     - SCI1, used as the serial console
   * - J2
     - GIOB[6]
     - User LED B6
   * - F1
     - GIOB[7]
     - User LED B7

.. note::

   The RM57L843 SCI1/LIN1 pins are used at their reset-default (primary)
   function, so no pin-mux configuration is required for the console.
   Header/connector pin numbers for LIN1RX/LIN1TX have not been confirmed
   against the LAUNCHXL2-RM57L schematic.

Serial Console
==============

SCI1 is used as the serial console. The default configuration is:

* 9600 baud, 8 data bits, no parity, 2 stop bits

The baud rate and stop bits are configurable via ``CONFIG_SCI1_BAUD`` and
``CONFIG_SCI1_2STOP``.

Power Supply
============

The LAUNCHXL2-RM57L can be powered over USB through the onboard XDS110
debug probe. Consult the
`LAUNCHXL2-RM57L user's guide <https://www.ti.com/tool/LAUNCHXL2-RM57L>`_
for the full range of supported input voltages.

Debugging
=========

The board's onboard XDS110 probe exposes a JTAG interface (Hercules'
ICEpick-C JTAG router is JTAG-only; XDS110's default SWD mode is not
usable here). OpenOCD can be used with a Cortex-R5 (``cortex_r4`` driver)
target configuration that routes through the ICEpick-C, followed by GDB
to load and debug the image.

nsh
---

Basic NuttShell configuration (console enabled on SCI1, at 9600 baud).
