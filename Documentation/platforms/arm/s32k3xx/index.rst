===========
NXP S32K3XX
===========

The `S32K3XX series <https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32k-general-purpose-mcus/s32k3-microcontrollers-for-general-purpose:S32K3>`_ is a family of automotive-grade general-purpose microcontrollers from NXP Semiconductors. The chips are based around single, dual (lock-step) or tripple Arm Cortex-M7 cores, running at clockspeeds up to 240 MHz.

Supported MCUs
==============

The following list includes MCUs from the S32K3XX series and indicates whether they are supported in NuttX:

========  =======  =================  =========
MCU       Support  Cores              Frequency
========  =======  =================  =========
S32K311   No       1x Cortex-M7       120 MHz
S32K312   No       1x Cortex-M7       120 MHz
S32K314   Yes*     1x Cortex-M7       160 MHz
S32K322   No       2x Cortex-M7       160 MHz
S32K324   Yes*     2x Cortex-M7       160 MHz
S32K341   No       LS Cortex-M7       160 MHz
S32K342   No       LS Cortex-M7       160 MHz
S32K344   Yes*     LS Cortex-M7       160 MHz
S32K328   No       2x Cortex-M7       160 MHz
S32K338   No       3x Cortex-M7       240 MHz
S32K348   No       LS Cortex-M7       160 MHz
S32K358   No       LS + 1x Cortex-M7  160 MHz
========  =======  =================  =========

| \* Same silicon in different configurations. Only a single core is currently being used (no SMP support).

Supported Modules & Peripherals
===============================

The following list indicates modules and peripherals supported in NuttX. Note that this list does not include all MCU modules. Please refer to the S32K3XX Reference Manual for a complete overview of its features.

==========  =======  ==========================
Peripheral  Support  Comments
==========  =======  ==========================
ADC         No
eDMA        Partial
eMIOS       No
EMAC        Yes
FlexCAN     Yes      SocketCAN-compatible
FlexIO      No
GPIO        Yes
LPCMP       No
LPI2C       Yes      I2C Master only
LPSPI       Yes
LPUART      Yes
QSPI        Yes
RTC         No
SAI         No
==========  =======  ==========================

ADC
---

12-Bit Successive Approximation (SAR) Analog-to-Digital Converter (ADC). No driver support (yet).

eDMA
----

Enhanced Direct Memory Access module. There is a driver that was copied from the S32K1XX and i.MX RT ports. Some modifications have been made to make it work with S32K3XX, but it is far from feature-complete.

eMIOS
-----

The Enhanced Modular IO Subsystem (eMIOS) is a flexible timer and I/O module for real-time control applications. Its channels can be used for (pulse) counters, PWM outputs, input period measurements, and more. There is no driver implementation available (yet).

EMAC
----

10/100/200 Mbit/s Ethernet MAC. Driver is available.

FlexCAN
-------

The S32K3XX family has up to 8x FlexCAN modules with CAN FD support. A SocketCAN-compatible driver is available.

FlexIO
------

A configurable module providing a range of functionality like emulation of a variety of serial/parallel communication protocols, flexible 16-bit timers or programmable logic blocks. No driver available.

GPIO
----

Pins can be configured using :c:func:`s32k3xx_pinconfig` function. Writing to pins is done by :c:func:`s32k3xx_gpiowrite` function and reading is done by :c:func:`s32k3xx_gpioread`.

LPCMP
---

Analog Comparator. No driver support (yet).

LPI2C
-----

Low-Power Inter-Integrated Circuit (I2C) module supporting an interface to an I2C bus as master and/or
as a slave. The lower-half of this driver is initialize by calling :c:func:`s32k3xx_i2cbus_initialize`.

LPSPI
-----

Low-Power Serial Peripheral Interface (SPI) module that supports an interface to an SPI bus as a master
and/or a slave. The lower-half of this driver is initialize by calling :c:func:`s32k3xx_lpspibus_initialize`.

LPUART
------

Low-Power Universal Asynchronous Receiver/Transmitter (UART) module. UART is initialized automatically during
MCU boot.

QSPI
----

QuadSPI memory interface for external serial flash devices. A basic driver is available.

RTC
---

Real-Time Clock module. A driver is not (yet) available.

SAI
---

Synchronous Audio Interface for digital audio over I2S (Inter-IC Sound). No driver implemented.


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
