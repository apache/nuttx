==============
 Nordic nRF91
==============

The nRF91 series of chips from Nordic Semiconductor are based around an ARM Cortex-M33 core
with integrated LTE-M/NB-IoT modem and GNSS.

Modem Support
=============

Modem is supported in the nRF91 using Nordic's `Modem library <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/nrf_modem/README.html>`_.

Supported modem features:

=============  ========= ===============
Modem feature  Support   Notes
=============  ========= ===============
AT             Yes       ``/dev/modem``
Socket         Yes       via ``usrsock``
SIOCLTECMD     Partial
GNSS           Yes
AGPS           No
Bootloader     No
Delta DFU      No
Modem Trace    No
=============  ========= ===============

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= =================
Peripheral  Support Notes
==========  ======= =================
CRUPTOCELL  No
DPPI        No
EGU         No
GPIO        Yes
GPIOTE      Yes
IPC         Yes      nrfx compatible
I2S         No
KMU         No
PDM         No
PWM         Yes
RTC         Yes
SPIM        Yes
SPIS        No
SPU         Yes
TIMER       Yes
TWIM        Yes
TWIS        No
UART        Yes
UARTE       No
WDT         No
==========  ======= =================

I2C
---

I2C is supported both in polling and interrupt mode (via EasyDMA).

.. note:: The I2C peripheral does not support sending two transfers without sending
   a START nor RSTART. For this reason, this is supported via an internal buffer where
   messages will be first copied to and sent together.

The lower-half of I2C bus is initialized by :c:func:`nrf91_i2cbus_initialize`.

SPI
---

SPI is supported both in polling and interrupt-based (via EasyDMA) mode. The latter
supports arbitrarily long transfers using Nordic's list-mode EasyDMA (intermediate
transfers are currently still manually started).

It is possible to use SPI without either MOSI/MISO pin defined by simply not providing
the relevant ``BOARD_SPI*_MISO/MOSI_PIN`` definition.

This implementation support power management hooks, which will disable SPI peripheral when
entering either SLEEP or STANDBY modes and reconfigure it when going back to NORMAL mode.

UART
----

UART is implemented using polling. UARTE EasyDMA feature is not yet supported.
This may introduce a large number of interrupts which may be undesirable.

PWM
---

PWM is supported via standard driver. This means that more advanced features such as
complex sequences or waveform modes are not yet supported.

TIMER
-----

The TIMER peripheral is exposed as standard timer.

RTC
---

The RTC peripheral is exposed as a standard timer, since it is really a low-power
timer, without any date handling capabilities.

Tool Issues
===========

OpenOCD
-------------
There is no official support for Nordic Cortex M33 chips (nRF9160 or nRF5340).

Segger J-Link
-------------
To start the GDB servers for the application core, use these commands::

    JLinkGDBServer -device nRF9160 -if SWD -speed 4000 -port 2331

Then you can connect GDB to targets::

    (gdb) target remote localhost:2331

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
