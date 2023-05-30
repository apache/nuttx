============
Nordic nRF53
============

The nRF53 series of chips from Nordic Semiconductor are based around an two ARM Cortex-M33 cores
and a multiprotocol 2.4 GHz transceiver.

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  ======= ===============
Peripheral  Support Notes
==========  ======= ===============
GPIO        Yes
GPIOTE      Yes
I2S         No
MWU         No
NFCT        No
PDM         No
DPPI        No
PWM         Yes
QDEC        No
QSPI        Yes
RADIO       No
RNG         No
RTC         Yes
SAADC       Yes
SPIM        Yes
SPIS        No
TEMP        No
TIMER       Yes
TWIM        Yes
TWIS        No
UART        Yes
UARTE       No
USBD        Yes
WDT         No
IPC         Yes     RPTUN supported
==========  ======= ===============

GPIO/GPIOTE
-----------

Pins can be configured/operated using ``nrf53_gpio_*`` functions. Interrupts are
handled via the GPIOTE peripheral in one of two ways: via a GPIOTE channel or via
PORT events. The former allows for simultaneous rising/falling edge-sensitive interrupts
per-pin. However, as there are a limited number of channels (and sometimes these
are used by some drivers for specific tasks), it may not always be possible to use
this mechanism. The latter approach for pin interrupts is via the PORT event, determined
by pin state on a their corresponding GPIO port. This is related to the SENSE capability
of pins, which can only be set to either rising or falling edge sensing.

Depending on ``CONFIG_NRF53_PER_PIN_INTERRUPTS`` option, you can set a callback for
the PORT event itself or you can set a callback for a given pin. In the latter case
the driver scans for pins with DETECT bit high and calls the configured callback
automatically.

Finally, GPIOTE can also be used to configure a channel in *task mode*, which allows to
control pin state via tasks/events.

ADC
---

The SAADC peripheral is exposed via standard ADC driver. The lower-half of this driver
is initialized by calling :c:func:`nrf53_adcinitialize`.

I2C
---

I2C is supported both in polling and interrupt mode (via EasyDMA).

.. note:: The I2C peripheral does not support sending two transfers without sending
   a START nor RSTART. For this reason, this is supported via an internal buffer where
   messages will be first copied to and sent together.

The lower-half of I2C bus is initialized by :c:func:`nrf53_i2cbus_initialize`.

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

QSPI
----

QSPI is supported both in interrupt-based (via EasyDMA) mode and is exposed
via standard QSPI interface.

TIMER
-----

The TIMER peripheral is exposed as standard timer.

RTC
---

The RTC peripheral is exposed as a standard timer, since it is really a low-power
timer, without any date handling capabilities.

USBD
----

The USBD peripheral is exposed via standard USBDEV interface.

Interprocessor Communication
============================

Interprocessor communication between the application core and the network core is realized with
the NuttX RPTUN device based on the OpenAMP framework.

The last 32kB of the application core RAM is used for a shared memory (address = 0x20078000).

BLE Support
===========

BLE is supported in the nRF53 using Nordic's `SoftDevice Controller <https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrfxlib/softdevice_controller/README.html>`_.

Tool Issues
===========

OpenOCD
-------------
There is no official support for Nordic Cortex M33 chips (nRF9160 or nRF5340).

Segger J-Link
-------------
To start the GDB servers for the application core and the network core, use these commands::

    JLinkGDBServer -device nRF5340_xxAA_APP -autoconnect 1 -if SWD -speed 4000 -port 2331 -swoport 2332 -telnetport 2333
    JLinkGDBServer -device nRF5340_xxAA_NET -autoconnect 1 -if SWD -speed 4000 -port 2334 -swoport 2335 -telnetport 2336

Then you can connect GDB to targets::

    (gdb_app) target remote localhost:2331
    (gdb_net) target remote localhost:2334

Flashing locked device
----------------------

1. Unlock the network core::

    nrfjprog --recover --coprocessor CP_NETWORK

2. Unlock the application core::

    nrfjprog --recover

3. Flash the network core::

    nrfjprog --coprocessor CP_NETWORK --program nuttx_net.hex --verify --chiperase

4. Flash the application core::

    nrfjprog --program nuttx_app.hex --verify --chiperase


Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
