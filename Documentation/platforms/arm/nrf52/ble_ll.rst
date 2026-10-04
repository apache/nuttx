======================
Nordic nRF52 BLE Radio
======================

.. note::

   This driver is experimental.

Radio lower half for the :doc:`/os/wireless/ble_ll`, derived from the
Apache Mynewt NimBLE nRF5x driver.  It is enabled with
``CONFIG_NRF52_BLE_CONTROLLER`` and initialized by the board with
:c:func:`nrf52_ble_initialize`.

It cannot be used together with the SoftDevice Controller.

LE Coded PHY (``CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY``) is available on
nRF52833 and nRF52840.

Reserved peripherals
====================

* ``RADIO`` - radio operations
* ``TIMER0`` - microsecond timer for radio events
* ``RTC0`` - Link Layer timer, ``CC[0]`` starts ``TIMER0`` and ``CC[2]``
  drives the Link Layer scheduler
* ``CCM`` - link encryption
* ``AAR`` - resolvable private address resolution
* ``ECB`` - AES-128 block encryption
* ``RNG`` - random numbers

PPI channels
============

The pre-programmed PPI channels 20, 21, 23, 25, 26, 27 and 31 are used.  Two
configurable channels implement the wait for response timer:
``CONFIG_NRF52_BLE_PPI_WFR0`` and ``CONFIG_NRF52_BLE_PPI_WFR1`` (channels 4
and 5 by default).

Interrupts
==========

``RADIO``, ``RTC0`` and ``RNG`` interrupts run at the default priority, so
they are masked by critical sections in the Link Layer.

Clocks
======

The low frequency clock is required (``CONFIG_NRF52_USE_LFCLK``).  The high
frequency crystal oscillator is requested when the Link Layer enables the
radio clock.

Configurations
==============

``nrf52840-dk:blell_nimble`` and ``nrf52832-dk:blell_nimble`` run the NimBLE
host from apps on top of the Link Layer.
