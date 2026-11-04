=======================
Nordic nRF54L BLE Radio
=======================

.. note::

   This driver is experimental and has not been tested on hardware.

Radio lower half for the :doc:`/os/wireless/ble_ll`, a port of the
:doc:`/platforms/arm/nrf52/ble_ll` with the nRF54L DPPI, GRTC and CCM/AAR
programming taken from the Zephyr Bluetooth controller.  It is enabled with
``CONFIG_NRF54L_BLE_CONTROLLER`` and initialized by the board with
:c:func:`nrf54l_ble_initialize`.

It cannot be used together with the SoftDevice Controller, the ``RADIO``
driver or ``TIMER6`` (TIMER10).

Timers
======

nRF54L has no RTC.  The Link Layer timer is derived from the 1 MHz GRTC
counter: ``ticks = us * 32768 / 1000000``.  GRTC ``CC[2]`` drives the Link
Layer scheduler.  A radio event is started by GRTC ``CC[3]``, which starts
TIMER10 through DPPI and PPIB; TIMER10 ``CC[0]`` then enables the radio.
TIMER10 runs at 1 MHz and is used for the inter frame spacing, the wait for
response timeout and packet timestamps, as TIMER0 on nRF52.

Reserved resources
==================

* ``RADIO``
* ``TIMER10``
* ``GRTC`` interrupt group 2, compare channels 2 and 3
* ``CCM00`` and ``AAR00`` - link encryption and address resolution
* DPPI channels 6 to 12 of DPPIC10, channels 6 and 7 of DPPIC00, channel 8
  of DPPIC20 and the same PPIB10/PPIB00 and PPIB21/PPIB11 channels

Limitations
===========

* AES block encryption uses the software AES library
  (``CONFIG_CRYPTO_SW_AES``), ECB00 is not used.
* Random numbers come from ``arc4random_buf()``; CRACEN is not used.
* There is no hardware filter accept list.
* The radio event delays are the nRF52 values.

Interrupts
==========

``RADIO`` and ``GRTC`` interrupts run at the default priority, so they are
masked by critical sections in the Link Layer.

Clocks
======

The low frequency clock is required (``CONFIG_NRF54L_USE_LFCLK``).  HFXO is
started at boot and stays running.

Configurations
==============

``nrf54lm20-dk:blell_nimble`` and ``nrf54l15-tag:blell_nimble`` run the
NimBLE host from apps on top of the Link Layer.
