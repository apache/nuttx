======================
Nordic nRF53 BLE Radio
======================

.. note::

   This driver is experimental.

Radio lower half for the :doc:`/os/wireless/ble_ll` on the nRF5340
network core, derived from the Apache Mynewt NimBLE nRF5x driver.  It is
enabled with ``CONFIG_NRF53_BLE_CONTROLLER`` and initialized by the board
with :c:func:`nrf53_ble_initialize`, or with
:c:func:`nrf53_ble_rpmsghci_server_initialize` to expose the Link Layer to
the application core through the RPMSG HCI server
(``CONFIG_BLUETOOTH_RPMSG_SERVER``).

It cannot be used together with the SoftDevice Controller.

LE Coded PHY (``CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY``) is available.

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

DPPI channels
=============

DPPI channels 0-5 are used.

Interrupts
==========

``RADIO``, ``RTC0`` and ``RNG`` interrupts run at the default priority, so
they are masked by critical sections in the Link Layer.

Clocks
======

The low frequency clock is required (``CONFIG_NRF53_USE_LFCLK``).  With the
LFXO source the application core must configure the oscillator
(``CONFIG_NRF53_OSCILLATOR_LFXO``).  The high frequency crystal oscillator is
requested when the Link Layer enables the radio clock.

Configurations
==============

``nrf5340-dk:rpmsghci_blell_cpunet`` runs the Link Layer on the network core
behind the RPMSG HCI server.  It pairs with the ``rpmsghci_bt_cpuapp`` or
``rpmsghci_nimble_cpuapp`` application core configurations.
