==============
BLE Link Layer
==============

``wireless/ble_ll`` is a Bluetooth Low Energy controller (Link Layer)
derived from the `Apache Mynewt NimBLE <https://github.com/apache/mynewt-nimble>`_
controller (commit ``b6831813cfe6da9b25c8be7136b40ce060bf9710``).

The Link Layer is registered as a Bluetooth HCI driver
(``struct bt_driver_s``), so it can be used by any host available in NuttX:
the NuttX Bluetooth stack (``wireless/bluetooth``), the NimBLE host from apps
over a ``PF_BLUETOOTH`` HCI socket or an external host over ``/dev/ttyHCI``
(``CONFIG_UART_BTH4``).

Architecture
============

The controller is split into a generic upper half and a radio specific
lower half::

   host (wireless/bluetooth, NimBLE host, /dev/ttyHCI)
     |
     |  struct bt_driver_s
     v
   wireless/ble_ll               upper half: scheduler, advertising,
     |                           scanning, connections, LL control
     |                           procedures, HCI
     |  struct ble_ll_lower_s    (include/nuttx/wireless/bluetooth/ble_ll.h)
     v
   radio lower half              PHY, timer, AES, filter accept list,
                                 address resolution, RNG

The upper half is registered with ``ble_ll_register()``, called by the
lower half initialization code.  ``ble_ll_initialize()`` initializes the
Link Layer without registering it and returns its ``struct bt_driver_s``,
for transports that take the driver directly (RPMSG HCI server).

The NimBLE OS abstraction is replaced with NuttX facilities:

=============================== ==========================================
NimBLE                          NuttX
=============================== ==========================================
LL task, event queue, callouts  dedicated work queue (``work_queue_create``)
``os_mbuf`` packet buffers      I/O buffers (``mm/iob``)
``hal_timer``, ``os_cputime``   lower-half timer and ``ble_ll_tmr.c``
``OS_ENTER_CRITICAL``           ``enter_critical_section()``
HCI transport                   ``struct bt_driver_s``
``syscfg``                      Kconfig
=============================== ==========================================

Supported features
==================

* Broadcaster, observer, peripheral and central roles
* Legacy advertising and scanning
* Multiple connections
* LE Encryption, LE Ping
* Connection Parameters Request procedure
* Peripheral-initiated Features Exchange
* LE Data Packet Length Extension
* LL Privacy
* LE 2M PHY
* LE Coded PHY (if supported by the radio)
* Channel Selection Algorithm #2
* Controller to Host flow control
* Sleep Clock Accuracy Update procedure
* Extended advertising and scanning, multiple advertising sets
* Periodic advertising and periodic advertising synchronization
* Periodic Advertising Sync Transfer
* Periodic Advertising ADI
* LE Enhanced Connection Update (connection subrating)
* Advertising Coding Selection

Not supported yet: isochronous channels, Direct Test Mode, Channel Sounding,
vendor specific HCI commands and external front-end modules.

Configuration
=============

``CONFIG_WIRELESS_BLE_LL`` enables the Link Layer.  It requires
``CONFIG_DRIVERS_BLUETOOTH`` or ``CONFIG_NET_BLUETOOTH``.  The roles,
features and Link Layer parameters are configured with ``CONFIG_BLE_LL_*``
options.

PDUs are stored in I/O buffers.  ``CONFIG_IOB_NBUFFERS`` must cover the ACL
buffers announced to the host (``CONFIG_BLE_LL_ACL_FROM_HS_COUNT``), the
received PDUs waiting for the Link Layer thread and the control PDUs of all
connections.

The Bluetooth network driver passes each HCI packet in a single I/O buffer,
so ``CONFIG_IOB_BUFSIZE`` must be at least 260 for 251 byte data PDUs.

Advertising reports longer than ``CONFIG_BLE_LL_EVT_SIZE`` are split into
multiple HCI events.  ``CONFIG_BLE_LL_EVT_SIZE`` must not exceed the event
buffer size of the host, which is 70 bytes for the NimBLE host from apps.

Extended advertising requires host support (``CONFIG_NIMBLE_BLE_EXT_ADV``
for the NimBLE host).  A host using a ``PF_BLUETOOTH`` socket is limited to
HCI packets of ``BLUETOOTH_MAX_FRAMELEN`` bytes, which limits the
extended advertising data it can set to about 70 bytes.

The Link Layer thread priority (``CONFIG_BLE_LL_PRIORITY``) must be higher
than other users with scheduling deadlines.

Lower half
==========

A radio lower half provides ``struct ble_ll_lower_s`` with a table of
operations (``struct ble_ll_lower_ops_s``):

* PHY: channel, timed TX/RX start, transmission, wait for response,
  encryption (AES-CCM), address resolution, TX power, PHY mode.
* Hardware support: filter accept list, resolving list, AES-128 block
  encryption, random number generator, device addresses.
* Timer: a free running counter clocked at ``CONFIG_BLE_LL_TMR_FREQ`` with a
  single compare channel.  All Link Layer timers are multiplexed on it.

The block encryption, random number generator and device address operations
are optional.  Without them the upper half uses the software AES library
(``CONFIG_CRYPTO_SW_AES``), ``arc4random_buf()`` and the configured public
address.  A hardware filter accept list is used when the lower half selects
``CONFIG_BLE_LL_HAVE_HW_WHITELIST``.

``bind()`` passes the upper half callbacks
(``struct ble_ll_lowercb_s``).  The callbacks are called from interrupt
context.  The Link Layer protects its state with
``enter_critical_section()``, so the radio and timer interrupts must be
maskable.

``sched_delay_us`` gives the time from a timer event to the start of a
transmission (interrupt processing and radio ramp-up).

Available lower halves:

* nRF52: :doc:`/platforms/arm/nrf52/ble_ll`
* nRF53: :doc:`/platforms/arm/nrf53/ble_ll`
