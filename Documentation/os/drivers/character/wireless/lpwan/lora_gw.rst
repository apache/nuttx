.. _lora_gw:

================================
LoRa gateway (concentrator) API
================================

A LoRa gateway does not listen to one channel at a time as an end device
does: a concentrator demodulates several channels at once, on two radio
front-ends, and every packet arrives with its own frequency, spreading factor
and coding rate. The commands that configure a radio, ``WLIOC_SETRADIOFREQ``
and the ``WLIOC_LORA_x`` family, therefore have nothing to act upon on such a
device, and what it needs instead is a channel plan, a way to be started and
stopped, and the counter its timestamps are taken from.

This is the device independent interface those drivers implement, declared in
``nuttx/wireless/lpwan/lora_gw.h``. An application drives a gateway through it
and never names a chip.

Character device
================

A driver of this class registers a character device, ``/dev/lora0`` by
convention, on which:

* ``read`` returns whole multiples of ``struct lora_gw_rxpkt_s``, oldest
  first, as many as fit in the buffer. It blocks until at least one packet is
  available unless the file was opened with ``O_NONBLOCK``, and fails with
  ``EINVAL`` if the buffer cannot hold one whole packet.

* ``write`` takes exactly one ``struct lora_gw_txpkt_s``. The packet carries
  its own frequency, power, modulation and, for a LoRaWAN downlink,
  ``invert_pol``. It is sent immediately with ``LORA_GW_TX_IMMEDIATE`` or at a
  concentrator timestamp with ``LORA_GW_TX_TIMESTAMPED``.

Both structures follow the layout of the userspace HAL that Semtech publishes
for this family of chips, which is what gateway software is written against
on other systems, so that such an application ports by replacing its
``lgw_receive`` with ``read`` and its ``lgw_send`` with ``write``. Two things
deliberately differ:

.. list-table::
   :header-rows: 1

   * - Field
     - Unit
   * - ``rssi_dbm10``, ``snr_db10``
     - Tenths of a dBm or dB, as integers, so that no floating point is
       needed in a driver
   * - ``datarate``
     - The spreading factor as a plain number, 7 to 12, not a bit mask
   * - ``bandwidth``
     - ``LORA_GW_BW_125K``, ``_250K`` or ``_500K``
   * - ``coderate``
     - ``enum wlioc_lora_cr_e``, shared with the end device drivers
   * - ``status``
     - ``LORA_GW_STAT_CRC_OK``, ``_CRC_BAD`` or ``_NO_CRC``

A packet reported as ``LORA_GW_STAT_CRC_BAD`` must never be forwarded as if
it were valid: those are mostly correlator false triggers.

IOCTL commands
==============

See ``nuttx/wireless/ioctl.h`` : ``WLIOC_GW_x``.

* ``WLIOC_GW_START`` resets the chip, loads the firmware of its internal
  MCUs, calibrates it and starts receiving on the selected channel plan.
  ``WLIOC_GW_STOP`` stops it and ``WLIOC_GW_RESET`` does both in sequence.

* ``WLIOC_GW_SETREGION`` selects a channel plan by name, for example
  ``"AU915"``, while the concentrator is stopped. ``WLIOC_GW_GETREGION``
  takes a ``struct lora_gw_regionreq_s``: an ``index`` of -1 describes the
  active plan, and 0 upwards enumerates the supported ones until ``ENODEV``.
  The description that comes back lists the centre frequency of each radio
  and the frequency, radio and type of every channel.

* ``WLIOC_GW_GETSTATUS`` fills a ``struct lora_gw_status_s`` with the state
  of the concentrator and its counters, including the packets dropped
  because their CRC failed.

* ``WLIOC_GW_GETTRIGCNT`` reads the internal counter of the concentrator, in
  microseconds. This is the time base a LoRaWAN network server schedules
  downlinks against.

Whether the units of the packet should instead follow the ones of the end
device commands, that is, bandwidth in Hz and levels scaled by a hundred as
in ``struct wlioc_rx_hdr_s``, is a question for the common LoRa API rather
than for one driver, and is left as it is until that API materialises.

SX1301 driver
=============

The Semtech SX1301 is the baseband processor of a LoRaWAN gateway: eight
multi-SF demodulators, one LoRa standard demodulator and one FSK
demodulator, driven by two SX125x radio front-ends that are reached through
an SPI bridge inside the SX1301 itself. It is the first implementation of the
interface above and is enabled with ``CONFIG_LPWAN_SX1301``.

Options
-------

* ``CONFIG_LPWAN_SX1301_SPIFREQ`` is the SPI clock, up to 10 MHz.

* ``CONFIG_LPWAN_SX1301_DEFAULT_REGION`` is the channel plan selected when
  the driver is registered: one of AU915, AU915-1, US915, US915-1, EU868,
  AS923, KR920 or IN866. AU915 and US915 default to the second sub-band,
  which is what The Things Network and the Brazilian deployments use.

* ``CONFIG_LPWAN_SX1301_PRIVATE_NETWORK`` switches the frame synchronisation
  word from the public LoRaWAN one to the private one.

* ``CONFIG_LPWAN_SX1301_RXBADCRC`` and ``CONFIG_LPWAN_SX1301_RXNOCRC``
  deliver the packets that a gateway normally drops, which is useful when
  bringing a shield up against an unknown transmitter.

Board implementation
--------------------

The driver is registered with ``sx1301_register``, which takes the device
path, an SPI bus and a ``struct sx1301_lower_s``. That structure carries the
two things the chip needs from the board: a ``reset`` hook driving its reset
line, and an optional ``band_select`` hook for the shields whose front-end
filters are switched between 868 and 915 MHz by a pair of GPIOs. See
``nuttx/wireless/lpwan/sx1301.h``.

A worked example, with the wiring of an LRWAN_GS_HF1 shield, the expected
boot output and a gateway forwarding to a public network, is in the
:ref:`Nucleo F746ZG <nucleo-f746zg>` page.
