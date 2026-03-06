============================
``sx127x_demo`` SX127X Radio
============================

This example demonstrates the use of the ``SX127X`` radio.

Overview
========

The ``sx127x_demo`` application exercises the `SX127X` low-power wide-area
network (LPWAN) radio driver in both transmit and receive modes.  It is
typically enabled via a board defconfig together with
``CONFIG_LPWAN_SX127X`` and the appropriate SPI and GPIO wiring for the
target module.

Default configuration
=====================

Unless explicitly overridden in the board or application configuration,
the SX127X driver uses the following relevant defaults:

* RF frequency: ``CONFIG_LPWAN_SX127X_RFFREQ_DEFAULT`` (board-specific,
  for example 433 MHz, 868 MHz or 915 MHz).
* TX power: ``CONFIG_LPWAN_SX127X_TXPOWER_DEFAULT``.
  The global default is **17 dBm**, which provides a stronger and more
  reliable link than the previous 14 dBm default.
* CRC: ``CONFIG_LPWAN_SX127X_CRCON``.
  The global default is **1** (CRC enabled), so payloads are protected
  by a checksum and corrupted frames are rejected.

Individual boards may override these values in their ``defconfig`` files.
For example, some boards tested with integrated SX1276 modules may set
``CONFIG_LPWAN_SX127X_TXPOWER_DEFAULT=20`` and adjust
``CONFIG_LPWAN_SX127X_RFFREQ_DEFAULT`` to match the module’s band.

Short-range FSK testing
=======================

When running short-range FSK tests (for example, two modules placed
around one meter apart on a lab bench), the following settings are
recommended:

* Keep ``CONFIG_LPWAN_SX127X_TXPOWER_DEFAULT`` at **17 dBm** or higher
  (some modules work well at 20 dBm) to avoid marginal RSSI values that
  can lead to intermittent reception.
* Ensure ``CONFIG_LPWAN_SX127X_CRCON=1`` on both transmitter and
  receiver so corrupted frames are discarded instead of being printed
  with incorrect data.

These defaults are based on empirical testing reported by users and are
intended to provide robust communication without requiring additional
per-board tuning.

Troubleshooting
===============

If the receiver occasionally shows no data, truncated data, or payloads
that do not match what was transmitted, check the following:

* Verify that the RF frequency and modulation settings are identical on
  both sides.
* Inspect the reported RSSI and SNR values; very low RSSI at short
  distance can indicate insufficient TX power or antenna issues.
* Confirm that CRC is enabled and that both peers are configured
  consistently (either both with CRC enabled or both with it disabled,
  though enabling CRC is recommended).

Adjusting TX power or antenna placement and ensuring consistent CRC
settings on the link typically resolves data integrity problems seen in
simple FSK tests.
