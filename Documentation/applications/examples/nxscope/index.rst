===================================
``nxscope`` NxScope library example
===================================

The ``nxscope`` example demonstrates the basic usage of the NxScope
library for real-time data streaming.

See :doc:`/applications/logging/nxscope/index` for more details about
the NxScope library and host-side tools.

This application initializes a set of example channels and starts a thread
that generates various waveforms (sine waves, counters, etc.) to be streamed
to a host client.

Configuration
=============

To use this example, enable ``CONFIG_EXAMPLES_NXSCOPE=y``. The available
configuration options depend on the interface selected in the ``nxscope``
configuration.

Common Options
--------------

- ``CONFIG_EXAMPLES_NXSCOPE_STREAMBUF_LEN``: NxScope stream buffer size.
- ``CONFIG_EXAMPLES_NXSCOPE_RXBUF_LEN``: NxScope RX buffer size.
- ``CONFIG_EXAMPLES_NXSCOPE_MAIN_INTERVAL``: Main loop interval in microseconds.
- ``CONFIG_EXAMPLES_NXSCOPE_FORCE_ENABLE``: Automatically enable all channels
  and start streaming on startup.
- ``CONFIG_EXAMPLES_NXSCOPE_CHARLOG``: Demonstrate sending text messages over
  a dedicated channel (channel 19).

Serial Interface Options
------------------------

These options are available when ``CONFIG_LOGGING_NXSCOPE_INTF_SERIAL=y``:

- ``CONFIG_EXAMPLES_NXSCOPE_SERIAL_PATH``: Device path (e.g. ``/dev/ttyUSB0``).
- ``CONFIG_EXAMPLES_NXSCOPE_SERIAL_BAUD``: Baud rate for the serial interface.
- ``CONFIG_EXAMPLES_NXSCOPE_CDCACM``: Enable USB CDC/ACM serial transport
  support.

UDP Interface Options
---------------------

These options are available when ``CONFIG_LOGGING_NXSCOPE_INTF_UDP=y``:

- ``CONFIG_EXAMPLES_NXSCOPE_UDP_PORT``: Local UDP port for the NxScope
  interface.

Timer Options
-------------

These options are available when ``CONFIG_EXAMPLES_NXSCOPE_TIMER=y``:

- ``CONFIG_EXAMPLES_NXSCOPE_TIMER_PATH``: Device path for the hardware timer.
- ``CONFIG_EXAMPLES_NXSCOPE_TIMER_INTERVAL``: Timer interval in microseconds.
- ``CONFIG_EXAMPLES_NXSCOPE_TIMER_SIGNO``: Signal number for timer
  notifications.

Command-Line Arguments
======================

The example accepts the following command-line arguments:

- ``-i <stream_interval_us>``: Data sampling thread interval in microseconds.
  Overrides the default value (``CONFIG_EXAMPLES_NXSCOPE_TIMER_INTERVAL``
  if timer is used, otherwise 100us).
- ``-m <main_interval_us>``: Main loop interval in microseconds.
  Overrides the default value (``CONFIG_EXAMPLES_NXSCOPE_MAIN_INTERVAL``).

Supported Channels
==================

The example initializes 32 channels to demonstrate different data types and
capabilities:

- **Channels 0-7**: Standard integer types (uint8 to int64).
- **Channels 8-9**: Floating point types (float and double).
- **Channels 10-15**: Fixed-point types (b8, b16, b32).
- **Channel 16**: Vector of floats (3-phase sine waveform).
- **Channel 17**: Vector of floats with metadata.
- **Channel 18**: No-data channel with metadata.
- **Channel 19**: Character channel for text logs (if enabled).
- **Channel 20**: Critical channel (if enabled).
