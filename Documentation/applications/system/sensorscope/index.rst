==========================================
``sensorscope`` NxScope sensor data stream
==========================================

Streams data from the sensor framework via ``NxScope``.

See :doc:`/applications/logging/nxscope/index` for more details about
the NxScope library and host-side tools.

Configuration
=============

Enable ``CONFIG_SYSTEM_SENSORSCOPE`` and configure the following:

- ``CONFIG_SYSTEM_SENSORSCOPE_SERIAL_PATH``: Device path (e.g. ``/dev/ttyUSB0``).
- ``CONFIG_SYSTEM_SENSORSCOPE_FETCH_INTERVAL``: Sensor data fetch interval in microseconds.
- ``CONFIG_SYSTEM_SENSORSCOPE_STREAMBUF_LEN``: NxScope stream buffer size.
- ``CONFIG_SYSTEM_SENSORSCOPE_CDCACM``: Use CDC/ACM serial transport.
