====================================
``adcscope`` NxScope ADC data stream
====================================

Streams ADC data from an ADC driver via ``NxScope``.

See :doc:`/applications/logging/nxscope/index` for more details about
the NxScope library and host-side tools.

Configuration
=============

Enable ``CONFIG_SYSTEM_ADCSCOPE`` and configure the following:

- ``CONFIG_SYSTEM_ADCSCOPE_ADC_PATH``: ADC device path (e.g. ``/dev/adc0``).
- ``CONFIG_SYSTEM_ADCSCOPE_SERIAL_PATH``: Serial device path (e.g. ``/dev/ttyUSB0``).
- ``CONFIG_SYSTEM_ADCSCOPE_FETCH_INTERVAL``: ADC data fetch interval in microseconds.
- ``CONFIG_SYSTEM_ADCSCOPE_MAIN_INTERVAL``: Main loop interval in microseconds.
- ``CONFIG_SYSTEM_ADCSCOPE_STREAMBUF_LEN``: NxScope stream buffer size.
- ``CONFIG_SYSTEM_ADCSCOPE_CDCACM``: Use CDC/ACM serial transport.
- ``CONFIG_SYSTEM_ADCSCOPE_SWTRIG``: Use software trigger for ADC (``ANIOC_TRIGGER``).
