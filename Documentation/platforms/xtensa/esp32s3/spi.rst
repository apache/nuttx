SPI Configuration for ESP32-S3
===============================

This guide explains how to configure SPI on ESP32-S3, addressing common issues
like the one reported in `Issue #12638 <https://github.com/apache/nuttx/issues/12638>`_.

Prerequisites
-------------
- NuttX configured for ESP32-S3
- Basic understanding of the NuttX configuration system

Configuration Steps
-------------------
1. Enable SPI support in menuconfig::

     Device Drivers -> SPI Driver Support -> [*] SPI

2. Select the SPI peripheral::

     System Type -> ESP32-S3 Peripheral Selection -> [*] SPI2

3. Configure additional SPI options as needed::

     Device Drivers -> SPI Driver Support -> [*] SPI Exchange

Common Issues and Solutions
---------------------------
- **SPI not appearing in menuconfig**: Ensure DMA is enabled first
- **Pin conflicts**: Check board pin assignments against your schematic
- **SPI transactions failing**: Verify clock polarity and phase settings

Verification
------------
After configuration, you can verify SPI is working by:

1. Building NuttX with your configuration
2. Running a simple SPI test application
3. Checking the output for expected communication

For more help, refer to the `NuttX mailing list <https://lists.apache.org/list.html?dev@nuttx.apache.org>`_.
