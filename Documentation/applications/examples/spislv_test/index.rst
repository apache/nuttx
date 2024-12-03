``spislv`` SPI slave example
============================

A simple example for the device functioning as an SPI slave. 
This example can be used to validate communication with another device 
operating as an SPI master. 
This example contains a hardcoded buffer, which is: 
{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
0x0C, 0x0D, 0x0E, 0x0F}. Whenever data is received,
this example will send the same number of bytes received,
consuming the hardcoded buffer in a circular manner.
and the following command is sent:

``spi exch -x 4 deadbeef``

The expected response in device running spislv_test app is: 

.. code-block:: bash

  Queued for sending to master: 01 02 03 04


This test requires the device to be configured in SPI slave mode.(your
specific spi slave hardware settings might require additional settings).

Specific configuration options for this example include:
Configs inside <> are the options that you need to find, It are arch de-
pendent and might be different for your board.

- ``<Enable SPI Peripheral Support>`` – Enables SPI peripheral support
  in System Type->Peripheral Support.

- ``<Configure SPI>`` – Configures SPI peripheral in System Type->SPI
  configuration. Set here the right pins to be used for SPI communication.

- ``CONFIG_SPI`` – In the SPI driver support, enable SPI Slave support.

- ``CONFIG_SPI_SLAVE`` – Enables SPI Slave support.

- ``SPI_SLAVE_DRIVER`` – Enables SPI Slave character driver.

- ``<Configure SPI>`` – Return to this option and enable SPI Slave mode.



