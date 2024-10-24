``spislv`` SPI slave example
============================

A simple example for the device functioning as an SPI slave. 
This example can be used to validate communication with another device 
operating as an SPI master. If the spitool is used on the other device,
and the following command is sent:

``spi exch -x 4 deadbeef``

The expected response in device running spislv_test app is: 

.. code-block:: bash

  Slave: 4 Bytes reads
  Value in hex form from /dev/spislv2: de ad be ef
  Slave: Writing value back to /dev/spislv2


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



