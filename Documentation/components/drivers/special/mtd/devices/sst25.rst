===============
SST25 NOR Flash
===============

NuttX provides support for SPI based NOR flashes from SST25 family
Supported capacity is up to 64 Mbit (8 MB).

-----
SST25
-----

Standard SST25 driver can be enabled by ``CONFIG_MTD_SST25`` option. This
driver supports the capacity up to 32 Mbit (4 MB). It is possible
to select the SPI mode with ``CONFIG_SST25_SPIMODE`` option and
communication frequency with ``CONFIG_SST25_SPIFREQUENCY`` option.

The flash allows to simulate 512-byte large erase blocks if option
``CONFIG_SST25_SECTOR512`` is enabled. You may also configure the flash
as read only if ``CONFIG_SST25_READONLY`` is set.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *sst25_initialize(FAR struct spi_dev_s *dev)

-------
SST25XX
-------

This driver supports SST25 flashes with 64 Mbit and larger capacity.
With the 64 MBit and larger parts, SST changed the write mechanism to
support page write instead of byte/word write like the smaller parts.
As a result, the SST25 driver is not compatible with the larger
density	parts, and the SST25XX driver must be used instead.

This driver is enabled by  ``CONFIG_MTD_SST25XX`` option. It is possible
to select the SPI mode with ``CONFIG_SST25XX_SPIMODE`` option and
communication frequency with ``CONFIG_SST25XX_SPIFREQUENCY`` option.

Various manufacturers may have produced the parts. ``0xBF`` is the manufacturer
ID for the parts manufactured by SST. This is the default option set by
``CONFIG_SST25XX_MANUFACTURER``. The same applies for memory type, the
default set by ``CONFIG_SST25XX_MEMORY_TYPE`` is ``0x25``.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *sst25xx_initialize(FAR struct spi_dev_s *dev)
