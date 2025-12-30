===============
SST26 NOR Flash
===============

NuttX provides support for QSPI/SPI based NOR flashes from SST26 family.
Supported capacity is up to 64 Mbit (8 MB).

Only SPI interface is supported at the moment.

The driver can be enabled by ``CONFIG_MTD_SST26`` option. It is possible
to select the SPI mode with ``CONFIG_SST26_SPIMODE`` option and
communication frequency with ``CONFIG_SST26_SPIFREQUENCY`` option.

Various manufacturers may have produced the parts. ``0xBF`` is the manufacturer
ID for the parts manufactured by SST. This is the default option set by
``CONFIG_SST26_MANUFACTURER``. The same applies for memory type, the
default set by ``CONFIG_SST26_MEMORY_TYPE`` is ``0x26``.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *sst26_initialize_spi(FAR struct spi_dev_s *dev,
                                              uint16_t spi_devid)
