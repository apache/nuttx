================
IS25XP NOR Flash
================

NuttX provides support for SPI based NOR flashes from IS25XP family.
Supported capacity is up to 512 Mbit (64 MB).

The driver can be enabled by ``CONFIG_MTD_IS25PX`` option. It is possible
to select the SPI mode with ``CONFIG_IS25PX_SPIMODE`` option and
communication frequency with ``CONFIG_IS25PX_SPIFREQUENCY`` option.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *is25xp_initialize(FAR struct spi_dev_s *dev,
                                           uint16_t spi_devid)
