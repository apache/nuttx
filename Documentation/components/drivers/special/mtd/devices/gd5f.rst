===============
GD5F NAND Flash
===============

NuttX provides support for NAND flashes from GD5F family.
The only supported variant is currently 4 Gbit (512 MB) large flash.

The flash is enabled by option ``CONFIG_MTD_GD5F``. It is possible
to select the SPI mode with ``CONFIG_GD5F_SPIMODE`` option and
communication frequency with ``CONFIG_GD5F_SPIFREQUENCY`` option.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *gd5f_initialize(FAR struct spi_dev_s *dev,
                                         uint32_t spi_devid)
