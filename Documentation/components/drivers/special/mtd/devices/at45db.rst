================
AT45DB NOR Flash
================

NuttX provides support for SPI based NOR flash AT45DB. The supported
capacity is 16 Mbit (2 MB).

The driver is enabled by option ``CONFIG_MTD_AT64DB``. It is possible
to set communication frequency with ``CONFIG_AT64DB_FREQUENCY`` option.

Option ``CONFIG_AT64DB_PREWAIT`` enables high performance write logic,
optin ``CONFIG_AT64DB_PRWSAVE`` allows to device to enter power save mode.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *at45db_initialize(FAR struct spi_dev_s *spi)
