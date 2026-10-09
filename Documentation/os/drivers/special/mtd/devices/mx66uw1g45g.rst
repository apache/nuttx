=====================
MX66UW1G45G NOR Flash
=====================

NuttX provides support for the Octal SPI NOR flash MX66UW1G45G using QSPI
and OPI transactions.

Supported capacity is up to 1 Gbit (128 MB).

The driver can be enabled by ``CONFIG_MTD_MX65UW1G45G`` option. It is possible
to select the SPI mode with ``CONFIG_MX66UW1G45G_SPIMODE`` option and
communication frequency with ``CONFIG_MMX66UW1G45G_QSPI_FREQUENCY`` option.

As this is the first supported chip in the MX66UW family the size of
flash is fixed at 1 Gbit.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *mx66uw1g456_initialize_spi(FAR struct qspi_dev_s *dev)
