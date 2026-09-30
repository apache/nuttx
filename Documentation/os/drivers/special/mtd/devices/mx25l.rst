===============
MX25L NOR Flash
===============

NuttX provides support for SPI based NOR flashes from MX25L family
Supported capacity is up to 256 Mbit (32 MB).

The driver can be enabled by ``CONFIG_MTD_MX25L`` option. It is possible
to select the SPI mode with ``CONFIG_MX25L_SPIMODE`` option and
communication frequency with ``CONFIG_MX25L_SPIFREQUENCY`` option.

Some devices (such as the EON EN25F80) support a smaller erase block
size (4K vs 64K). The option ``CONFIG_MX25L_SUBSECTOR_ERASE`` enables
support for sub-sector erase. The SMART file system can take advantage of
this option if it is enabled.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *mx25l_initialize_spi(FAR struct spi_dev_s *dev)
