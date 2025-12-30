====================
M25P/MT25Q NOR Flash
====================

NuttX provides support for SPI based NOR flashes from M25P/MT25Q family
Supported capacity is up to 1 Gbit (128 MB).

The driver can be enabled by ``CONFIG_MTD_M25P`` option. It is possible
to select the SPI mode with ``CONFIG_M25P_SPIMODE`` option and
communication frequency with ``CONFIG_M25P_SPIFREQUENCY`` option.

Various manufacturers may have produced the parts. ``0x20`` is the manufacturer
ID for the parts manufactured by STMicro. This is the default option set by
``CONFIG_M25P_MANUFACTURER``.  If, for example, you are using the a
Macronix International MX25 serial FLASH, the correct manufacturer ID would be
``0xC2``. Configuration option ``CONFIG_M25P_MEMORY_TYPE`` sets the memory
type value.	The memory type for M25 P series is ``0x20``, but the driver also
supports F series devices, such as the EON EN25F80 part which adds
a 4K sector erase capability.  The memory type for F series parts
from EON is ``0x31``.  The 4K sector erase size will automatically be
enabled when filesystems that can use it are enabled, such as SMART.

Some devices (such as the EON EN25F80) support a smaller erase block
size (4K vs 64K). The option ``CONFIG_M25P_SUBSECTOR_ERASE`` enables
support for sub-sector erase. The SMART file system can take advantage of
this option if it is enabled.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *m25p_initialize(FAR struct spi_dev_s *dev)
