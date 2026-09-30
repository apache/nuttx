=============
AT25EE EEPROM
=============

NuttX provides support for SPI based EEPROM AT25EE. The supported
capacity is 16 Mbit (2 MB). MTD on EEPROM can perform poorly, so it is
possible only usable if the EEPROM has a clock speed 10MHz or higher.
EEPROMs that use the same commands as the 25AA160 should work OK.

The driver is enabled by option ``CONFIG_MTD_AT25EE``.

For applications where a file system is used on the AT25 EEPROM,
the tiny page sizes will result in very inefficient EEPROM usage.
In such cases, it is better if blocks are comprised of "clusters" of
pages so that the file system block size is, say, 128, 256 or
512 bytes. In any event, the block size *must* be an even multiple of the
number of pages and, often, needs to be a factor 2. This is up to the user
to check!

You can configure the EEPROM to use native blocks by selecting
``CONFIG_USE_NATIVE_AT25EE_BLOCK_SIZE`` or set them manually with
``CONFIG_MANUALLY_SET_AT25EE_BLOCK_SIZE`` option. If latter is set,
option ``CONFIG_MANUAL_AT25EE_BLOCK_SIZE`` becomes available.

EEPROM does not need to be erased before write. However, in some
applications (e.g if an erase verify is wanted, or if a particular
file system requires this) block erase (i.e. writing each byte to
0xff) can be enabled by ``CONFIG_AT25EE_ENABLE_BLOCK_ERASE``.

The memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *at25ee_initialize(FAR struct spi_dev_s *dev,
                                           uint16_t spi_devid,
                                           enum eeprom_25xx_e devtype,
                                           int readonly)
