=============
AT24XX EEPROM
=============

NuttX provides support for I2C based AT24XX EEPROMS. These includes
AT24C32, AT24C64, AT24C128 and AT24C256.

The driver is enabled by option ``CONFIG_MTD_AT24XX`. It is possible
to support multiple AT25XX devices by enabling ``CONFIG_AT24XX_MULTI``.
This build in additional support for multiple AT24XX devices, each with
dynamically allocated device structures with a separate I2C
addresses (but otherwise identical -- support for multiple, different
AT24xx, devices not yet supported).

The size of the EEPROM is configured by ``CONFIG_AT24XX_SIZE``.
This is the XX in the AT24Cxx part number.  For example, if you have
an AT24C64, then the correct value is 64. This value is also the capacity
of the part in kilobits. For example, the 64 supports 64 Kbits or
64/8 = 8 KiB.

The static I2C address if single EEPROM is used can be configured by
``CONFIG_AT24XX_ADDR`` option.

If the device supports extended memory, then ``CONFIG_AT24XX_EXTENDED`` may be
set to enable the ``MTDIOC_EXTENDED`` ioctl() operation.  When the
extended operation is selected, calls to the driver read method will
return data from the extended memory region. The extended memory region
size is configured by ``CONFIG_AT24XX_EXTSIZE``.

I2C communication frequency can be set with ``CONFIG_AT24XX_FREQUENCY``.
This value must represent a valid I2C speed (normally less than
400.000) or the driver might fail.

The memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   #ifdef CONFIG_AT24XX_MULTI
   FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_master_s *dev,
                                          uint8_t address)
   #else
   FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_master_s *dev)
   #endif
