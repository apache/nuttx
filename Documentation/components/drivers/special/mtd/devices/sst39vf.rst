=================
SST39VF NOR Flash
=================

NuttX provides support for NOR flashes from SST39VF family
Supported capacity is up to 32 Mbit (4 MB).

The driver assumes the device is present at the configured memory address.
This address is configurable by ``CONFIG_SST39VF_BASE_ADDRESS``. The driver
itself is enabled by ``CONFIG_SST39FV``, mind the typo in the option name.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *sst39vf_initialize(void)
