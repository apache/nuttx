===============
MX25R NOR Flash
===============

NuttX provides support for QSPI based NOR flashes from MX25R family
Supported capacity is up to 256 Mbit (32 MB).

The driver can be enabled by ``CONFIG_MTD_MX25RXX`` option. It is possible
to select the QSPI mode with ``CONFIG_MX25RXX_QSPIMODE`` option and
communication frequency with ``CONFIG_MX25RXX_QSPI_FREQUENCY`` option. This
frequency is used for all commands except for read commands. The speed
for this command is configured by ``CONFIG_MX25RXX_QSPI_READ_FREQUENCY``
option.

The flash allows to simulate 512-byte large erase blocks if option
``CONFIG_MX25RXX_SECTOR512`` is enabled and 128-byte large pages
if ``CONFIG_MX25RXX_PAGE128`` is set.

It is also possible to run the driver in MX25LXX mode by enabling
``CONFIG_MX25RXX_LXX`` option.

The size of the flash is set automatically during the device initialization.

The flash memory has to be initialized before used. This is typically done
from board support package layer during the board's bringup phase. This
operation is performed by following function.

.. code-block:: C

   #include <nuttx/mtd/mtd.h>

   FAR struct mtd_dev_s *mx25rxx_initialize(FAR struct qspi_dev_s *qspi,
                                            bool unprotect)
