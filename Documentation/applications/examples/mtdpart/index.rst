``mtdpart`` MTD Partition Test
==============================

This examples provides a simple test of MTD partition logic.

- ``CONFIG_EXAMPLES_MTDPART`` – Enables the MTD partition test example.

- ``CONFIG_EXAMPLES_MTDPART_ARCHINIT`` – The default is to use the RAM MTD device
  at ``drivers/mtd/rammtd.c``. But an architecture-specific MTD driver can be used
  instead by defining ``CONFIG_EXAMPLES_MTDPART_ARCHINIT``. In this case, the
  initialization logic will call ``mtdpart_archinitialize()`` to obtain the MTD
  driver instance.

- ``CONFIG_EXAMPLES_MTDPART_NPARTITIONS`` – This setting provides the number of
  partitions to test. The test will divide the reported size of the MTD device
  into equal-sized sub-regions for each test partition. Default: ``3``.

When ``CONFIG_EXAMPLES_MTDPART_ARCHINIT`` is not defined, this test will use the
RAM MTD device at ``drivers/mtd/rammtd.c`` to simulate FLASH. The size of the
allocated RAM drive will be: ``CONFIG_EXMPLES_RAMMTD_ERASESIZE *
CONFIG_EXAMPLES_MTDPART_NEBLOCKS``.

* ``CONFIG_EXAMPLES_MTDPART_ERASESIZE`` – This value gives the size of one erase
  block in the MTD RAM device. This must exactly match the default configuration
  in ``drivers/mtd/rammtd.c``!

* ``CONFIG_EXAMPLES_MTDPART_NEBLOCKS`` – This value gives the number of erase
  blocks in MTD RAM device.
