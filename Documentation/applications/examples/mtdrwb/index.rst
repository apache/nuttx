=============================================
``mtdrwb`` MTD Read-ahead and Write Buffering
=============================================

This examples provides a simple test of MTD Read-Ahead/Write buffering logic.

- ``CONFIG_EXAMPLES_MTDRWB`` – Enables the MTD R/W buffering test example.

- ``CONFIG_EXAMPLES_MTDRWB_ARCHINIT`` – The default is to use the RAM MTD device
  at ``drivers/mtd/rammtd.c``. But an architecture-specific MTD driver can be used
  instead by defining ``CONFIG_EXAMPLES_MTDRWB_ARCHINIT``. In this case, the
  initialization logic will call ``mtdrwb_archinitialize()`` to obtain the MTD
  driver instance.

When ``CONFIG_EXAMPLES_MTDRWB_ARCHINIT`` is not defined, this test will use the
RAM MTD device at ``drivers/mtd/rammtd.c`` to simulate FLASH. The size of the
allocated RAM drive will be: ``CONFIG_EXMPLES_RAMMTD_ERASESIZE *
CONFIG_EXAMPLES_MTDRWB_NEBLOCKS``

- ``CONFIG_EXAMPLES_MTDRWB_ERASESIZE`` – This value gives the size of one erase
  block in the MTD RAM device. This must exactly match the default configuration
  in ``drivers/mtd/rammtd.c``!

- ``CONFIG_EXAMPLES_MTDRWB_NEBLOCKS`` – This value gives the number of erase
  blocks in MTD RAM device.
