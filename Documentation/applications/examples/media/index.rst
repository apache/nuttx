``media``
=========

The media test simply writes values onto the media hidden behind a character
driver and verifies that the media can be successfully written and read. This
low level test is useful in the early phases of the bringup of a new block or
mtd driver because it avoids the complexity of a file system.

This test uses a character driver and cannot directly access block or mtd
drivers. This test is suitable for use EEPROM character drivers (see
``nuttx/drivers/eeprom``), or with block drivers wrapped as character drivers (see
``nuttx/drivers/bch``)

.. code-block:: C

  int ret = bchdev_register(<path-to-block-driver>, <path-to-character-driver>,
                            false);

MTD drivers need an additional wrapper layer, the FTL wrapper must first be used
to convert the MTD driver to a block device:

.. code-block:: C

  int ret = ftl_initialize(<N>, mtd);
  ret = bchdev_register(/dev/mtdblock<N>, <path-to-character-driver>, false);
