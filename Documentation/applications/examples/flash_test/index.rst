``flash_test`` SMART Flash test
===============================

Author: Ken Pettit, Date: April 24, 2013

This example performs a SMART flash block device test. This test performs a
sector allocate, read, write, free and garbage collection test on a SMART MTD
block device.

- ``CONFIG_EXAMPLES_FLASH_TEST=y`` – Enables the FLASH Test.

Dependencies:

- ``CONFIG_MTD_SMART=y`` – SMART block driver support.
- ``CONFIG_BUILD_PROTECTED=n`` and ``CONFIG_BUILD_KERNEL=n`` – This test uses
  internal OS interfaces and so is not available in the NUTTX kernel builds.

This application performs a SMART flash block device test. This test performs a
sector allocate, read, write, free and garbage collection test on a SMART MTD
block device. This test can be built only as an NSH command

**Note**: This test uses internal OS interfaces and so is not available in the
NUTTX kernel build::

  Usage:
    flash_test mtdblock_device

  Additional options:
    --force                     to replace existing installation
