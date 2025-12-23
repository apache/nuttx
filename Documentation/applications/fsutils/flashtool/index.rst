===================================
``flashtool`` Flash Device Utility
===================================

.. _cmdflashtools:

``flashtool`` Manage MTD Flash Devices
======================================

**Command Syntax**::

  flashtool [OPTION [ARG]] ...

**Synopsis**. The ``flashtool`` command is a utility for managing MTD
(Memory Technology Device) flash devices in NuttX. It provides operations
for reading, writing, erasing flash pages and blocks, as well as querying
device information and checking for bad blocks.

**Options**

===================  ====================================================
``-h``               Show help statement with all available options.
``-i``               Display device geometry information (page size,
                     erase block size, number of blocks).
``-r <page>``        Read the specified page(s) from flash and display
                     contents in hexadecimal dump format.
``-w <page>``        Write data to the specified page(s) from a file
                     (requires ``-f`` option).
``-e <block>``       Erase the specified block(s).
``-n <num>``         Specify the number of pages or blocks to operate on
                     (default: 1).
``-f <file>``        Specify the source file for write operations.
``-b``               Check for bad blocks across the entire device.
``-c``               Erase all flash content (bulk erase).
``-d <device>``      Specify the MTD device name (required for all
                     operations).
===================  ====================================================

Display Device Information
--------------------------

To display the geometry information of a flash device::

  nsh> flashtool -i -d /dev/mtd0
  Size of one read/write page: 2048
  Size of one erase block:     131072
  Number of erase blocks:      1024

This displays:

- **Page size**: The size of one read/write page in bytes
- **Erase block size**: The size of one erase block in bytes
- **Number of blocks**: Total number of erase blocks in the device

Read Flash Pages
----------------

To read a single page from flash and display its contents::

  nsh> flashtool -r 0 -d /dev/mtd0
  Flash pages contents:
  Flash page 0:
  0000: ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ................
  0010: ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ................
  ...

To read multiple consecutive pages::

  nsh> flashtool -r 10 -n 5 -d /dev/mtd0

This reads 5 pages starting from page 10 and displays each page's content
in hexadecimal dump format.

Write Flash Pages
-----------------

To write data from a file to flash pages::

  nsh> flashtool -w 0 -n 10 -f /tmp/firmware.bin -d /dev/mtd0
  Write data from /tmp/firmware.bin...

This writes data from ``/tmp/firmware.bin`` to 10 pages starting from page 0.

**Example**::

  nsh> ls -l /tmp
  /tmp:
   -rw-rw-rw-    8192 firmware.bin
  nsh> flashtool -e 0 -n 1 -d /dev/mtd0
  Erase block from 0 to 0!
  nsh> flashtool -w 0 -n 4 -f /tmp/firmware.bin -d /dev/mtd0
  Write data from /tmp/firmware.bin...
  nsh>

Erase Flash Blocks
------------------

To erase a single block::

  nsh> flashtool -e 5 -d /dev/mtd0
  Erase block from 5 to 5!

To erase multiple consecutive blocks::

  nsh> flashtool -e 0 -n 8 -d /dev/mtd0
  Erase block from 0 to 7!

This erases 8 blocks starting from block 0.

Bulk Erase
----------

To erase the entire flash device::

  nsh> flashtool -c -d /dev/mtd0

Check Bad Blocks
----------------

To check for bad blocks across the entire device::

  nsh> flashtool -b -d /dev/mtd0
  Check bad blocks in flash......
  Block 42 is bad!
  Block 157 is bad!

Or if no bad blocks are found::

  nsh> flashtool -b -d /dev/mtd0
  Check bad blocks in flash......
  No bad blocks!

Configuration
-------------

The ``flashtool`` command can be enabled in the NuttX configuration::

  CONFIG_FSUTILS_FLASHTOOL=y
