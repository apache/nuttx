====================
Block Device Drivers
====================

Block device drivers have these properties:

-  ``include/nuttx/fs/fs.h``. All structures and APIs needed
   to work with block drivers are provided in this header file.

-  ``struct block_operations``. Each block device driver must
   implement an instance of ``struct block_operations``. That
   structure defines a call table with the following methods:

-  ``int register_blockdriver(const char *path, const struct block_operations *bops, mode_t mode, void *priv);``.
   Each block driver registers itself by calling
   ``register_blockdriver()``, passing it the ``path`` where it
   will appear in the :ref:`pseudo file system <file_system_overview>` and
   it's initialized instance of ``struct block_operations``.

-  **User Access**. Users do not normally access block drivers
   directly, rather, they access block drivers indirectly through
   the ``mount()`` API. The ``mount()`` API binds a block driver
   instance with a file system and with a mountpoint. Then the
   user may use the block driver to access the file system on the
   underlying media. *Example*: See the ``cmd_mount()``
   implementation in ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Character Driver as a Block Device**. See the
   loop device at ``drivers/loop.c``. *Example*: See the
   ``cmd_losetup()`` implementation in
   ``apps/nshlib/nsh_fscmds.c``.

-  **Accessing a Block Driver as Character Device**. See the
   Block-to-Character (BCH) conversion logic in ``drivers/bch/``.
   *Example*: See the ``cmd_dd()`` implementation in
   ``apps/nshlib/nsh_ddcmd.c``.

-  **Examples**. ``drivers/loop.c``,
   ``drivers/mmcsd/mmcsd_spi.c``, ``drivers/ramdisk.c``, etc.

``ramdisk.c``
=============

Can be used to set up a block of memory or (read-only) FLASH as
a block driver that can be mounted as a file system.  See
include/nuttx/drivers/ramdisk.h.

EEPROM
======

EEPROMs are a form of Memory
  Technology Device (MTD).  EEPROMs are non-volatile memory like FLASH, but
  differ in underlying memory technology and differ in usage in many respects:
  They may not be organized into blocks (at least from the standpoint of the
  user) and it is not necessary to erase the EEPROM memory before re-writing
  it.  In addition, EEPROMs tend to be much smaller than FLASH parts, usually
  only a few kilobytes vs megabytes for FLASH.  EEPROM tends to be used to
  retain a small amount of device configuration information; FLASH tends
  to be used for program or massive data storage. For these reasons, it may
  not be convenient to use the more complex MTD interface but instead use
  the simple character interface provided by the EEPROM drivers.

EEPROM Device Support
---------------------

drivers/eeprom/spi_xx25xx.c
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a driver for SPI EEPROMs that use the same commands as the
25AA160::

    Manufacturer Device     Bytes PgSize AddrLen
    Microchip
                 25xx010A     128   16     1
                 25xx020A     256   16     1
                 25AA02UID    256   16     1
                 25AA02E48    256   16     1
                 25AA02E64    256   16     1
                 25xx040      512   16     1+bit
                 25xx040A     512   16     1+bit
                 25xx080     1024   16     1
                 25xx080A    1024   16     2
                 25xx080B    1024   32     2
                 25xx080C    1024   16     x
                 25xx080D    1024   32     x
                 25xx160     2048   16     2
                 25xx160A/C  2048   16     2    TESTED
                 25xx160B/D  2048   32     2
                 25xx160C    2048   16     2
                 25xx160D    2048   32     2
                 25xx320     4096   32     2
                 25xx320A    4096   32     2
                 25xx640     8192   32     2
                 25xx640A    8192   32     2
                 25xx128    16384   64     2
                 25xx256    32768   64     2
                 25xx512    65536  128     2
                 25xx1024  131072  256     3
    Atmel
                 AT25010B     128    8     1
                 AT25020B     256    8     1
                 AT25040B     512    8     1+bit
                 AT25080B    1024   32     2
                 AT25160B    2048   32     2
                 AT25320B    4096   32     2
                 AT25640B    8192   32     2
                 AT25128B   16384   64     2
                 AT25256B   32768   64     2
                 AT25512    65536  128     2
                 AT25M01   131072  256     3

drivers/mtd/at24xx.c
~~~~~~~~~~~~~~~~~~~~

This is a driver for I2C-based at24cxx EEPROM (at24c32, at24c64, at24c128,
at24c256, at24c512).  This driver is currently provided as an MTD driver
but could easily be modified to support the character driver interface.

File Systems
------------

Most EEPROM parts are too small to be candidates for use with a file
system.  The character driver interface is optimal for these small parts
because you can open and access the EEPROM part as if it were a single,
fixed size file.

It is also possible to use these character drivers with a file system.
The character driver can converted to a block device using the NuttX loop
device.  The loop device can be found the file drivers/loop.c.  Interface
function prototypes can be found in include/nuttx/fs/fs.h::

    int losetup(FAR const char *devname, FAR const char *filename,
                uint16_t sectsize, off_t offset, bool readonly);

Given a file or character devices at 'filename', losetup will create the
block device 'devname' using a bogus sector size of sectsize.  'offset' is
normally zero but can be used to provide an offset into the EEPROM where
the block driver data starts;  The EEPROM block driver can also be read-
only.

There is a corresponding function that will destroy the loop device::

    int loteardown(FAR const char *devname);
